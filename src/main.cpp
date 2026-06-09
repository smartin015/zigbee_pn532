/**
 * Zigbee NFC Reader/Writer — Xiao ESP32C6 + PN532
 *
 * Acts as a Zigbee End Device (ZED) that bridges NFC tags into a Zigbee
 * network via a manufacturer-specific cluster (0xFC00).
 *
 * ── Zigbee model ───────────────────────────────────────────────────────
 *   Endpoint 1  →  Cluster 0xFC00  (NFC Bridge)
 *     attr 0x0000  nfc_text           (char string, R, reportable)
 *     attr 0x0001  nfc_tag_uid        (octet string, R, reportable)
 *     attr 0x0002  nfc_pending_write  (char string, RW)
 *
 * When a tag is scanned its text + UID are reported to the coordinator.
 * The coordinator can write to nfc_pending_write to queue a tag write —
 * the next tag presented will be programmed with that text.
 *
 * ── Wiring (I2C) ──────────────────────────────────────────────────────
 *   Xiao ESP32C6  →  PN532
 *   ------------------------
 *   3.3V          →  VCC
 *   GND           →  GND
 *   D4  (GPIO4)   →  SDA
 *   D5  (GPIO5)   →  SCL
 *
 * ── Serial console commands ───────────────────────────────────────────
 *   r  read & report tag once
 *   w  prompt for text, write to next tag
 *   c  continuous scan (any key stops)
 *   s  show Zigbee network status
 *   f  factory-reset Zigbee & rejoin
 */

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>

// ── ESP-IDF Zigbee headers ─────────────────────────────────────────────
#include <esp_zb.h>
#include <esp_zb_common.h>
#include <esp_zb_core.h>
#include <esp_zigbee_core.h>
#include <zcl/esp_zb_zcl_common.h>
#include <zcl/esp_zb_zcl_basic.h>
#include <zcl/esp_zb_zcl_identify.h>

// ── Pin definitions ────────────────────────────────────────────────────
#define PN532_SDA  D4
#define PN532_SCL  D5

// ── Zigbee constants ───────────────────────────────────────────────────
#define ZB_EP_NFC          1
#define ZB_CLUSTER_NFC     0xFC00
#define ZB_MANUF_CODE      0x1234
#define ZB_ATTR_NFC_TEXT   0x0000
#define ZB_ATTR_NFC_UID    0x0001
#define ZB_ATTR_NFC_WRITE  0x0002
#define ZB_TEXT_MAX_LEN    128

// ── NDEF constants ─────────────────────────────────────────────────────
#define NDEF_TEXT_RECORD_TYPE  0x54
#define NDEF_TNF_WELL_KNOWN    0x01
#define NDEF_MAX_PAYLOAD       128

// ── Globals ────────────────────────────────────────────────────────────
Adafruit_PN532          nfc(PN532_SDA, PN532_SCL);
static char             g_nfc_text   [ZB_TEXT_MAX_LEN + 1] = "";
static uint8_t          g_nfc_uid    [7] = {0};
static uint8_t          g_nfc_uid_len = 0;
static char             g_pending_write[ZB_TEXT_MAX_LEN + 1] = "";
static bool             g_has_pending_write = false;
static bool             g_zb_joined  = false;

static esp_zb_ep_list_t        *g_ep_list  = nullptr;
static esp_zb_cluster_list_t   *g_clusters = nullptr;

// ── Forward declarations ───────────────────────────────────────────────
static bool   waitForTag   (uint8_t *uid, uint8_t *uidLen, uint16_t toMs);
static void   printUID     (const uint8_t *uid, uint8_t uidLen);
static bool   readTagText  (const uint8_t *uid, uint8_t uidLen,
                            char *out, size_t outSize);
static bool   writeTagText (const uint8_t *uid, uint8_t uidLen,
                            const char *text);
static size_t buildTextNDEF(const char *text, uint8_t *buf, size_t sz);
static bool   parseTextNDEF(const uint8_t *data, size_t len,
                            char *out, size_t outSize);
static bool   readTagData  (const uint8_t *uid, uint8_t uidLen,
                            uint8_t page, uint8_t *buf, uint8_t len);
static bool   writeTagData (const uint8_t *uid, uint8_t uidLen,
                            uint8_t page, const uint8_t *data, uint8_t len);
static void   zb_setup     ();
static void   zb_report    ();
static void   zb_signal_handler(esp_zb_app_signal_type_t sig,
                                esp_zb_app_signal_t *params);
static void   console_read ();
static void   console_write();
static void   console_scan ();
static void   console_status();
static void   console_factory_reset();

// ==========================================================================
//  NDEF helpers
// ==========================================================================

static size_t buildTextNDEF(const char *text, uint8_t *buf, size_t sz) {
    size_t tLen = strlen(text);
    size_t pl   = 1 + 2 + tLen;
    size_t tot  = 2 + 1 + 1 + pl;
    if (tot > sz) return 0;

    uint8_t *p = buf;
    *p++ = (1 << 7) | (1 << 6) | (1 << 4) | NDEF_TNF_WELL_KNOWN;
    *p++ = 0x01;
    *p++ = (uint8_t)pl;
    *p++ = NDEF_TEXT_RECORD_TYPE;
    *p++ = 0x02;
    *p++ = 'e'; *p++ = 'n';
    memcpy(p, text, tLen);
    return tot;
}

static bool parseTextNDEF(const uint8_t *data, size_t len,
                          char *out, size_t outSize) {
    if (len < 8) return false;
    const uint8_t *p = data;
    uint8_t flags = *p++;
    if ((flags & 0x07) != NDEF_TNF_WELL_KNOWN) return false;
    bool sr = flags & (1 << 4);
    uint8_t typeLen = *p++;

    uint32_t payloadLen;
    if (sr) { payloadLen = *p++; }
    else    { payloadLen = ((uint32_t)p[0]<<24)|((uint32_t)p[1]<<16)|
                           ((uint32_t)p[2]<<8)|(uint32_t)p[3]; p += 4; }

    if (typeLen != 1 || *p++ != NDEF_TEXT_RECORD_TYPE) return false;
    if (payloadLen < 3) return false;

    uint8_t status  = *p++;
    uint8_t langLen = status & 0x3F;
    if (payloadLen < (uint32_t)(1 + langLen)) return false;
    p += langLen;

    uint32_t textLen = payloadLen - 1 - langLen;
    if (textLen >= outSize) textLen = outSize - 1;
    memcpy(out, p, textLen);
    out[textLen] = '\0';
    return true;
}

// ==========================================================================
//  Tag I/O helpers
// ==========================================================================

static bool waitForTag(uint8_t *uid, uint8_t *uidLen, uint16_t toMs) {
    return nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, uidLen, toMs);
}

static void printUID(const uint8_t *uid, uint8_t uidLen) {
    Serial.print(F("UID: "));
    for (uint8_t i = 0; i < uidLen; i++) {
        if (uid[i] < 0x10) Serial.print('0');
        Serial.print(uid[i], HEX);
        Serial.print(' ');
    }
    Serial.println();
}

static bool readTagData(const uint8_t *uid, uint8_t uidLen,
                        uint8_t page, uint8_t *buf, uint8_t len) {
    uint8_t pages = (len + 3) / 4;
    for (uint8_t i = 0; i < pages; i++)
        if (!nfc.ntag2xx_ReadPage(page + i, buf + (i * 4)))
            return false;
    return true;
}

static bool writeTagData(const uint8_t *uid, uint8_t uidLen,
                         uint8_t page, const uint8_t *data, uint8_t len) {
    uint8_t pages = (len + 3) / 4;
    for (uint8_t i = 0; i < pages; i++) {
        uint8_t buf[4] = {0};
        uint8_t rem = len - i * 4;
        memcpy(buf, data + i * 4, rem < 4 ? rem : 4);
        if (!nfc.ntag2xx_WritePage(page + i, buf)) return false;
    }
    return true;
}

static bool readTagText(const uint8_t *uid, uint8_t uidLen,
                        char *out, size_t outSize) {
    uint8_t raw[NDEF_MAX_PAYLOAD + 16] = {0};
    if (!readTagData(uid, uidLen, 4, raw, sizeof(raw))) return false;
    return parseTextNDEF(raw, sizeof(raw), out, outSize);
}

static bool writeTagText(const uint8_t *uid, uint8_t uidLen,
                         const char *text) {
    uint8_t ndef[NDEF_MAX_PAYLOAD + 16] = {0};
    size_t nLen = buildTextNDEF(text, ndef, sizeof(ndef));
    if (!nLen) return false;
    return writeTagData(uid, uidLen, 4, ndef, nLen);
}

// ==========================================================================
//  Zigbee — attribute read / write callbacks
// ==========================================================================

static esp_zb_zcl_status_t zb_read_attr(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx)
{
    uint16_t id = attr->attribute.id;
    Serial.printf("Zb attr read req: cluster=0x%04X attr=0x%04X\n",
                  attr->cluster_id, id);

    if (id == ZB_ATTR_NFC_TEXT) {
        esp_zb_zcl_set_attribute_val(
            endpoint, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_TEXT, g_nfc_text, false);
    } else if (id == ZB_ATTR_NFC_UID) {
        esp_zb_zcl_set_attribute_val(
            endpoint, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_UID, g_nfc_uid, false);
    } else if (id == ZB_ATTR_NFC_WRITE) {
        esp_zb_zcl_set_attribute_val(
            endpoint, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_WRITE, g_pending_write, false);
    }
    return ESP_ZB_ZCL_STATUS_SUCCESS;
}

static esp_zb_zcl_status_t zb_write_attr(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx)
{
    uint16_t id = attr->attribute.id;
    Serial.printf("Zb attr write req: cluster=0x%04X attr=0x%04X\n",
                  attr->cluster_id, id);

    if (id == ZB_ATTR_NFC_WRITE) {
        const char *val = (const char *)attr->data.value;
        size_t len = attr->data.size;
        if (len > ZB_TEXT_MAX_LEN) len = ZB_TEXT_MAX_LEN;
        memcpy(g_pending_write, val, len);
        g_pending_write[len] = '\0';
        g_has_pending_write = (len > 0);

        Serial.print(F("Zb: queued write text = \""));
        Serial.print(g_pending_write);
        Serial.println('"');

        esp_zb_zcl_set_attribute_val(
            endpoint, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_WRITE, g_pending_write, false);
        return ESP_ZB_ZCL_STATUS_SUCCESS;
    }
    return ESP_ZB_ZCL_STATUS_READ_ONLY;
}

// ==========================================================================
//  Zigbee — signal handler (network join / leave)
// ==========================================================================

static void zb_signal_handler(esp_zb_app_signal_type_t sig,
                              esp_zb_app_signal_t *params)
{
    switch (sig) {

    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        Serial.println(F("Zb: starting BDB commissioning…"));
        esp_zb_bdb_start_top_level_commissioning(
            ESP_ZB_BDB_MODE_NETWORK_STEERING);
        break;

    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (params->bdb_commissioning_status == ESP_ZB_BDB_SUCCESS) {
            Serial.println(F("Zb: network joined ✓"));
            g_zb_joined = true;
        } else {
            Serial.printf("Zb: steering failed (err=%d)\n",
                          params->bdb_commissioning_status);
            esp_zb_scheduler_alarm(
                (esp_zb_callback_t)esp_zb_bdb_start_top_level_commissioning,
                ESP_ZB_BDB_MODE_NETWORK_STEERING, 5000);
        }
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        Serial.println(F("Zb: left network"));
        g_zb_joined = false;
        break;

    case ESP_ZB_ZDO_SIGNAL_LEAVE_INDICATION: {
        auto *leave = (esp_zb_zdo_signal_leave_indication_t *)
                      params->signal_data;
        Serial.printf("Zb: device left — short=0x%04X ieee=",
                      leave->device_short_addr);
        for (int i = 7; i >= 0; i--)
            Serial.printf("%02X", leave->ieee_addr[i]);
        Serial.println();
        break;
    }

    default:
        Serial.printf("Zb: signal %d\n", (int)sig);
        break;
    }
}

// ==========================================================================
//  Zigbee — report to coordinator
// ==========================================================================

static void zb_report() {
    if (!g_zb_joined) return;

    esp_zb_zcl_set_attribute_val(
        ZB_EP_NFC, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ATTR_NFC_TEXT, g_nfc_text, false);

    esp_zb_zcl_set_attribute_val(
        ZB_EP_NFC, ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ZB_ATTR_NFC_UID, g_nfc_uid, false);

    esp_zb_zcl_report_attr_cmd_t report = {};
    report.address_mode = ESP_ZB_APS_ADDR_MODE_64_ENDP_PRESENT;
    report.zcl_basic_cmd.dst_addr_u.addr_short = 0x0000;
    report.zcl_basic_cmd.dst_endpoint = 1;
    report.clusterID  = ZB_CLUSTER_NFC;
    report.attributeID = ZB_ATTR_NFC_TEXT;
    report.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;

    esp_zb_zcl_report_attr_req(&report);
}

// ==========================================================================
//  Zigbee — initialise endpoint + cluster
// ==========================================================================

static void zb_setup() {
    Serial.println(F("Zb: configuring end-device…"));

    g_clusters = esp_zb_cluster_list_create();

    esp_zb_zcl_basic_cluster_add_attr(g_clusters);
    esp_zb_zcl_identify_cluster_add_attr(g_clusters);
    esp_zb_basic_cluster_add_attr(g_clusters);

    esp_zb_custom_cluster_cfg_t nfc_cfg = {};
    nfc_cfg.cluster_id   = ZB_CLUSTER_NFC;
    nfc_cfg.attr_read_cb = zb_read_attr;
    nfc_cfg.attr_write_cb = zb_write_attr;
    esp_zb_custom_cluster_add_cb(g_clusters, &nfc_cfg);

    esp_zb_custom_cluster_add_attr(g_clusters, ZB_CLUSTER_NFC,
        ZB_ATTR_NFC_TEXT,  ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        g_nfc_text);

    esp_zb_custom_cluster_add_attr(g_clusters, ZB_CLUSTER_NFC,
        ZB_ATTR_NFC_UID,  ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
        g_nfc_uid);

    esp_zb_custom_cluster_add_attr(g_clusters, ZB_CLUSTER_NFC,
        ZB_ATTR_NFC_WRITE, ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
        ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
        g_pending_write);

    esp_zb_endpoint_config_t ep_cfg = {};
    ep_cfg.endpoint       = ZB_EP_NFC;
    ep_cfg.app_profile_id = 0x0104;
    ep_cfg.app_device_id  = 0x0000;
    ep_cfg.app_device_version = 0;

    g_ep_list = esp_zb_ep_list_create();
    esp_zb_ep_list_add_ep(g_ep_list, g_clusters, ep_cfg);

    esp_zb_device_register(g_ep_list);
    esp_zb_set_primary_network_channel_set(ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK);

    esp_zb_sleep_enable();
    esp_zb_set_rx_on_when_idle(false);

    esp_zb_app_signal_handler_register(zb_signal_handler);

    Serial.println(F("Zb: starting stack (ZED)…"));
    esp_zb_start(false);
    Serial.println(F("Zb: stack started, searching for network…"));
}

// ==========================================================================
//  Serial console commands
// ==========================================================================

static void console_read() {
    Serial.println(F("Bring a tag close…"));
    uint8_t uid[7], uidLen;
    if (!waitForTag(uid, &uidLen, 5000)) {
        Serial.println(F("Timeout."));
        return;
    }
    printUID(uid, uidLen);

    char text[ZB_TEXT_MAX_LEN + 1] = "";
    if (readTagText(uid, uidLen, text, sizeof(text))) {
        Serial.print(F("Text: ")); Serial.println(text);
        strncpy(g_nfc_text, text, ZB_TEXT_MAX_LEN);
        memcpy(g_nfc_uid, uid, uidLen);
        g_nfc_uid_len = uidLen;
        zb_report();
    } else {
        Serial.println(F("No NDEF text record."));
    }
}

static void console_write() {
    Serial.println(F("Enter text (end with newline):"));
    String in;
    uint32_t dl = millis() + 30000;
    while (millis() < dl) {
        if (Serial.available()) { in = Serial.readStringUntil('\n'); in.trim(); break; }
        delay(10);
    }
    if (!in.length()) { Serial.println(F("Cancelled.")); return; }

    Serial.print(F("Will write: ")); Serial.println(in);
    Serial.println(F("Bring a tag…"));

    uint8_t uid[7], uidLen;
    if (!waitForTag(uid, &uidLen, 10000)) {
        Serial.println(F("Timeout.")); return;
    }
    printUID(uid, uidLen);

    if (writeTagText(uid, uidLen, in.c_str())) {
        Serial.println(F("✓ Written."));
        g_has_pending_write = false;
        g_pending_write[0] = '\0';
    } else {
        Serial.println(F("✗ Write failed."));
    }
}

static void console_scan() {
    Serial.println(F("Continuous scan — any key to stop."));
    uint8_t lastUID[7] = {0};
    uint8_t lastUIDLen  = 0;

    while (!Serial.available()) {
        uint8_t uid[7], uidLen;
        if (!waitForTag(uid, &uidLen, 200)) {
            memset(lastUID, 0, 7);
            lastUIDLen = 0;
            delay(100);
            continue;
        }
        if (uidLen == lastUIDLen && !memcmp(uid, lastUID, uidLen)) {
            delay(100); continue;
        }
        memcpy(lastUID, uid, uidLen);
        lastUIDLen = uidLen;

        printUID(uid, uidLen);

        char text[ZB_TEXT_MAX_LEN + 1] = "";
        if (readTagText(uid, uidLen, text, sizeof(text))) {
            Serial.print(F("  ")); Serial.println(text);
            strncpy(g_nfc_text, text, ZB_TEXT_MAX_LEN);
            memcpy(g_nfc_uid, uid, uidLen);
            g_nfc_uid_len = uidLen;
            zb_report();
        }

        if (g_has_pending_write && g_pending_write[0]) {
            Serial.print(F("  -> writing pending: "));
            Serial.println(g_pending_write);
            if (writeTagText(uid, uidLen, g_pending_write)) {
                Serial.println(F("  ✓ written."));
            } else {
                Serial.println(F("  x write failed."));
            }
            g_has_pending_write = false;
            g_pending_write[0]  = '\0';
        }
        delay(300);
    }
    while (Serial.available()) Serial.read();
    Serial.println(F("\nStopped."));
}

static void console_status() {
    Serial.println(F("-- Zigbee NFC Bridge Status --"));
    Serial.print(F("  Network: "));
    Serial.println(g_zb_joined ? F("joined ✓") : F("not joined x"));
    Serial.print(F("  NFC text:   \"")); Serial.print(g_nfc_text);
    Serial.println('"');
    Serial.print(F("  NFC UID:    "));
    if (g_nfc_uid_len) printUID(g_nfc_uid, g_nfc_uid_len);
    else Serial.println(F("(none)"));
    Serial.print(F("  Pending write: "));
    if (g_has_pending_write) {
        Serial.print('"'); Serial.print(g_pending_write); Serial.println('"');
    } else {
        Serial.println(F("(none)"));
    }
}

static void console_factory_reset() {
    Serial.println(F("Factory reset — clearing Zigbee NVS…"));
    esp_zb_factory_reset();
    delay(1000);
    Serial.println(F("Rebooting…"));
    ESP.restart();
}

// ==========================================================================
//  Arduino entry points
// ==========================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println(F("\n=== Zigbee NFC Bridge — Xiao ESP32C6 + PN532 ==="));

    nfc.begin();
    uint32_t ver = nfc.getFirmwareVersion();
    if (!ver) {
        Serial.println(F("ERROR: PN532 not found. Halting."));
        Serial.println(F("Check SDA->D4, SCL->D5, 3.3V, GND."));
        while (1) delay(1000);
    }
    Serial.printf("PN532: chip=0x%02lX  fw=%lu.%lu\n",
                  (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
    nfc.SAMConfig();

    zb_setup();

    Serial.println(F("\nCommands:  r)ead  w)rite  c)ontinuous scan  s)tatus  f)actory-reset"));
    Serial.println(F("Ready.\n"));
}

void loop() {
    // Pending write from Zigbee coordinator?
    if (g_has_pending_write && g_pending_write[0]) {
        Serial.print(F("Pending write from Zb: \""));
        Serial.print(g_pending_write);
        Serial.println(F("\" — bring a tag…"));

        uint8_t uid[7], uidLen;
        if (waitForTag(uid, &uidLen, 30000)) {
            printUID(uid, uidLen);
            if (writeTagText(uid, uidLen, g_pending_write)) {
                Serial.println(F("✓ Written via Zigbee."));
                char txt[ZB_TEXT_MAX_LEN + 1] = "";
                if (readTagText(uid, uidLen, txt, sizeof(txt))) {
                    strncpy(g_nfc_text, txt, ZB_TEXT_MAX_LEN);
                }
                memcpy(g_nfc_uid, uid, uidLen);
                g_nfc_uid_len = uidLen;
                zb_report();
            } else {
                Serial.println(F("x Zigbee-initiated write failed."));
            }
        } else {
            Serial.println(F("Timeout — write cancelled."));
        }
        g_has_pending_write = false;
        g_pending_write[0]  = '\0';
    }

    if (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'r': while (Serial.available()) Serial.read(); console_read();   break;
            case 'w': while (Serial.available()) Serial.read(); console_write();  break;
            case 'c': while (Serial.available()) Serial.read(); console_scan();   break;
            case 's': while (Serial.available()) Serial.read(); console_status(); break;
            case 'f': while (Serial.available()) Serial.read(); console_factory_reset(); break;
            default:  while (Serial.available()) Serial.read(); break;
        }
    }

    delay(10);
}
