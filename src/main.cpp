/**
 * Zigbee NFC Reader/Writer — Xiao ESP32C6 + PN532
 *
 * Acts as a Zigbee End Device bridging NFC tag text data into a Zigbee
 * network.  Built on the Arduino Zigbee library (Zigbee.h).
 *
 * ── Zigbee model ───────────────────────────────────────────────────────
 *   Endpoint 1  →  Cluster 0xFC00  (NFC Bridge, manufacturer-specific)
 *     attr 0x0000  nfc_text           char string   R, reportable
 *     attr 0x0001  nfc_tag_uid        octet string  R, reportable
 *     attr 0x0002  nfc_pending_write  char string   RW
 *
 * ── Wiring (I2C) ──────────────────────────────────────────────────────
 *   Xiao ESP32C6  →  PN532
 *   ------------------------
 *   3.3V          →  VCC
 *   GND           →  GND
 *   D4  (GPIO4)   →  SDA
 *   D5  (GPIO5)   →  SCL
 *
 * ── Serial console ────────────────────────────────────────────────────
 *   r  read & report tag once
 *   w  prompt for text, write to next tag
 *   c  continuous scan (any key stops)
 *   s  show Zigbee network status
 *   f  factory‑reset Zigbee & rejoin
 */

#include <Arduino.h>
#ifndef ZIGBEE_MODE_ED
#error "Zigbee end device mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <Adafruit_PN532.h>

// ── Pin definitions ────────────────────────────────────────────────────
#define PN532_SDA       D4
#define PN532_SCL       D5
#define NFC_ENDPOINT    1

// ── Custom cluster constants ───────────────────────────────────────────
#define ZB_CLUSTER_NFC       0xFC00
#define ZB_ATTR_NFC_TEXT     0x0000
#define ZB_ATTR_NFC_UID      0x0001
#define ZB_ATTR_NFC_WRITE    0x0002
#define ZB_TEXT_MAX_LEN      128

// ── NDEF constants ─────────────────────────────────────────────────────
#define NDEF_TEXT_RECORD_TYPE  0x54
#define NDEF_TNF_WELL_KNOWN    0x01
#define NDEF_MAX_PAYLOAD       128

// ── Globals ────────────────────────────────────────────────────────────
Adafruit_PN532  nfc(PN532_SDA, PN532_SCL);
static char     g_nfc_text        [ZB_TEXT_MAX_LEN + 1] = "";
static uint8_t  g_nfc_uid         [7] = {0};
static uint8_t  g_nfc_uid_len      = 0;
static char     g_pending_write   [ZB_TEXT_MAX_LEN + 1] = "";
static bool     g_has_pending_write = false;

// ==========================================================================
//  Custom Zigbee endpoint for NFC bridge
// ==========================================================================

// Forward declarations for static callbacks
static esp_zb_zcl_status_t nfc_attr_read(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx);
static esp_zb_zcl_status_t nfc_attr_write(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx);

class NfcEndpoint : public ZigbeeEP {
public:
    NfcEndpoint(uint8_t ep = NFC_ENDPOINT)
        : ZigbeeEP(ep)
    {
        _device_id = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID;

        // ── Build cluster list ───────────────────────────────────────
        _cluster_list = esp_zb_zcl_cluster_list_create();

        esp_zb_cluster_list_add_basic_cluster(
            _cluster_list,
            esp_zb_basic_cluster_create(nullptr),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
        esp_zb_cluster_list_add_identify_cluster(
            _cluster_list,
            esp_zb_identify_cluster_create(nullptr),
            ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        // ── Custom NFC cluster ───────────────────────────────────────
        esp_zb_custom_cluster_cfg_t nfc_cfg = {};
        nfc_cfg.cluster_id    = ZB_CLUSTER_NFC;
        nfc_cfg.attr_read_cb  = nfc_attr_read;
        nfc_cfg.attr_write_cb = nfc_attr_write;
        esp_zb_custom_cluster_add_cb(_cluster_list, &nfc_cfg);

        esp_zb_custom_cluster_add_attr(_cluster_list, ZB_CLUSTER_NFC,
            ZB_ATTR_NFC_TEXT,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY |
                ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_nfc_text);

        esp_zb_custom_cluster_add_attr(_cluster_list, ZB_CLUSTER_NFC,
            ZB_ATTR_NFC_UID,
            ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY |
                ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_nfc_uid);

        esp_zb_custom_cluster_add_attr(_cluster_list, ZB_CLUSTER_NFC,
            ZB_ATTR_NFC_WRITE,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            g_pending_write);

        _ep_config = {
            .endpoint         = _endpoint,
            .app_profile_id   = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id    = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
    }

    // ── Public helpers ────────────────────────────────────────────────

    bool setNfcText(const char *text) {
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_TEXT, (void *)text, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setNfcUid(const uint8_t *uid, uint8_t len) {
        uint8_t buf[8] = {0};
        buf[0] = len;
        memcpy(buf + 1, uid, len);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_UID, buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setPendingWrite(const char *text) {
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_WRITE, (void *)text, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportNfcText() {
        esp_zb_zcl_report_attr_cmd_t cmd = {};
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd.attributeID  = ZB_ATTR_NFC_TEXT;
        cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.clusterID    = ZB_CLUSTER_NFC;
        cmd.zcl_basic_cmd.src_endpoint = _endpoint;
        return reportClusterAttribute(&cmd);
    }
};

// ── Static callbacks for the custom cluster ─────────────────────────────

static esp_zb_zcl_status_t nfc_attr_read(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx)
{
    Serial.printf("Zb attr read req: attr=0x%04X\n", attr->attribute.id);
    return ESP_ZB_ZCL_STATUS_SUCCESS;
}

static esp_zb_zcl_status_t nfc_attr_write(
    esp_zb_zcl_addr_t *src, uint8_t endpoint,
    const esp_zb_zcl_attribute_t *attr, void *ctx)
{
    uint16_t id = attr->attribute.id;
    Serial.printf("Zb attr write req: attr=0x%04X\n", id);

    if (id == ZB_ATTR_NFC_WRITE) {
        const uint8_t *data = (const uint8_t *)attr->data.value;
        size_t len = attr->data.size;
        if (len > 0) {
            uint8_t strLen = data[0];
            if (strLen > ZB_TEXT_MAX_LEN) strLen = ZB_TEXT_MAX_LEN;
            memcpy(g_pending_write, data + 1, strLen);
            g_pending_write[strLen] = '\0';
            g_has_pending_write = (strLen > 0);
        } else {
            g_pending_write[0] = '\0';
            g_has_pending_write = false;
        }

        Serial.print(F("Zb: queued write text = \""));
        Serial.print(g_pending_write);
        Serial.println('"');
        return ESP_ZB_ZCL_STATUS_SUCCESS;
    }
    return ESP_ZB_ZCL_STATUS_READ_ONLY;
}

// ── Global endpoint instance ────────────────────────────────────────────
NfcEndpoint nfcEp(NFC_ENDPOINT);

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

static void updateNfcState(const uint8_t *uid, uint8_t uidLen,
                           const char *text) {
    strncpy(g_nfc_text, text, ZB_TEXT_MAX_LEN);
    g_nfc_text[ZB_TEXT_MAX_LEN] = '\0';
    memcpy(g_nfc_uid, uid, uidLen);
    g_nfc_uid_len = uidLen;
    nfcEp.setNfcText(g_nfc_text);
    nfcEp.setNfcUid(g_nfc_uid, g_nfc_uid_len);
    nfcEp.reportNfcText();
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
        updateNfcState(uid, uidLen, text);
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
            updateNfcState(uid, uidLen, text);
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
    Serial.println(Zigbee.connected() ? F("joined ✓") : F("not joined x"));
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

// ==========================================================================
//  Arduino entry points
// ==========================================================================

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);
    Serial.println(F("\n=== Zigbee NFC Bridge — Xiao ESP32C6 + PN532 ==="));

    // ── PN532 ──
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

    // ── Zigbee ──
    nfcEp.setManufacturerAndModel("Espressif", "ZigbeeNFCEndpoint");

    Zigbee.addEndpoint(&nfcEp);

    Serial.println(F("Starting Zigbee (End Device)…"));
    if (!Zigbee.begin()) {
        Serial.println(F("Zigbee failed to start! Rebooting…"));
        ESP.restart();
    }
    Serial.println(F("Zigbee started, connecting to network…"));
    while (!Zigbee.connected()) {
        Serial.print('.');
        delay(100);
    }
    Serial.println();
    Serial.println(F("Connected ✓"));

    Serial.println(F("\nCommands:  r)ead  w)rite  c)ontinuous scan  s)tatus  f)actory-reset"));
    Serial.println(F("Ready.\n"));
}

void loop() {
    // ── Pending write from Zigbee coordinator? ──
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
                    updateNfcState(uid, uidLen, txt);
                }
            } else {
                Serial.println(F("x Zigbee-initiated write failed."));
            }
        } else {
            Serial.println(F("Timeout — write cancelled."));
        }
        g_has_pending_write = false;
        g_pending_write[0]  = '\0';
    }

    // ── Serial commands ──
    if (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'r': while (Serial.available()) Serial.read(); console_read();   break;
            case 'w': while (Serial.available()) Serial.read(); console_write();  break;
            case 'c': while (Serial.available()) Serial.read(); console_scan();   break;
            case 's': while (Serial.available()) Serial.read(); console_status(); break;
            case 'f':
                while (Serial.available()) Serial.read();
                Serial.println(F("Factory reset…"));
                Zigbee.factoryReset();
                break;
            default:  while (Serial.available()) Serial.read(); break;
        }
    }

    // ── Long‑press factory reset (BOOT button, 3 s) ──
    static uint32_t btnPressStart = 0;
    if (digitalRead(BOOT_PIN) == LOW) {
        if (!btnPressStart) btnPressStart = millis();
        else if (millis() - btnPressStart > 3000) {
            Serial.println(F("Factory reset via BOOT button…"));
            Zigbee.factoryReset();
        }
    } else {
        btnPressStart = 0;
    }

    delay(10);
}
