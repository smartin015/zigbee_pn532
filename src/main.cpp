/**
 * Zigbee NFC Reader/Writer — Xiao ESP32C6 + PN532
 *
 * Acts as a Zigbee Router bridging NFC tag text data into a Zigbee
 * network.  Built on the Arduino Zigbee library (Zigbee.h).
 *
 * Supports hot-plugging the PN532 module: the device boots and joins the
 * Zigbee network regardless of whether the PN532 is attached, then
 * periodically polls for it.  A boolean attribute is reported so the
 * coordinator always knows whether the reader is reachable.
 *
 * ── Zigbee model ───────────────────────────────────────────────────────
 *   Endpoint 1  →  Cluster 0xFC00  (NFC Bridge, manufacturer-specific)
 *     attr 0x0000  nfc_text           char string   R, reportable
 *     attr 0x0001  nfc_tag_uid        octet string  R, reportable
 *     attr 0x0002  nfc_pending_write  char string   RW
 *     attr 0x0003  nfc_reader_present bool          R, reportable
 *     attr 0x0007  nfc_last_read_ts   uint32        R, reportable
 *     attr 0x0008  nfc_last_write_ts  char string   R, reportable
 *     attr 0x0009  nfc_last_seen_ts   char string   R, reportable
 *     attr 0x000A  nfc_buzzer_trigger   bool          RW
 *
 * ── Wiring (I2C) ──────────────────────────────────────────────────────
 *   Xiao ESP32C6  →  PN532
 *   ------------------------
 *   3.3V          →  VCC
 *   GND           →  GND
 *   D4  (GPIO4)   →  SDA
 *   D5  (GPIO5)   →  SCL
 *
 */

#include <Arduino.h>
#ifndef ZIGBEE_MODE_ZCZR
#error "Zigbee coordinator/router mode is not selected in Tools->Zigbee mode"
#endif

#include "Zigbee.h"
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <time.h>
#include "esp_timer.h"
#include "esp_mac.h"
#include "esp_ieee802154.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "zboss_api.h"
#ifdef __cplusplus
}
#endif

// ── Pin definitions ────────────────────────────────────────────────────
#define PN532_SDA       D4
#define PN532_SCL       D5
#define BUZZER_PIN      D7    // piezo buzzer (passive or active)
#define BUZZER_GND      D8
#define PN532_I2C_ADDR  0x24   // PN532 default I2C address (7-bit)
#define NFC_ENDPOINT    1

// ── Custom cluster constants ───────────────────────────────────────────
#define ZB_CLUSTER_NFC             0xFC00
#define ZB_ATTR_NFC_TEXT           0x0000
#define ZB_ATTR_NFC_UID            0x0001
#define ZB_ATTR_NFC_WRITE          0x0002
#define ZB_ATTR_NFC_READER_PRESENT 0x0003
#define ZB_ATTR_NFC_LAST_READ_TS   0x0007
#define ZB_ATTR_NFC_LAST_WRITE_TS  0x0008
#define ZB_ATTR_NFC_LAST_SEEN_TS   0x0009
#define ZB_ATTR_NFC_BUZZER_TRIGGER      0x000A
#define ZB_TEXT_MAX_LEN                 128

// ── NDEF constants ─────────────────────────────────────────────────────
#define NDEF_TEXT_RECORD_TYPE  0x54
#define NDEF_TNF_WELL_KNOWN    0x01
#define NDEF_MAX_PAYLOAD       128
#define NFC_READ_BUF_SIZE      160   // pages 4..43 — covers full ZB_TEXT_MAX_LEN NDEF

// ── Globals (Zigbee string format: first byte = length) ────────────────
Adafruit_PN532  nfc(PN532_SDA, PN532_SCL);

// Zigbee char string: [len_byte][data...][null_terminator]
static uint8_t  g_nfc_text_buf     [ZB_TEXT_MAX_LEN + 2] = {0};  // len + data + null
static uint8_t  g_nfc_uid_buf      [8] = {0};                    // len + 7 max UID
static uint8_t  g_nfc_uid_len       = 0;
static uint8_t  g_pending_write_buf[ZB_TEXT_MAX_LEN + 2] = {0};  // len + data + null
static bool     g_has_pending_write = false;


#define ZB_TS_STR_LEN            21    // "YYYY-MM-DDTHH:MM:SSZ" includes null
static uint8_t  g_last_read_ts_buf [ZB_TS_STR_LEN + 2] = {0};  // len + text + null
static uint8_t  g_last_write_ts_buf[ZB_TS_STR_LEN + 2] = {0};
static uint8_t  g_last_seen_ts_buf      [ZB_TS_STR_LEN + 2] = {0};

// Buzzer trigger — set true by Z2M, cleared after beep
static bool     g_buzzer_trigger = false;

// Time sync — offset between Zigbee UTC and local millis()
static time_t   g_time_sync_utc     = 0;    // Unix UTC seconds at last sync
static uint32_t g_time_sync_millis  = 0;    // local millis() at last sync
static uint32_t g_last_time_sync_ms = 0;    // millis() when we last attempted sync
#define TIME_SYNC_INTERVAL_MS   (60 * 60 * 1000)   // re-sync every hour

// ── Debug output: swallowed when no USB host is connected ─────────────
class NullStream : public Print {
    size_t write(uint8_t) override { return 1; }
    size_t write(const uint8_t *buf, size_t size) override { return size; }
};
static NullStream g_null_out;
static Print      &g_out = Serial;   // reassigned to g_null_out if headless

// Continuous read mode — always on at boot
static bool     g_continuous_read = true;

// PN532 hot-plug state
static bool     g_nfc_reader_present = false;
static uint32_t g_nfc_last_check_ms  = 0;
#define NFC_PRESENCE_CHECK_INTERVAL_MS  5000

// ── Helpers ───────────────────────────────────────────────────────────

// Short piezo beep.  Uses tone() for passive buzzers; also works with
// active buzzers (just drives a square wave).
static void beep(unsigned int durationMs = 100, unsigned int freq = 2400) {
    tone(BUZZER_PIN, freq, durationMs);
}

// Compute current UTC time from last sync + elapsed millis.
// Returns 0 if time has never been synced.
static time_t getCurrentUtc() {
    if (g_time_sync_utc == 0) return 0;
    uint32_t elapsed = millis() - g_time_sync_millis;
    return g_time_sync_utc + (time_t)(elapsed / 1000);
}

// Write a C string into a Zigbee char string buffer (length-prefixed)
static void zbStringWrite(uint8_t *zbuf, const char *src, size_t maxDataLen) {
    size_t slen = strlen(src);
    if (slen > maxDataLen) slen = maxDataLen;
    zbuf[0] = (uint8_t)slen;
    memcpy(zbuf + 1, src, slen);
    zbuf[1 + slen] = '\0';  // null terminator for C convenience
}

// ==========================================================================
//  Custom Zigbee endpoint for NFC bridge
// ==========================================================================

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

        // ── Custom NFC cluster (0xFC00) ──────────────────────────────
        esp_zb_attribute_list_t *nfc_attr_list =
            esp_zb_zcl_attr_list_create(ZB_CLUSTER_NFC);

        // Attr 0x0000: nfc_text (char string, read + reportable)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_TEXT,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_nfc_text_buf);

        // Attr 0x0001: nfc_tag_uid (octet string, read + reportable)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_UID,
            ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_nfc_uid_buf);

        // Attr 0x0002: nfc_pending_write (char string, read+write)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_WRITE,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            g_pending_write_buf);

        // Attr 0x0003: nfc_reader_present (bool, read + reportable)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_READER_PRESENT,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            &g_nfc_reader_present);

        // Attr 0x0007: nfc_last_read_ts (char string, read + reportable)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_LAST_READ_TS,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_last_read_ts_buf);

        // Attr 0x0008: nfc_last_write_ts (char string, read + reportable)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_LAST_WRITE_TS,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_last_write_ts_buf);

        // Attr 0x0009: nfc_last_seen_ts (char string, read + reportable)
        // Updated whenever any tag is detected, even without NDEF text.
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_LAST_SEEN_TS,
            ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY | ESP_ZB_ZCL_ATTR_ACCESS_REPORTING,
            g_last_seen_ts_buf);

        // Attr 0x000A: nfc_buzzer_trigger (bool, RW)
        // Write true to sound the buzzer; firmware clears after beep.
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_BUZZER_TRIGGER,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &g_buzzer_trigger);

        esp_zb_cluster_list_add_custom_cluster(
            _cluster_list, nfc_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);

        _ep_config = {
            .endpoint         = _endpoint,
            .app_profile_id   = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id    = ESP_ZB_HA_SIMPLE_SENSOR_DEVICE_ID,
            .app_device_version = 0
        };
    }

    // ── Public helpers ────────────────────────────────────────────────

    bool setNfcText(const char *text) {
        zbStringWrite(g_nfc_text_buf, text, ZB_TEXT_MAX_LEN);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_TEXT, g_nfc_text_buf, false);
        g_out.printf("Zb set attr 0x0000 (text) -> %s (val=\"%s\")\n",
                      ret == ESP_ZB_ZCL_STATUS_SUCCESS ? "OK" : "FAIL", text);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setNfcUid(const uint8_t *uid, uint8_t len) {
        if (len > 7) len = 7;
        g_nfc_uid_buf[0] = len;
        memcpy(g_nfc_uid_buf + 1, uid, len);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_UID, g_nfc_uid_buf, false);
        g_out.printf("Zb set attr 0x0001 (uid) len=%d -> %s\n",
                      len, ret == ESP_ZB_ZCL_STATUS_SUCCESS ? "OK" : "FAIL");
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setPendingWrite(const char *text) {
        zbStringWrite(g_pending_write_buf, text, ZB_TEXT_MAX_LEN);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_WRITE, g_pending_write_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportNfcText() {
        return reportAttr(ZB_ATTR_NFC_TEXT);
    }

    bool reportNfcUid() {
        return reportAttr(ZB_ATTR_NFC_UID);
    }

    bool setReaderPresent(bool present) {
        g_nfc_reader_present = present;
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_READER_PRESENT, &g_nfc_reader_present, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportReaderPresent() {
        return reportAttr(ZB_ATTR_NFC_READER_PRESENT);
    }

    // Format a Unix UTC time_t as ISO 8601 into the attribute buffer
    static void formatISO8601(time_t utc, uint8_t *zbuf) {
        struct tm *t = gmtime(&utc);
        if (t && zbuf) {
            int len = snprintf((char *)(zbuf + 1), ZB_TS_STR_LEN,
                               "%04d-%02d-%02dT%02d:%02d:%02dZ",
                               t->tm_year + 1900, t->tm_mon + 1, t->tm_mday,
                               t->tm_hour, t->tm_min, t->tm_sec);
            zbuf[0] = (uint8_t)(len > 0 ? len : 0);
            zbuf[1 + zbuf[0]] = '\0';
        }
    }

    bool setLastReadTs(time_t utc) {
        formatISO8601(utc, g_last_read_ts_buf);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_LAST_READ_TS, g_last_read_ts_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportLastReadTs() {
        return reportAttr(ZB_ATTR_NFC_LAST_READ_TS);
    }

    bool setLastWriteTs(time_t utc) {
        formatISO8601(utc, g_last_write_ts_buf);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_LAST_WRITE_TS, g_last_write_ts_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportLastWriteTs() {
        return reportAttr(ZB_ATTR_NFC_LAST_WRITE_TS);
    }

    bool setLastSeenTs(time_t utc) {
        formatISO8601(utc, g_last_seen_ts_buf);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_LAST_SEEN_TS, g_last_seen_ts_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool reportLastSeenTs() {
        return reportAttr(ZB_ATTR_NFC_LAST_SEEN_TS);
    }

private:
    bool reportAttr(uint16_t attr_id) {
        uint32_t t0 = micros();
        esp_zb_zcl_report_attr_cmd_t cmd = {};
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd.attributeID  = attr_id;
        cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.clusterID    = ZB_CLUSTER_NFC;
        cmd.zcl_basic_cmd.src_endpoint = _endpoint;
        cmd.manuf_specific = 0x00U;
        cmd.dis_default_resp = 0x01U;  // disable Default Response — shaves one poll round-trip
        bool ok = reportClusterAttribute(&cmd);
        uint32_t dt = micros() - t0;
        g_out.printf("Zb report attr 0x%04X on cluster 0x%04X -> %s (%lu us)\n",
                      attr_id, ZB_CLUSTER_NFC, ok ? "OK" : "FAIL", (unsigned long)dt);
        return ok;
    }

    // ── Capture Zigbee Time cluster updates for ISO 8601 timestamps ──
    void zbReadTimeCluster(const esp_zb_zcl_attribute_t *attribute) override {
        if (attribute->id == ESP_ZB_ZCL_ATTR_TIME_TIME_ID
            && attribute->data.type == ESP_ZB_ZCL_ATTR_TYPE_UTC_TIME
            && attribute->data.value != nullptr) {
            uint32_t zb_utc = *(const uint32_t *)attribute->data.value;
            // Zigbee UTC epoch → Unix epoch (offset = 946684800 seconds)
            g_time_sync_utc    = (time_t)zb_utc + 946684800ULL;
            g_time_sync_millis = millis();
            g_out.printf("Time synced: Unix UTC=%lu\n", (unsigned long)g_time_sync_utc);
        }
        ZigbeeEP::zbReadTimeCluster(attribute);  // let base class give the semaphore
    }

    // ── Handle ZCL write attribute ───────────────────────────────────
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override {
        if (message->info.cluster == ZB_CLUSTER_NFC) {
            if (message->attribute.id == ZB_ATTR_NFC_WRITE) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING
                    && message->attribute.data.value != nullptr) {
                    const uint8_t *src = (const uint8_t *)message->attribute.data.value;
                    // zigbee-herdsman may double-encode: [outer_len][inner_len][data…]
                    // where outer_len == inner_len + 1.  Detect & unwrap.
                    if (message->attribute.data.size >= 3
                        && message->attribute.data.size == (uint16_t)src[0] + 1
                        && src[0] == src[1] + 1
                        && src[1] <= ZB_TEXT_MAX_LEN) {
                        src++;
                    }
                    uint8_t srcLen = src[0];  // ZCL char string: [len][data…]
                    if (srcLen > ZB_TEXT_MAX_LEN) srcLen = ZB_TEXT_MAX_LEN;
                    memcpy(g_pending_write_buf + 1, src + 1, srcLen);
                    g_pending_write_buf[0] = srcLen;
                    g_pending_write_buf[1 + srcLen] = '\0';
                    g_has_pending_write = (srcLen > 0);

                    if (g_has_pending_write) {
                        g_out.print(F("Zb: queued write text = \""));
                        g_out.print((const char *)(g_pending_write_buf + 1));
                        g_out.println('"');
                    } else {
                        g_out.println(F("Zb: pending write cancelled (empty string)"));
                    }
                }
            } else if (message->attribute.id == ZB_ATTR_NFC_BUZZER_TRIGGER) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
                    && message->attribute.data.value != nullptr) {
                    bool trig = *(const bool *)message->attribute.data.value;
                    if (trig) {
                        g_out.println(F("Zb: buzzer triggered"));
                        beep(150);
                        g_buzzer_trigger = false;
                    }
                }
            } else {
                log_w("Zb attr write: unknown attr 0x%04X on cluster 0x%04X",
                      message->attribute.id, message->info.cluster);
            }
        } else {
            log_w("Zb attr write: unknown cluster 0x%04X", message->info.cluster);
        }
    }
};

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
    g_out.print(F("UID: "));
    for (uint8_t i = 0; i < uidLen; i++) {
        if (uid[i] < 0x10) g_out.print('0');
        g_out.print(uid[i], HEX);
        g_out.print(' ');
    }
    g_out.println();
}

static bool readTagData(const uint8_t *uid, uint8_t uidLen,
                        uint8_t page, uint8_t *buf, uint8_t len) {
    (void)uid; (void)uidLen;
    uint8_t pages = (len + 3) / 4;
    for (uint8_t i = 0; i < pages; i++) {
        if (!nfc.ntag2xx_ReadPage(page + i, buf + (i * 4)))
            return false;
        // Use busy-wait delayMicroseconds — delay() is stretched by tickless idle
        delayMicroseconds(2000);
    }
    return true;
}

static bool writeTagData(const uint8_t *uid, uint8_t uidLen,
                         uint8_t page, const uint8_t *data, uint8_t len) {
    (void)uid; (void)uidLen;
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
    uint8_t raw[NFC_READ_BUF_SIZE] = {0};

    if (readTagData(uid, uidLen, 4, raw, sizeof(raw)))
        return parseTextNDEF(raw, sizeof(raw), out, outSize);

    return false;
}

static bool writeTagText(const uint8_t *uid, uint8_t uidLen,
                         const char *text) {
    uint8_t ndef[NDEF_MAX_PAYLOAD + 16] = {0};
    size_t nLen = buildTextNDEF(text, ndef, sizeof(ndef));
    if (!nLen) return false;

    if (!writeTagData(uid, uidLen, 4, ndef, nLen))
        return false;

    // Timestamp the write
    nfcEp.setLastWriteTs(getCurrentUtc());
    nfcEp.reportLastWriteTs();

    return true;
}

static void updateNfcState(const uint8_t *uid, uint8_t uidLen,
                           const char *text) {
    uint32_t t_start = micros();
    g_nfc_uid_len = uidLen;
    g_out.println(F("Zb: updateNfcState — setting + reporting text + UID"));

    uint32_t t0 = micros();
    nfcEp.setNfcText(text);
    g_out.printf("  setNfcText: %lu us\n", (unsigned long)(micros() - t0));

    t0 = micros();
    nfcEp.setNfcUid(uid, uidLen);
    g_out.printf("  setNfcUid: %lu us\n", (unsigned long)(micros() - t0));

    t0 = micros();
    nfcEp.reportNfcText();
    g_out.printf("  reportNfcText: %lu us\n", (unsigned long)(micros() - t0));

    t0 = micros();
    nfcEp.reportNfcUid();
    g_out.printf("  reportNfcUid: %lu us\n", (unsigned long)(micros() - t0));

    // Timestamp the read
    t0 = micros();
    nfcEp.setLastReadTs(getCurrentUtc());
    g_out.printf("  setLastReadTs: %lu us\n", (unsigned long)(micros() - t0));

    t0 = micros();
    nfcEp.reportLastReadTs();
    g_out.printf("  reportLastReadTs: %lu us\n", (unsigned long)(micros() - t0));

    // Force turbo poll for the next 3 received packets (APS ACKs).
    // This collapses poll latency from the long-poll interval (100ms) down
    // to the turbo-poll interval (100ms) for the responses to these reports.
    zb_zdo_pim_start_turbo_poll_packets(3);

    g_out.printf("  updateNfcState total: %lu us\n", (unsigned long)(micros() - t_start));
}

// ==========================================================================
//  Serial console commands
// ==========================================================================

// Dump raw tag memory for debugging
static void console_dump() {
    if (!g_nfc_reader_present) {
        g_out.println(F("PN532 not present."));
        return;
    }
    g_out.println(F("Bring a tag…"));
    uint8_t uid[7], uidLen;
    if (!waitForTag(uid, &uidLen, 5000)) {
        g_out.println(F("Timeout.")); return;
    }
    printUID(uid, uidLen);
    nfcEp.setLastSeenTs(getCurrentUtc());
    nfcEp.reportLastSeenTs();

    // Read pages 3-10 (page 3=OTP, pages 4+ = user data)
    uint8_t raw[8 * 4] = {0};
    if (!readTagData(uid, uidLen, 3, raw, sizeof(raw))) {
        g_out.println(F("Read failed.")); return;
    }
    for (uint8_t pg = 0; pg < 8; pg++) {
        g_out.printf("  Pg%02d: ", pg + 3);
        for (uint8_t b = 0; b < 4; b++) {
            uint8_t val = raw[pg * 4 + b];
            if (val < 0x10) g_out.print('0');
            g_out.print(val, HEX); g_out.print(' ');
        }
        g_out.print("  ");
        for (uint8_t b = 0; b < 4; b++) {
            uint8_t val = raw[pg * 4 + b];
            g_out.write(val >= 0x20 && val < 0x7F ? (char)val : '.');
        }
        g_out.println();
    }
}

static void console_help() {
    g_out.println(F("\nCommands:"));
    g_out.println(F("  s — show Zigbee & PN532 status"));
    g_out.println(F("  f — factory‑reset Zigbee & rejoin"));
    g_out.println(F("  x — reboot the MCU"));
    g_out.println(F("  b — beep"));
    g_out.println(F("  ? — this help"));
    g_out.println();
}

static void console_status() {
    g_out.println(F("-- Zigbee NFC Bridge Status --"));
    g_out.print(F("  Network: "));
    g_out.println(Zigbee.connected() ? F("joined ✓") : F("not joined x"));
    g_out.print(F("  PN532 reader: "));
    g_out.println(g_nfc_reader_present ? F("present ✓") : F("absent x"));
    g_out.print(F("  NFC text:   \""));
    g_out.print((const char *)(g_nfc_text_buf + 1));
    g_out.println('"');
    g_out.print(F("  NFC UID:    "));
    if (g_nfc_uid_len) printUID(g_nfc_uid_buf + 1, g_nfc_uid_len);
    else g_out.println(F("(none)"));
    g_out.print(F("  Last seen:  "));
    g_out.println(g_last_seen_ts_buf[0] ? (const char *)(g_last_seen_ts_buf + 1) : "(none)");
    g_out.print(F("  Last read:  "));
    g_out.println(g_last_read_ts_buf[0] ? (const char *)(g_last_read_ts_buf + 1) : "(none)");
    g_out.print(F("  Last write: "));
    g_out.println(g_last_write_ts_buf[0] ? (const char *)(g_last_write_ts_buf + 1) : "(none)");
    g_out.print(F("  Pending write: "));
    if (g_has_pending_write && g_pending_write_buf[0]) {
        g_out.print('"'); g_out.print((const char *)(g_pending_write_buf + 1)); g_out.println('"');
    } else {
        g_out.println(F("(none)"));
    }
    g_out.println();
}

// ==========================================================================
//  PN532 presence detection (hot-plug)
// ==========================================================================

// Silent I2C bus probe — no error spam when PN532 is absent.
// On first successful detection after absence (or at boot) SAMConfig is run.
static bool checkNfcPresence() {
    static bool was_present = false;

    Wire.beginTransmission(PN532_I2C_ADDR);
    uint8_t err = Wire.endTransmission();
    bool present = (err == 0);

    // If the I2C bus timed out it may be stuck; reset the peripheral.
    if (err == 5) {
        Wire.end();
        delay(5);
        nfc.begin();  // re-init with correct pins
    }

    if (present && !was_present) {
        // Freshly attached — configure SAM
        nfc.SAMConfig();
        g_out.println(F("PN532 detected — SAM configured"));
    } else if (!present && was_present) {
        g_out.println(F("PN532 lost"));
    }

    was_present = present;
    return present;
}

// Call periodically from loop(); reports changes to the Zigbee attribute.
static void pollNfcPresence() {
    if (millis() - g_nfc_last_check_ms < NFC_PRESENCE_CHECK_INTERVAL_MS) return;
    g_nfc_last_check_ms = millis();

    bool now = checkNfcPresence();
    if (now != g_nfc_reader_present) {
        g_nfc_reader_present = now;
        nfcEp.setReaderPresent(now);
        nfcEp.reportReaderPresent();
        g_out.printf("PN532 reader present: %s\n", now ? "yes" : "no");
    }
}

// ==========================================================================
//  Arduino entry points
// ==========================================================================

void setup() {
    pinMode(WIFI_ANT_CONFIG, OUTPUT);
    digitalWrite(WIFI_ANT_CONFIG, HIGH);

    Serial.begin(115200);
    for (uint32_t wait = millis(); millis() - wait < 2000 && !Serial;) delay(10);
    if (!Serial) g_out = g_null_out;
    g_out.println(F("\n=== Zigbee NFC Bridge — Xiao ESP32C6 + PN532 ==="));

    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUZZER_GND, OUTPUT);
    digitalWrite(BUZZER_GND, LOW);
    digitalWrite(BUZZER_PIN, LOW);
    beep(30);

    nfc.begin();
    // Prevent I2C bus stalls from blocking the entire device.
    // Normal PN532 transactions complete in < 5 ms; 50 ms is generous.
    Wire.setTimeOut(50);
    Wire.setClock(100000);
    uint32_t ver = nfc.getFirmwareVersion();
    if (ver) {
        g_out.printf("PN532: chip=0x%02lX  fw=%lu.%lu\n",
                      (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
        nfc.SAMConfig();
        g_nfc_reader_present = true;
    } else {
        g_out.println(F("PN532 not found at boot — will poll for hot-plug…"));
        g_nfc_reader_present = false;
    }
    g_nfc_last_check_ms = millis();
    beep(30);

    nfcEp.setManufacturerAndModel("Espressif", "ZigbeeNFCEndpoint");
    nfcEp.addTimeCluster();
    nfcEp.setPowerSource(ZB_POWER_SOURCE_MAINS);
    Zigbee.addEndpoint(&nfcEp);
    g_out.println(F("Starting Zigbee (Router)…"));
    if (!Zigbee.begin(ZIGBEE_ROUTER)) {
        g_out.println(F("Zigbee failed to start! Rebooting…"));
        ESP.restart();
    }

    beep(30);

    g_out.println(F("Zigbee started, connecting to network…"));
    {
        uint32_t joinStart = millis();
        while (!Zigbee.connected()) {
            if (millis() - joinStart > 20000) {
                g_out.println(F("\nZigbee join timeout — rebooting…"));
                ESP.restart();
            }
            g_out.print('.');
            delay(250);
        }
    }
    g_out.println();
    g_out.println(F("Connected ✓"));

    // Ensure rx_on_when_idle so APS ACKs and ZCL responses are
    // delivered directly (no poll-based indirect transmission).
    esp_zb_set_rx_on_when_idle(true);
    beep(30);

    g_out.print(F("Syncing time from coordinator… "));
    if (g_time_sync_utc == 0) {
        struct tm now = nfcEp.getTime(1, 0x0000);
        if (now.tm_year > 0) {
            g_time_sync_utc    = mktime(&now);
            g_time_sync_millis = millis();
            g_out.printf("OK (Unix UTC=%lu)\n", (unsigned long)g_time_sync_utc);
        } else {
            g_out.println(F("failed — timestamps will be empty until next sync"));
        }
    } else {
        g_out.printf("already synced (Unix UTC=%lu)\n", (unsigned long)g_time_sync_utc);
    }
    g_last_time_sync_ms = millis();
    beep(30);

    nfcEp.setReaderPresent(g_nfc_reader_present);
    nfcEp.reportReaderPresent();

    g_out.println(F("Ready.\n"));
    beep(50);
}

void loop() {
    // ── Periodic time re-sync (hourly) ──
    if (millis() - g_last_time_sync_ms > TIME_SYNC_INTERVAL_MS) {
        g_out.print(F("Re-syncing time… "));
        struct tm now = nfcEp.getTime(1, 0x0000);
        if (now.tm_year > 0 && g_time_sync_utc == 0) {
            g_time_sync_utc    = mktime(&now);
            g_time_sync_millis = millis();
        }
        g_last_time_sync_ms = millis();
        g_out.println(g_time_sync_utc ? F("OK") : F("failed"));
    }

    // ── Poll PN532 presence (hot-plug detection) ──
    pollNfcPresence();

    // ── Continuous read mode (always on at boot) ──
    if (g_continuous_read && g_nfc_reader_present) {
        // lastGoodUID  = tag we already processed successfully (debounce)
        // lastAnnUID   = tag we already announced (no repeat beeps/prints)
        static uint8_t lastGoodUID[7] = {0};
        static uint8_t lastGoodUIDLen  = 0;
        static uint8_t lastAnnUID[7]   = {0};
        static uint8_t lastAnnUIDLen   = 0;

        uint8_t uid[7], uidLen;
        if (waitForTag(uid, &uidLen, 200)) {
            bool isNewAnnounce = (uidLen != lastAnnUIDLen
                                  || memcmp(uid, lastAnnUID, uidLen));
            bool isNewGood     = (uidLen != lastGoodUIDLen
                                  || memcmp(uid, lastGoodUID, uidLen));

            // Announce new tag once (print UID, beep, timestamp)
            if (isNewAnnounce) {
                memcpy(lastAnnUID, uid, uidLen);
                lastAnnUIDLen = uidLen;
                printUID(uid, uidLen);
                nfcEp.setLastSeenTs(getCurrentUtc());
                nfcEp.reportLastSeenTs();
                beep(80);
            }

            // Process if we haven't successfully handled this tag yet
            if (isNewGood) {
                // Priority: pending write from Zigbee coordinator?
                if (g_has_pending_write && g_pending_write_buf[0]) {
                    g_out.print(F("  -> writing pending: "));
                    g_out.println((const char *)(g_pending_write_buf + 1));
                    if (writeTagText(uid, uidLen, (const char *)(g_pending_write_buf + 1))) {
                        g_out.println(F("  ✓ written."));
                        memcpy(lastGoodUID, uid, uidLen);
                        lastGoodUIDLen = uidLen;
                        // Read back what we just wrote
                        char txt[ZB_TEXT_MAX_LEN + 1] = "";
                        if (readTagText(uid, uidLen, txt, sizeof(txt))) {
                            updateNfcState(uid, uidLen, txt);
                        }
                    } else {
                        g_out.println(F("  x write failed — will retry."));
                        // Don't set lastGoodUID → retry same tag next loop
                    }
                    g_has_pending_write = false;
                    g_pending_write_buf[0] = 0;
                    g_pending_write_buf[1] = '\0';
                    nfcEp.setPendingWrite("");
                } else {
                    // No pending write — read the tag
                    char text[ZB_TEXT_MAX_LEN + 1] = "";
                    if (readTagText(uid, uidLen, text, sizeof(text))) {
                        g_out.print(F("  ")); g_out.println(text);
                        updateNfcState(uid, uidLen, text);
                        memcpy(lastGoodUID, uid, uidLen);
                        lastGoodUIDLen = uidLen;
                        beep(80);   // successful text read beep
                    } else {
                        // No NDEF text yet — report UID anyway, then retry
                        g_out.println(F("  (no NDEF text — will retry)"));
                        nfcEp.setNfcUid(uid, uidLen);
                        nfcEp.reportNfcUid();
                        // Don't set lastGoodUID → retry same tag next loop
                    }
                }
            }
            // else: same tag already processed successfully → skip
        } else {
            // No tag in range — reset all debounce memory
            memset(lastGoodUID, 0, 7);
            lastGoodUIDLen = 0;
            memset(lastAnnUID, 0, 7);
            lastAnnUIDLen = 0;
        }
    }

    // ── Serial commands ──
    if (Serial.available()) {
        switch ((char)Serial.read()) {
            case 's': while (Serial.available()) Serial.read(); console_status(); break;
            case '?': while (Serial.available()) Serial.read(); console_help();   break;
            case 'b': while (Serial.available()) Serial.read(); beep(80); g_out.println("beep");   break;
            case 'f':
                while (Serial.available()) Serial.read();
                g_out.println(F("Factory reset…"));
                Zigbee.factoryReset();
                break;
            case 'x':
                while (Serial.available()) Serial.read();
                g_out.println(F("Rebooting…"));
                delay(100);
                ESP.restart();
                break;
            default:  while (Serial.available()) Serial.read(); break;
        }
    }

    // ── Long‑press factory reset (BOOT button, 3 s) ──
    static uint32_t btnPressStart = 0;
    if (digitalRead(BOOT_PIN) == LOW) {
        if (!btnPressStart) btnPressStart = millis();
        else if (millis() - btnPressStart > 3000) {
            g_out.println(F("Factory reset via BOOT button…"));
            Zigbee.factoryReset();
        }
    } else {
        btnPressStart = 0;
    }

    // Busy-wait to avoid tickless idle stretching (delay() → vTaskDelay can stretch to 1000ms)
    delayMicroseconds(2000);
}
