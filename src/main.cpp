/**
 * Zigbee NFC Reader/Writer — Xiao ESP32C6 + PN532
 *
 * Acts as a Zigbee End Device bridging NFC tag text data into a Zigbee
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
 *     attr 0x0004  nfc_auth_pwd       octet string  RW   (4 bytes)
 *     attr 0x0005  nfc_auth_pack      octet string  RW   (2 bytes)
 *     attr 0x0006  nfc_auth_enabled   bool          RW
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
 *   s  show Zigbee & PN532 status
 *   f  factory‑reset Zigbee & rejoin
 *   ?  help
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
#define PN532_I2C_ADDR  0x24   // PN532 default I2C address (7-bit)
#define NFC_ENDPOINT    1

// ── Custom cluster constants ───────────────────────────────────────────
#define ZB_CLUSTER_NFC             0xFC00
#define ZB_ATTR_NFC_TEXT           0x0000
#define ZB_ATTR_NFC_UID            0x0001
#define ZB_ATTR_NFC_WRITE          0x0002
#define ZB_ATTR_NFC_READER_PRESENT 0x0003
#define ZB_ATTR_NFC_AUTH_PWD       0x0004
#define ZB_ATTR_NFC_AUTH_PACK      0x0005
#define ZB_ATTR_NFC_AUTH_ENABLED   0x0006
#define ZB_TEXT_MAX_LEN            128

// ── NDEF constants ─────────────────────────────────────────────────────
#define NDEF_TEXT_RECORD_TYPE  0x54
#define NDEF_TNF_WELL_KNOWN    0x01
#define NDEF_MAX_PAYLOAD       128

// ── Globals (Zigbee string format: first byte = length) ────────────────
Adafruit_PN532  nfc(PN532_SDA, PN532_SCL);

// Zigbee char string: [len_byte][data...][null_terminator]
static uint8_t  g_nfc_text_buf     [ZB_TEXT_MAX_LEN + 2] = {0};  // len + data + null
static uint8_t  g_nfc_uid_buf      [8] = {0};                    // len + 7 max UID
static uint8_t  g_nfc_uid_len       = 0;
static uint8_t  g_pending_write_buf[ZB_TEXT_MAX_LEN + 2] = {0};  // len + data + null
static bool     g_has_pending_write = false;

// Password authentication (PWD_AUTH + PACK)
static uint8_t  g_auth_pwd_buf[5]  = {0};    // octet string: len(4) + 4 bytes
static uint8_t  g_auth_pack_buf[3] = {0};    // octet string: len(2) + 2 bytes
static bool     g_auth_enabled      = false;
#define NTAG_CMD_PWD_AUTH          0x1B
#define NTAG_PAGE_CFG_AUTH0        131     // page 0x83: mirror, mirror_page, AUTH0, ACCESS
#define NTAG_PAGE_CFG_PWD          133     // page 0x85: PWD[4]
#define NTAG_PAGE_CFG_PACK         134     // page 0x86: PACK[2], RFUI[2]

// PN532 hot-plug state
static bool     g_nfc_reader_present = false;
static uint32_t g_nfc_last_check_ms  = 0;
#define NFC_PRESENCE_CHECK_INTERVAL_MS  5000

// ── Helpers to convert between C-string and Zigbee string ─────────────

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

        // Attr 0x0004: nfc_auth_pwd (octet string, RW, 4 bytes)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_AUTH_PWD,
            ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            g_auth_pwd_buf);

        // Attr 0x0005: nfc_auth_pack (octet string, RW, 2 bytes)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_AUTH_PACK,
            ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            g_auth_pack_buf);

        // Attr 0x0006: nfc_auth_enabled (bool, RW)
        esp_zb_custom_cluster_add_custom_attr(
            nfc_attr_list,
            ZB_ATTR_NFC_AUTH_ENABLED,
            ESP_ZB_ZCL_ATTR_TYPE_BOOL,
            ESP_ZB_ZCL_ATTR_ACCESS_READ_WRITE,
            &g_auth_enabled);

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
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setNfcUid(const uint8_t *uid, uint8_t len) {
        if (len > 7) len = 7;
        g_nfc_uid_buf[0] = len;
        memcpy(g_nfc_uid_buf + 1, uid, len);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_UID, g_nfc_uid_buf, false);
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

    bool setAuthPwd(const uint8_t *pwd) {
        g_auth_pwd_buf[0] = 4;
        memcpy(g_auth_pwd_buf + 1, pwd, 4);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_AUTH_PWD, g_auth_pwd_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setAuthPack(const uint8_t *pack) {
        g_auth_pack_buf[0] = 2;
        memcpy(g_auth_pack_buf + 1, pack, 2);
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_AUTH_PACK, g_auth_pack_buf, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    bool setAuthEnabled(bool enabled) {
        g_auth_enabled = enabled;
        esp_zb_zcl_status_t ret = setClusterAttribute(
            ZB_CLUSTER_NFC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ZB_ATTR_NFC_AUTH_ENABLED, &g_auth_enabled, false);
        return ret == ESP_ZB_ZCL_STATUS_SUCCESS;
    }

    const uint8_t *getAuthPwd() const { return g_auth_pwd_buf[0] == 4 ? g_auth_pwd_buf + 1 : nullptr; }
    const uint8_t *getAuthPack() const { return g_auth_pack_buf[0] == 2 ? g_auth_pack_buf + 1 : nullptr; }
    bool isAuthEnabled() const { return g_auth_enabled; }

private:
    bool reportAttr(uint16_t attr_id) {
        esp_zb_zcl_report_attr_cmd_t cmd = {};
        cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        cmd.attributeID  = attr_id;
        cmd.direction    = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
        cmd.clusterID    = ZB_CLUSTER_NFC;
        cmd.zcl_basic_cmd.src_endpoint = _endpoint;
        cmd.manuf_specific = 0x00U;
        cmd.dis_default_resp = 0x00U;
        return reportClusterAttribute(&cmd);
    }

private:
    // ── Handle ZCL write attribute ───────────────────────────────────
    void zbAttributeSet(const esp_zb_zcl_set_attr_value_message_t *message) override {
        if (message->info.cluster == ZB_CLUSTER_NFC) {
            if (message->attribute.id == ZB_ATTR_NFC_WRITE) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_CHAR_STRING
                    && message->attribute.data.value != nullptr) {
                    const uint8_t *src = (const uint8_t *)message->attribute.data.value;
                    // ESP Zigbee SDK prepends the ZCL type byte before the
                    // string value for custom attributes.  Detect & skip it.
                    if (src[0] == (uint8_t)message->attribute.data.type) {
                        src++;
                    }
                    uint8_t srcLen = src[0];
                    if (srcLen > ZB_TEXT_MAX_LEN) srcLen = ZB_TEXT_MAX_LEN;
                    memcpy(g_pending_write_buf + 1, src + 1, srcLen);
                    g_pending_write_buf[0] = srcLen;
                    g_pending_write_buf[1 + srcLen] = '\0';
                    g_has_pending_write = (srcLen > 0);

                    Serial.print(F("Zb: queued write text = \""));
                    Serial.print((const char *)(g_pending_write_buf + 1));
                    Serial.println('"');
                }
            } else if (message->attribute.id == ZB_ATTR_NFC_AUTH_PWD) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING
                    && message->attribute.data.value != nullptr) {
                    const uint8_t *src = (const uint8_t *)message->attribute.data.value;
                    // ESP Zigbee SDK prepends the ZCL type byte (0x41) before the
                    // octet string value for custom attributes.  Detect & skip it.
                    if (src[0] == (uint8_t)message->attribute.data.type) {
                        src++;
                    }
                    uint8_t srcLen = src[0];
                    if (srcLen >= 4) {
                        g_auth_pwd_buf[0] = 4;
                        memcpy(g_auth_pwd_buf + 1, src + 1, 4);
                        Serial.print(F("Zb: auth PWD set = "));
                        for (uint8_t i = 0; i < 4; i++) {
                            if (g_auth_pwd_buf[1+i] < 0x10) Serial.print('0');
                            Serial.print(g_auth_pwd_buf[1+i], HEX);
                        }
                        Serial.println();
                    }
                }
            } else if (message->attribute.id == ZB_ATTR_NFC_AUTH_PACK) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_OCTET_STRING
                    && message->attribute.data.value != nullptr) {
                    const uint8_t *src = (const uint8_t *)message->attribute.data.value;
                    if (src[0] == (uint8_t)message->attribute.data.type) {
                        src++;
                    }
                    uint8_t srcLen = src[0];
                    if (srcLen >= 2) {
                        g_auth_pack_buf[0] = 2;
                        memcpy(g_auth_pack_buf + 1, src + 1, 2);
                        Serial.print(F("Zb: auth PACK set = "));
                        for (uint8_t i = 0; i < 2; i++) {
                            if (g_auth_pack_buf[1+i] < 0x10) Serial.print('0');
                            Serial.print(g_auth_pack_buf[1+i], HEX);
                        }
                        Serial.println();
                    }
                }
            } else if (message->attribute.id == ZB_ATTR_NFC_AUTH_ENABLED) {
                if (message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL
                    && message->attribute.data.value != nullptr) {
                    g_auth_enabled = *(const bool *)message->attribute.data.value;
                    Serial.printf("Zb: auth %s\n", g_auth_enabled ? "ENABLED" : "DISABLED");
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

// ── NTAG PWD_AUTH ────────────────────────────────────────────────────
// Sends password to a selected NTAG tag and checks the PACK response.
// Returns true if PACK matches the expected value.
static bool ntagPasswordAuth(const uint8_t *uid, uint8_t uidLen,
                             const uint8_t *pwd4, const uint8_t *expectedPack2) {
    (void)uid; (void)uidLen;
    uint8_t cmd[] = {NTAG_CMD_PWD_AUTH, pwd4[0], pwd4[1], pwd4[2], pwd4[3]};
    uint8_t resp[2] = {0};
    uint8_t respLen = 2;
    if (!nfc.inDataExchange(cmd, sizeof(cmd), resp, &respLen)) {
        Serial.println(F("PWD_AUTH: exchange failed"));
        return false;
    }
    if (respLen != 2) {
        Serial.printf("PWD_AUTH: unexpected resp len %d\n", respLen);
        return false;
    }
    // Verify PACK (LSB first from tag)
    if (resp[0] != expectedPack2[0] || resp[1] != expectedPack2[1]) {
        Serial.printf("PWD_AUTH: PACK mismatch (got %02X%02X, expected %02X%02X)\n",
                      resp[0], resp[1], expectedPack2[0], expectedPack2[1]);
        return false;
    }
    Serial.println(F("PWD_AUTH: OK"));
    return true;
}

// ── Configure tag with password ───────────────────────────────────────
// Writes PWD, PACK, and sets AUTH0=4 so pages 4+ require authentication.
// Must be called while tag is selected and BEFORE AUTH0 takes effect.
static bool ntagConfigureAuth(const uint8_t *uid, uint8_t uidLen,
                              const uint8_t *pwd4, const uint8_t *pack2) {
    (void)uid; (void)uidLen;
    // Write PWD to page 133 (LSB first per NTAG spec)
    uint8_t pwdPage[4] = {pwd4[0], pwd4[1], pwd4[2], pwd4[3]};
    if (!nfc.ntag2xx_WritePage(NTAG_PAGE_CFG_PWD, pwdPage)) {
        Serial.println(F("cfg: write PWD failed"));
        return false;
    }
    delay(2);
    // Write PACK to page 134 (first 2 bytes, RFUI zeroed)
    uint8_t packPage[4] = {pack2[0], pack2[1], 0x00, 0x00};
    if (!nfc.ntag2xx_WritePage(NTAG_PAGE_CFG_PACK, packPage)) {
        Serial.println(F("cfg: write PACK failed"));
        return false;
    }
    delay(2);
    // Read page 131, modify AUTH0 byte, write back
    uint8_t cfgPage[4] = {0};
    if (!nfc.ntag2xx_ReadPage(NTAG_PAGE_CFG_AUTH0, cfgPage)) {
        Serial.println(F("cfg: read AUTH0 page failed"));
        return false;
    }
    delay(2);
    cfgPage[2] = 0x04;  // AUTH0 = page 4 (protect NDEF data and above)
    if (!nfc.ntag2xx_WritePage(NTAG_PAGE_CFG_AUTH0, cfgPage)) {
        Serial.println(F("cfg: write AUTH0 failed"));
        return false;
    }
    Serial.println(F("Tag auth configured: PWD+PACK set, AUTH0=4"));
    return true;
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
    (void)uid; (void)uidLen;
    uint8_t pages = (len + 3) / 4;
    for (uint8_t i = 0; i < pages; i++) {
        if (!nfc.ntag2xx_ReadPage(page + i, buf + (i * 4)))
            return false;
        delay(2);  // prevent tag-side timeout on long bursts
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
    // If auth is enabled and we have credentials, authenticate first
    if (g_auth_enabled && g_auth_pwd_buf[0] == 4 && g_auth_pack_buf[0] == 2) {
        if (!ntagPasswordAuth(uid, uidLen, g_auth_pwd_buf + 1, g_auth_pack_buf + 1)) {
            Serial.println(F("Auth failed — tag not trusted"));
            return false;
        }
    }
    // Single read burst — 32 bytes (8 pages) is known to work reliably.
    // Covers NDEF text records up to ~27 chars.  For longer messages
    // the caller (console_read / updateNfcState) will truncate.
    uint8_t raw[32] = {0};
    if (!readTagData(uid, uidLen, 4, raw, sizeof(raw)))
        return false;
    return parseTextNDEF(raw, sizeof(raw), out, outSize);
}

static bool writeTagText(const uint8_t *uid, uint8_t uidLen,
                         const char *text) {
    uint8_t ndef[NDEF_MAX_PAYLOAD + 16] = {0};
    size_t nLen = buildTextNDEF(text, ndef, sizeof(ndef));
    if (!nLen) return false;
    if (!writeTagData(uid, uidLen, 4, ndef, nLen))
        return false;

    // If auth is enabled, configure the tag with PWD/PACK/AUTH0
    // after writing NDEF data (tag must still be selected).
    if (g_auth_enabled && g_auth_pwd_buf[0] == 4 && g_auth_pack_buf[0] == 2) {
        if (!ntagConfigureAuth(uid, uidLen, g_auth_pwd_buf + 1, g_auth_pack_buf + 1)) {
            Serial.println(F("Warning: auth config failed — tag data was written"));
            // Don't fail the whole write; data is on the tag.
        }
    }
    return true;
}

static void updateNfcState(const uint8_t *uid, uint8_t uidLen,
                           const char *text) {
    g_nfc_uid_len = uidLen;
    nfcEp.setNfcText(text);
    nfcEp.setNfcUid(uid, uidLen);
    nfcEp.reportNfcText();
}

// ==========================================================================
//  Serial console commands
// ==========================================================================

static void console_read() {
    if (!g_nfc_reader_present) {
        Serial.println(F("PN532 not present — cannot read."));
        return;
    }
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
    if (!g_nfc_reader_present) {
        Serial.println(F("PN532 not present — cannot write."));
        return;
    }
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
        g_pending_write_buf[0] = 0;
        g_pending_write_buf[1] = '\0';
        nfcEp.setPendingWrite("");
    } else {
        Serial.println(F("✗ Write failed."));
    }
}

static void console_scan() {
    if (!g_nfc_reader_present) {
        Serial.println(F("PN532 not present — cannot scan."));
        return;
    }
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

        if (g_has_pending_write && g_pending_write_buf[0]) {
            Serial.print(F("  -> writing pending: "));
            Serial.println((const char *)(g_pending_write_buf + 1));
            if (writeTagText(uid, uidLen, (const char *)(g_pending_write_buf + 1))) {
                Serial.println(F("  ✓ written."));
            } else {
                Serial.println(F("  x write failed."));
            }
            g_has_pending_write = false;
            g_pending_write_buf[0] = 0;
            g_pending_write_buf[1] = '\0';
            nfcEp.setPendingWrite("");
        }
        delay(300);
    }
    while (Serial.available()) Serial.read();
    Serial.println(F("\nStopped."));
}

// Dump raw tag memory for debugging
static void console_dump() {
    if (!g_nfc_reader_present) {
        Serial.println(F("PN532 not present."));
        return;
    }
    Serial.println(F("Bring a tag…"));
    uint8_t uid[7], uidLen;
    if (!waitForTag(uid, &uidLen, 5000)) {
        Serial.println(F("Timeout.")); return;
    }
    printUID(uid, uidLen);

    // Authenticate if required (pages 4+ are protected when auth enabled)
    if (g_auth_enabled && g_auth_pwd_buf[0] == 4 && g_auth_pack_buf[0] == 2) {
        if (!ntagPasswordAuth(uid, uidLen, g_auth_pwd_buf + 1, g_auth_pack_buf + 1)) {
            Serial.println(F("Auth failed — cannot dump protected pages"));
            return;
        }
    }

    // Read pages 3-10 (page 3=OTP, pages 4+ = user data)
    uint8_t raw[8 * 4] = {0};
    if (!readTagData(uid, uidLen, 3, raw, sizeof(raw))) {
        Serial.println(F("Read failed.")); return;
    }
    for (uint8_t pg = 0; pg < 8; pg++) {
        Serial.printf("  Pg%02d: ", pg + 3);
        for (uint8_t b = 0; b < 4; b++) {
            uint8_t val = raw[pg * 4 + b];
            if (val < 0x10) Serial.print('0');
            Serial.print(val, HEX); Serial.print(' ');
        }
        Serial.print("  ");
        for (uint8_t b = 0; b < 4; b++) {
            uint8_t val = raw[pg * 4 + b];
            Serial.write(val >= 0x20 && val < 0x7F ? (char)val : '.');
        }
        Serial.println();
    }
}

static void console_auth() {
    Serial.println(F("Enter PWD (8 hex chars) then PACK (4 hex chars):"));
    Serial.setTimeout(15000);
    String pwdStr = Serial.readStringUntil('\n');
    pwdStr.trim();
    pwdStr.replace(" ", "");
    String packStr = Serial.readStringUntil('\n');
    packStr.trim();
    packStr.replace(" ", "");
    if (pwdStr.length() != 8 || packStr.length() != 4) {
        Serial.println(F("Invalid — need 8 hex PWD + 4 hex PACK"));
        return;
    }
    uint8_t pwd[4], pack[2];
    for (uint8_t i = 0; i < 4; i++) {
        char b[3] = {pwdStr[i*2], pwdStr[i*2+1], 0};
        pwd[i] = (uint8_t)strtoul(b, nullptr, 16);
    }
    for (uint8_t i = 0; i < 2; i++) {
        char b[3] = {packStr[i*2], packStr[i*2+1], 0};
        pack[i] = (uint8_t)strtoul(b, nullptr, 16);
    }
    nfcEp.setAuthPwd(pwd);
    nfcEp.setAuthPack(pack);
    nfcEp.setAuthEnabled(true);
    Serial.print(F("Auth set — PWD="));
    for (uint8_t i = 0; i < 4; i++) {
        if (pwd[i] < 0x10) Serial.print('0');
        Serial.print(pwd[i], HEX);
    }
    Serial.print(F(" PACK="));
    for (uint8_t i = 0; i < 2; i++) {
        if (pack[i] < 0x10) Serial.print('0');
        Serial.print(pack[i], HEX);
    }
    Serial.println(F(" ENABLED"));
}

static void console_help() {
    Serial.println(F("\nCommands:"));
    Serial.println(F("  r — read & report tag once"));
    Serial.println(F("  w — prompt for text, write to next tag"));
    Serial.println(F("  c — continuous scan (any key stops)"));
    Serial.println(F("  s — show Zigbee & PN532 status"));
    Serial.println(F("  a — set auth password + pack (8+4 hex chars)"));
    Serial.println(F("  f — factory‑reset Zigbee & rejoin"));
    Serial.println(F("  d — raw tag memory dump (debug)"));
    Serial.println(F("  ? — this help"));
    Serial.println();
}

static void console_status() {
    Serial.println(F("-- Zigbee NFC Bridge Status --"));
    Serial.print(F("  Network: "));
    Serial.println(Zigbee.connected() ? F("joined ✓") : F("not joined x"));
    Serial.print(F("  PN532 reader: "));
    Serial.println(g_nfc_reader_present ? F("present ✓") : F("absent x"));
    Serial.print(F("  NFC text:   \""));
    Serial.print((const char *)(g_nfc_text_buf + 1));
    Serial.println('"');
    Serial.print(F("  NFC UID:    "));
    if (g_nfc_uid_len) printUID(g_nfc_uid_buf + 1, g_nfc_uid_len);
    else Serial.println(F("(none)"));
    Serial.print(F("  Pending write: "));
    if (g_has_pending_write && g_pending_write_buf[0]) {
        Serial.print('"'); Serial.print((const char *)(g_pending_write_buf + 1)); Serial.println('"');
    } else {
        Serial.println(F("(none)"));
    }
    Serial.print(F("  Auth: "));
    Serial.print(g_auth_enabled ? F("ENABLED  pwd=") : F("DISABLED  pwd="));
    if (g_auth_pwd_buf[0] == 4) {
        for (uint8_t i = 0; i < 4; i++) {
            if (g_auth_pwd_buf[1+i] < 0x10) Serial.print('0');
            Serial.print(g_auth_pwd_buf[1+i], HEX);
        }
    } else { Serial.print(F("(none)")); }
    Serial.print(F("  pack="));
    if (g_auth_pack_buf[0] == 2) {
        for (uint8_t i = 0; i < 2; i++) {
            if (g_auth_pack_buf[1+i] < 0x10) Serial.print('0');
            Serial.print(g_auth_pack_buf[1+i], HEX);
        }
    } else { Serial.print(F("(none)")); }
    Serial.println();
}

// ==========================================================================
//  PN532 presence detection (hot-plug)
// ==========================================================================

// Silent I2C bus probe — no error spam when PN532 is absent.
// On first successful detection after absence (or at boot) SAMConfig is run.
static bool checkNfcPresence() {
    static bool was_present = false;

    Wire.beginTransmission(PN532_I2C_ADDR);
    bool present = (Wire.endTransmission() == 0);

    if (present && !was_present) {
        // Freshly attached — configure SAM
        nfc.SAMConfig();
        Serial.println(F("PN532 detected — SAM configured"));
    } else if (!present && was_present) {
        Serial.println(F("PN532 lost"));
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
        Serial.printf("PN532 reader present: %s\n", now ? "yes" : "no");
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
    if (ver) {
        Serial.printf("PN532: chip=0x%02lX  fw=%lu.%lu\n",
                      (ver >> 24) & 0xFF, (ver >> 16) & 0xFF, (ver >> 8) & 0xFF);
        nfc.SAMConfig();
        g_nfc_reader_present = true;
    } else {
        Serial.println(F("PN532 not found at boot — will poll for hot-plug…"));
        g_nfc_reader_present = false;
    }
    g_nfc_last_check_ms = millis();

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

    // Initial report of reader presence
    nfcEp.setReaderPresent(g_nfc_reader_present);
    nfcEp.reportReaderPresent();

    Serial.println(F("\nCommands:  r)ead  w)rite  c)ontinuous scan  s)tatus  f)actory-reset  ?)help"));
    Serial.println(F("Ready.\n"));
}

void loop() {
    // ── Poll PN532 presence (hot-plug detection) ──
    pollNfcPresence();

    // ── Pending write from Zigbee coordinator? ──
    if (g_nfc_reader_present && g_has_pending_write && g_pending_write_buf[0]) {
        Serial.print(F("Pending write from Zb: \""));
        Serial.print((const char *)(g_pending_write_buf + 1));
        Serial.println(F("\" — bring a tag…"));

        uint8_t uid[7], uidLen;
        if (waitForTag(uid, &uidLen, 30000)) {
            printUID(uid, uidLen);
            if (writeTagText(uid, uidLen, (const char *)(g_pending_write_buf + 1))) {
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
        g_pending_write_buf[0] = 0;
        g_pending_write_buf[1] = '\0';
        nfcEp.setPendingWrite("");
    }

    // ── Serial commands ──
    if (Serial.available()) {
        switch ((char)Serial.read()) {
            case 'r': while (Serial.available()) Serial.read(); console_read();   break;
            case 'w': while (Serial.available()) Serial.read(); console_write();  break;
            case 'c': while (Serial.available()) Serial.read(); console_scan();   break;
            case 's': while (Serial.available()) Serial.read(); console_status(); break;
            case '?': while (Serial.available()) Serial.read(); console_help();   break;
            case 'd': while (Serial.available()) Serial.read(); console_dump();   break;
            case 'a': while (Serial.available()) Serial.read(); console_auth();   break;
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
