// zigbee2mqtt external converter for the Zigbee NFC Reader/Writer Bridge
//
// Usage: copy to zigbee2mqtt data/external_converters/
//   cp nfc-bridge.js /opt/zigbee2mqtt/data/external_converters/
//   systemctl restart zigbee2mqtt

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');

// Z2M uses string cluster keys for fromZigbee matching (msg.cluster is a string),
// but endpoint.bind/read/configureReporting expect a number. Keep both.
const CLUSTER_NUM = 0xFC00;
const CLUSTER_STR = '64512';

const ATTR_TEXT = 0x0000;
const ATTR_UID = 0x0001;
const ATTR_WRITE = 0x0002;
const ATTR_PRESENT = 0x0003;
const ATTR_AUTH_PWD = 0x0004;
const ATTR_AUTH_PACK = 0x0005;
const ATTR_AUTH_ENABLED = 0x0006;
const ATTR_LAST_READ_TS = 0x0007;
const ATTR_LAST_WRITE_TS = 0x0008;

// ZCL data types used by the firmware (must match ESP_ZB_ZCL_ATTR_TYPE_*)
const ZCL_OCTET_STRING = 0x41;
const ZCL_CHAR_STRING  = 0x42;
const ZCL_BOOLEAN      = 0x10;

// Helper: decode a ZCL string/octet value to JS.
// zigbee-herdsman may already decode CHAR_STRING → JS string or keep it
// as a length-prefixed Buffer.  OCTET_STRING is almost always a Buffer.
// Handle both so reports and read-responses work regardless.
function decodeString(raw) {
    if (raw === undefined || raw === null) return '';
    // Already decoded by herdsman
    if (typeof raw === 'string') return raw;
    // Length-prefixed Buffer (ZCL raw)
    if (Buffer.isBuffer(raw) && raw.length >= 1) {
        const len = Math.min(raw[0], raw.length - 1);
        return raw.slice(1, 1 + len).toString('utf8');
    }
    return '';
}

function decodeOctetHex(raw) {
    if (raw === undefined || raw === null) return '';
    // Already a hex string (unlikely but handle it)
    if (typeof raw === 'string') return raw;
    // Length-prefixed Buffer (ZCL raw)
    if (Buffer.isBuffer(raw) && raw.length >= 1) {
        // May or may not have the ZCL length prefix — detect
        let offset = 0;
        let len = raw.length;
        // If raw[0] == raw.length - 1, assume it's length-prefixed
        if (raw.length >= 2 && raw[0] === raw.length - 1) {
            offset = 1;
            len = raw[0];
        } else if (raw.length > 1 && raw[0] <= raw.length - 1) {
            // Conservative: treat first byte as length only if it's reasonable
            offset = 1;
            len = raw[0];
        }
        return raw.slice(offset, offset + len).toString('hex');
    }
    return '';
}

// ── fromZigbee converters ─────────────────────────────────────────────

const fzLocal = {
    nfc_text: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_TEXT];
            meta.logger.debug(`NFC: nfc_text converter — type=${msg.type}, val=${JSON.stringify(val)}, keys=${JSON.stringify(Object.keys(msg.data))}`);
            if (val !== undefined) {
                const text = decodeString(val);
                meta.logger.debug(`NFC: decoded nfc_text="${text}"`);
                return {nfc_text: text};
            }
            meta.logger.debug(`NFC: ATTR_TEXT (0x0000) not in msg.data`);
        },
    },
    nfc_tag_uid: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_UID];
            meta.logger.debug(`NFC: nfc_tag_uid converter — type=${msg.type}, val=${JSON.stringify(val)}`);
            if (val !== undefined) {
                const hex = decodeOctetHex(val);
                meta.logger.debug(`NFC: decoded nfc_tag_uid="${hex}"`);
                return {nfc_tag_uid: hex};
            }
            meta.logger.debug(`NFC: ATTR_UID (0x0001) not in msg.data`);
        },
    },
    nfc_pending_write: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_WRITE];
            if (val !== undefined) {
                return {nfc_pending_write: decodeString(val)};
            }
        },
    },
    nfc_reader_present: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_PRESENT];
            if (val !== undefined) {
                return {nfc_reader_present: !!val};
            }
        },
    },
    nfc_auth_enabled: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_AUTH_ENABLED];
            if (val !== undefined) {
                return {nfc_auth_enabled: !!val};
            }
        },
    },
    nfc_auth_pwd: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_AUTH_PWD];
            if (val !== undefined) {
                return {nfc_auth_pwd: decodeOctetHex(val)};
            }
        },
    },
    nfc_auth_pack: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_AUTH_PACK];
            if (val !== undefined) {
                return {nfc_auth_pack: decodeOctetHex(val)};
            }
        },
    },
    nfc_last_read_ts: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_LAST_READ_TS];
            if (val !== undefined) {
                return {nfc_last_read_ts: decodeString(val)};
            }
        },
    },
    nfc_last_write_ts: {
        cluster: CLUSTER_STR,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            const val = msg.data[ATTR_LAST_WRITE_TS];
            if (val !== undefined) {
                return {nfc_last_write_ts: decodeString(val)};
            }
        },
    },
};

// ── toZigbee converters ───────────────────────────────────────────────

// zigbee-herdsman option to skip attribute validation for custom clusters
const WRITE_OPTS = {writeUndiv: true};

const tzLocal = {
    nfc_pending_write: {
        key: ['nfc_pending_write'],
        convertSet: async (entity, key, value, meta) => {
            // writeUndiv requires a type hint but does NOT add the ZCL
            // length prefix for CHAR_STRING (unlike OCTET_STRING).
            // We include the prefix ourselves; firmware handles any
            // double-encoding that may occur.
            const text = value.slice(0, 128);
            const buf = Buffer.alloc(1 + text.length);
            buf[0] = text.length;
            if (text.length > 0) buf.write(text, 1, 'utf8');
            await entity.write(CLUSTER_NUM, {
                [ATTR_WRITE]: {value: buf, type: ZCL_CHAR_STRING},
            }, WRITE_OPTS);
            return {state: {nfc_pending_write: text}};
        },
    },
    nfc_auth_pwd: {
        key: ['nfc_auth_pwd'],
        convertSet: async (entity, key, value, meta) => {
            // value is a hex string of 4 bytes (8 hex chars)
            const hex = value.replace(/[^0-9a-fA-F]/g, '');
            if (hex.length !== 8) throw new Error('nfc_auth_pwd must be 8 hex chars (4 bytes)');
            // Send raw bytes — zigbee-herdsman adds the ZCL octet string length
            // prefix automatically when type is specified.
            const bytes = Buffer.from(hex, 'hex');
            await entity.write(CLUSTER_NUM, {
                [ATTR_AUTH_PWD]: {value: bytes, type: ZCL_OCTET_STRING},
            }, WRITE_OPTS);
            return {state: {nfc_auth_pwd: hex}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER_NUM, [ATTR_AUTH_PWD]);
        },
    },
    nfc_auth_pack: {
        key: ['nfc_auth_pack'],
        convertSet: async (entity, key, value, meta) => {
            // value is a hex string of 2 bytes (4 hex chars)
            const hex = value.replace(/[^0-9a-fA-F]/g, '');
            if (hex.length !== 4) throw new Error('nfc_auth_pack must be 4 hex chars (2 bytes)');
            const bytes = Buffer.from(hex, 'hex');
            await entity.write(CLUSTER_NUM, {
                [ATTR_AUTH_PACK]: {value: bytes, type: ZCL_OCTET_STRING},
            }, WRITE_OPTS);
            return {state: {nfc_auth_pack: hex}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER_NUM, [ATTR_AUTH_PACK]);
        },
    },
    nfc_auth_enabled: {
        key: ['nfc_auth_enabled'],
        convertSet: async (entity, key, value, meta) => {
            await entity.write(CLUSTER_NUM, {
                [ATTR_AUTH_ENABLED]: {value: !!value, type: ZCL_BOOLEAN},
            }, WRITE_OPTS);
            return {state: {nfc_auth_enabled: !!value}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER_NUM, [ATTR_AUTH_ENABLED]);
        },
    },
};

// ── Definition ────────────────────────────────────────────────────────

const definition = {
    zigbeeModel: ['ZigbeeNFCEndpoint', 'ZigbeeNFCBridge'],
    model: 'ZigbeeNFCBridge',
    vendor: 'Espressif',
    description: 'Zigbee NFC Reader/Writer Bridge (PN532)',
    fromZigbee: [
        fzLocal.nfc_text,
        fzLocal.nfc_tag_uid,
        fzLocal.nfc_pending_write,
        fzLocal.nfc_reader_present,
        fzLocal.nfc_auth_enabled,
        fzLocal.nfc_auth_pwd,
        fzLocal.nfc_auth_pack,
        fzLocal.nfc_last_read_ts,
        fzLocal.nfc_last_write_ts,
    ],
    toZigbee: [
        tzLocal.nfc_pending_write,
        tzLocal.nfc_auth_pwd,
        tzLocal.nfc_auth_pack,
        tzLocal.nfc_auth_enabled,
    ],
    exposes: [
        exposes.text('nfc_text', exposes.access.STATE)
            .withDescription('Last-read NFC tag text'),
        exposes.text('nfc_tag_uid', exposes.access.STATE)
            .withDescription('UID of the last-read NFC tag (hex)'),
        exposes.text('nfc_pending_write', exposes.access.ALL)
            .withDescription('Text queued for write to next presented tag'),
        exposes.binary('nfc_reader_present', exposes.access.STATE, true, false)
            .withDescription('Whether the PN532 module is reachable'),
        exposes.binary('nfc_auth_enabled', exposes.access.ALL, true, false)
            .withDescription('Enable NTAG password authentication'),
        exposes.text('nfc_auth_pwd', exposes.access.ALL)
            .withDescription('NTAG authentication password (4 bytes, hex)'),
        exposes.text('nfc_auth_pack', exposes.access.ALL)
            .withDescription('NTAG password acknowledge (2 bytes, hex)'),
        exposes.text('nfc_last_read_ts', exposes.access.STATE)
            .withDescription('ISO 8601 timestamp of the last successful tag read'),
        exposes.text('nfc_last_write_ts', exposes.access.STATE)
            .withDescription('ISO 8601 timestamp of the last successful tag write'),
    ],
    meta: {multiEndpoint: false},
    configure: async (device, coordinatorEndpoint, logger) => {
        const endpoint = device.getEndpoint(1);

        // Step 1: Bind (essential — without this, reports never reach the coordinator)
        try {
            await reporting.bind(endpoint, coordinatorEndpoint, [CLUSTER_NUM]);
            logger.info('NFC Bridge: bound cluster 0xFC00');
        } catch (e) {
            logger.warn('NFC Bridge: bind failed — ' + e.message);
        }

        // Step 2: Configure reporting (best-effort; custom clusters may not support it)
        try {
            await endpoint.configureReporting(CLUSTER_NUM, [
                {attribute: ATTR_TEXT,        minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
                {attribute: ATTR_UID,         minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
                {attribute: ATTR_PRESENT,     minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 1},
                {attribute: ATTR_LAST_READ_TS,  minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
                {attribute: ATTR_LAST_WRITE_TS, minimumReportInterval: 0, maximumReportInterval: 3600, reportableChange: 0},
            ]);
            logger.info('NFC Bridge: reporting configured for cluster 0xFC00');
        } catch (e) {
            logger.debug('NFC Bridge: configureReporting skipped — ' + e.message);
        }

        // Step 3: Read initial attribute values
        try {
            await endpoint.read(CLUSTER_NUM, [
                ATTR_TEXT, ATTR_UID, ATTR_PRESENT,
                ATTR_AUTH_ENABLED, ATTR_AUTH_PWD, ATTR_AUTH_PACK,
                ATTR_LAST_READ_TS, ATTR_LAST_WRITE_TS,
            ]);
            logger.info('NFC Bridge: read initial values sent');
        } catch (e) {
            logger.warn('NFC Bridge: read failed — ' + e.message);
        }
    },
};

module.exports = definition;
