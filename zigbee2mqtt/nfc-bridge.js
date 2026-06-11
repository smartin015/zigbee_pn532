// zigbee2mqtt external converter for the Zigbee NFC Reader/Writer Bridge
//
// Usage: copy to zigbee2mqtt data/external_converters/
//   cp nfc-bridge.js /opt/zigbee2mqtt/data/external_converters/
//   systemctl restart zigbee2mqtt

const fz = require('zigbee-herdsman-converters/converters/fromZigbee');
const tz = require('zigbee-herdsman-converters/converters/toZigbee');
const exposes = require('zigbee-herdsman-converters/lib/exposes');
const reporting = require('zigbee-herdsman-converters/lib/reporting');

const CLUSTER = 0xFC00;
const ATTR_TEXT = 0x0000;
const ATTR_UID = 0x0001;
const ATTR_WRITE = 0x0002;
const ATTR_PRESENT = 0x0003;
const ATTR_AUTH_PWD = 0x0004;
const ATTR_AUTH_PACK = 0x0005;
const ATTR_AUTH_ENABLED = 0x0006;

// Helper: decode ZCL char-string (length-prefixed) or octet-string to JS value
function decodeString(raw) {
    if (!Buffer.isBuffer(raw) || raw.length < 1) return '';
    const len = Math.min(raw[0], raw.length - 1);
    return raw.slice(1, 1 + len).toString('utf8');
}

// ── fromZigbee converters ─────────────────────────────────────────────

const fzLocal = {
    nfc_text: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_TEXT] !== undefined) {
                return {nfc_text: decodeString(msg.data[ATTR_TEXT])};
            }
        },
    },
    nfc_tag_uid: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_UID] !== undefined) {
                const raw = msg.data[ATTR_UID];
                if (Buffer.isBuffer(raw) && raw.length > 1) {
                    const len = Math.min(raw[0], raw.length - 1);
                    return {nfc_tag_uid: raw.slice(1, 1 + len).toString('hex')};
                }
            }
        },
    },
    nfc_pending_write: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_WRITE] !== undefined) {
                return {nfc_pending_write: decodeString(msg.data[ATTR_WRITE])};
            }
        },
    },
    nfc_reader_present: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_PRESENT] !== undefined) {
                return {nfc_reader_present: !!msg.data[ATTR_PRESENT]};
            }
        },
    },
    nfc_auth_enabled: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_AUTH_ENABLED] !== undefined) {
                return {nfc_auth_enabled: !!msg.data[ATTR_AUTH_ENABLED]};
            }
        },
    },
    nfc_auth_pwd: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_AUTH_PWD] !== undefined) {
                const raw = msg.data[ATTR_AUTH_PWD];
                if (Buffer.isBuffer(raw) && raw.length > 1) {
                    const len = Math.min(raw[0], raw.length - 1);
                    return {nfc_auth_pwd: raw.slice(1, 1 + len).toString('hex')};
                }
            }
        },
    },
    nfc_auth_pack: {
        cluster: CLUSTER,
        type: ['attributeReport', 'readResponse'],
        convert: (model, msg, publish, options, meta) => {
            if (msg.data[ATTR_AUTH_PACK] !== undefined) {
                const raw = msg.data[ATTR_AUTH_PACK];
                if (Buffer.isBuffer(raw) && raw.length > 1) {
                    const len = Math.min(raw[0], raw.length - 1);
                    return {nfc_auth_pack: raw.slice(1, 1 + len).toString('hex')};
                }
            }
        },
    },
};

// ── toZigbee converters ───────────────────────────────────────────────

const tzLocal = {
    nfc_pending_write: {
        key: ['nfc_pending_write'],
        convertSet: async (entity, key, value, meta) => {
            const len = Math.min(value.length, 128);
            const buf = Buffer.alloc(1 + len);
            buf[0] = len;
            if (len > 0) buf.write(value.slice(0, len), 1, 'utf8');
            await entity.write(CLUSTER, {[ATTR_WRITE]: buf}, {writeUndiv: true});
            return {state: {nfc_pending_write: value.slice(0, len)}};
        },
    },
    nfc_auth_pwd: {
        key: ['nfc_auth_pwd'],
        convertSet: async (entity, key, value, meta) => {
            // value is a hex string of 4 bytes (8 hex chars)
            const hex = value.replace(/[^0-9a-fA-F]/g, '');
            if (hex.length !== 8) throw new Error('nfc_auth_pwd must be 8 hex chars (4 bytes)');
            const bytes = Buffer.from(hex, 'hex');
            const buf = Buffer.alloc(5);
            buf[0] = 4;  // length prefix
            bytes.copy(buf, 1);
            await entity.write(CLUSTER, {[ATTR_AUTH_PWD]: buf}, {writeUndiv: true});
            return {state: {nfc_auth_pwd: hex}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER, [ATTR_AUTH_PWD]);
        },
    },
    nfc_auth_pack: {
        key: ['nfc_auth_pack'],
        convertSet: async (entity, key, value, meta) => {
            // value is a hex string of 2 bytes (4 hex chars)
            const hex = value.replace(/[^0-9a-fA-F]/g, '');
            if (hex.length !== 4) throw new Error('nfc_auth_pack must be 4 hex chars (2 bytes)');
            const bytes = Buffer.from(hex, 'hex');
            const buf = Buffer.alloc(3);
            buf[0] = 2;  // length prefix
            bytes.copy(buf, 1);
            await entity.write(CLUSTER, {[ATTR_AUTH_PACK]: buf}, {writeUndiv: true});
            return {state: {nfc_auth_pack: hex}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER, [ATTR_AUTH_PACK]);
        },
    },
    nfc_auth_enabled: {
        key: ['nfc_auth_enabled'],
        convertSet: async (entity, key, value, meta) => {
            await entity.write(CLUSTER, {[ATTR_AUTH_ENABLED]: !!value}, {writeUndiv: true});
            return {state: {nfc_auth_enabled: !!value}};
        },
        convertGet: async (entity, key, meta) => {
            await entity.read(CLUSTER, [ATTR_AUTH_ENABLED]);
        },
    },
};

// ── Definition ────────────────────────────────────────────────────────

const definition = {
    zigbeeModel: ['ZigbeeNFCEndpoint'],
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
    ],
    meta: {multiEndpoint: false},
    configure: async (device, coordinatorEndpoint, logger) => {
        try {
            const endpoint = device.getEndpoint(1);
            await reporting.bind(endpoint, coordinatorEndpoint, [CLUSTER]);
            logger.info('NFC Bridge: bound cluster 0xFC00');
        } catch (e) {
            logger.warn('NFC Bridge: configure failed — ' + e.message);
        }
    },
};

module.exports = definition;
