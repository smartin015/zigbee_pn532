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
            await entity.write(CLUSTER, {[ATTR_WRITE]: buf});
            return {state: {nfc_pending_write: value.slice(0, len)}};
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
    ],
    toZigbee: [tzLocal.nfc_pending_write],
    exposes: [
        exposes.text('nfc_text', exposes.access.STATE)
            .withDescription('Last-read NFC tag text'),
        exposes.text('nfc_tag_uid', exposes.access.STATE)
            .withDescription('UID of the last-read NFC tag (hex)'),
        exposes.text('nfc_pending_write', exposes.access.ALL)
            .withDescription('Text queued for write to next presented tag'),
        exposes.binary('nfc_reader_present', exposes.access.STATE, true, false)
            .withDescription('Whether the PN532 module is reachable'),
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
