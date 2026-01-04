import { setTimeout } from 'node:timers/promises';
import { SerialPort } from 'serialport';

const PACKET_LENGTH_BYTES = 1;
const PACKET_DATA_BYTES = 16;
const PACKET_CRC_BYTES = 1;
const PACKET_CRC_INDEX = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES;
const PACKET_LENGTH = PACKET_LENGTH_BYTES + PACKET_DATA_BYTES + PACKET_CRC_BYTES;

const PACKET_ACK_DATA0 = 0x15;
const PACKET_RETX_DATA0 = 0x19;

const serialPath = "/dev/ttyACM0"; // Update this to your serial port path
const baudRate = 115200;

const crc8 = (data: Buffer | Array<number>) => {
    let crc = 0;

    for (const byte of data) {
        crc = (crc ^ byte) & 0xFF;
        for (let i = 0; i < 8; i++) {
            if (crc & 0x80) {
                crc = ((crc << 1) ^ 0x07) & 0xFF;
            } else {
                crc = (crc << 1) & 0xFF;
            }
        }
    }

    return crc;
};

const delay = (ms: number) => setTimeout(ms);

class Packet {
    length: number;
    data: Buffer;
    crc: number;

    static ack = new Packet(1, Buffer.from([PACKET_ACK_DATA0])).toBuffer();
    static retx = new Packet(1, Buffer.from([PACKET_RETX_DATA0])).toBuffer();

    constructor(length: number, data: Buffer, crc?: number) {
        this.length = length;
        this.data = data;

        if (this.data.length < PACKET_DATA_BYTES) {
            const bytesToPad = PACKET_DATA_BYTES - this.length;
            const padding = Buffer.alloc(bytesToPad, 0xFF);
            this.data = Buffer.concat([this.data, padding]);
        }

        if (typeof crc === 'undefined') {
            this.crc = this.computeCrc();
        } else {
            this.crc = crc;
        }
    }

    computeCrc() {
        const allData = [this.length, ...this.data];
        return crc8(allData);
    }

    toBuffer() {
        return Buffer.concat([
            Buffer.from([this.length]), this.data, Buffer.from([this.crc])]);
    }

    isSingleBytePacket(byte: number) {
        if (this.length !== 1) return false;
        if (this.data[0] !== byte) return false;
        for (let i = 1; i < PACKET_DATA_BYTES; i++) {
            if (this.data[i] !== 0xFF) return false;
        }
        return true;
    }

    isAck() {
        return this.isSingleBytePacket(PACKET_ACK_DATA0);
    }

    isRetx() {
        return this.isSingleBytePacket(PACKET_RETX_DATA0);
    }
}

// Serial port instance
const uart = new SerialPort({ path: serialPath, baudRate });

// Packet buffer
let packets: Packet[] = [];

let lastPacket: Buffer = Packet.ack

const writePacket = (packet: Buffer) => {
    uart.write(packet);
    lastPacket = packet;
}

let rxBuffer = Buffer.from([]);
const consumeFromBuffer = (n: number) => {
    const consumed = rxBuffer.slice(0, n);
    rxBuffer = rxBuffer.slice(n);
    return consumed;
}

uart.on('data', data => {

    // Add data to the packet
    rxBuffer = Buffer.concat([rxBuffer, data]);

    // Check if we can build a packet
    while (rxBuffer.length >= PACKET_LENGTH) {
        const raw = consumeFromBuffer(PACKET_LENGTH);
        const packet = new Packet(raw[0], raw.slice(1, 1 + PACKET_DATA_BYTES), raw[PACKET_CRC_INDEX]);
        const computedCrc = packet.computeCrc();

        // Check if the packet is valid and request a retransmission if not
        if (computedCrc !== packet.crc) {
            writePacket(Packet.retx);
            return;
        }

        if (packet.isRetx()) {
            writePacket(lastPacket);
            return;
        }

        if (packet.isAck()) {
            return;
        }

        packets.push(packet);
        writePacket(Packet.ack);
    }
});

// Wait for a packet to be available
const waitForPacket = async () => {
    while (packets.length < 1) {
        await delay(1);
    }
    const packet = packets[0];
    packets = packets.slice(1);
    return packet;
}

const main = async () => {
    console.log("Waiting for packet...");
    const packet = await waitForPacket();
    console.log("Packet received:", packet);
}

main();
