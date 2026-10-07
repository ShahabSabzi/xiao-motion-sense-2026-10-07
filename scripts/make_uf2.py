#!/usr/bin/env python3
"""Convert the application Intel HEX to nRF52840 UF2 (no dependencies).
Only flash application addresses from 0x26000 upward; never rewrite the
SoftDevice, MBR, UICR, settings pages or bootloader. Requires a Seeed
bootloader/SoftDevice already present, as on a normal Sense board.
"""
import struct
import sys
from pathlib import Path

def convert(source, target):
    data = {}
    base = 0
    for line in Path(source).read_text().splitlines():
        if not line.startswith(':'):
            continue
        record = bytes.fromhex(line[1:])
        if sum(record) & 255:
            raise ValueError('Invalid Intel HEX checksum')
        size = record[0]
        address = (record[1] << 8) | record[2]
        kind = record[3]
        if kind == 0:
            for i, value in enumerate(record[4:4 + size]):
                loc = base + address + i
                if not 0x26000 <= loc < 0xF4000:
                    raise ValueError('HEX address outside application: ' + hex(loc))
                data[loc] = value
        elif kind == 4:
            base = int.from_bytes(record[4:6], 'big') << 16
        elif kind == 2:
            base = int.from_bytes(record[4:6], 'big') << 4
        elif kind == 1:
            break
    if not data:
        raise ValueError('No application bytes')
    addresses = sorted({a & ~255 for a in data})
    with Path(target).open('wb') as out:
        for number, address in enumerate(addresses):
            header = struct.pack('<8I', 0x0A324655, 0x9E5D5157, 0x2000,
                                 address, 256, number, len(addresses), 0xADA52840)
            payload = bytes(data.get(address + i, 0xFF) for i in range(256))
            out.write(header + payload + bytes(220) + struct.pack('<I', 0x0AB16F30))
    print('UF2:', len(addresses), 'blocks,', len(data), 'application bytes')

if __name__ == '__main__':
    convert(sys.argv[1], sys.argv[2])
