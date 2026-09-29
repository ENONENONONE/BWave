#!/usr/bin/env python3
"""Convert BGR565 splash screen data to bwave_boot_screen.h (RGB565).

Source data arrives from the image tool in BGR565 format (blue in high 5 bits).
The ST7789 display pipeline expects RGB565 (red in high 5 bits).
This script swaps R <-> B on all pixels and writes the firmware header.

Usage:
    python3 convert_splash.py [source.txt] [output.h]
    python3 convert_splash.py ~/Documents/splash.txt main/bwave_boot_screen.h
"""

import re
import sys

def convert(src_path, dst_path):
    with open(src_path, 'r') as f:
        content = f.read()

    m = re.search(r'(\d+)\s+unique', content)
    n_colors = m.group(1) if m else '???'

    vals = re.findall(r'0x[0-9a-fA-F]{4}', content)

    if len(vals) != 55040:
        print(f'ERROR: expected 55040 values, got {len(vals)}')
        sys.exit(1)

    fixed = []
    for v in vals:
        px = int(v, 16)
        b = (px >> 11) & 0x1F
        g = (px >>  5) & 0x3F
        r =  px        & 0x1F
        fixed.append('0x%04x' % ((r << 11) | (g << 5) | b))

    lines = [
        '#ifndef BWAVE_BOOT_SCREEN_H',
        '#define BWAVE_BOOT_SCREEN_H',
        '',
        '#include <stdint.h>',
        '',
        f'/* 172x320 ST7789 RGB565 -- 110080 bytes, {n_colors} unique colors, 3 layers */',
        'static const uint16_t BOOT_SCREEN[55040] = {',
    ]

    for i in range(0, len(fixed), 16):
        chunk = fixed[i:i+16]
        line = '  ' + ','.join(chunk)
        if i + 16 < len(fixed):
            line += ','
        lines.append(line)

    lines += ['};', '', '#endif', '']

    with open(dst_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f'Wrote {dst_path}')
    print(f'  {len(fixed)} pixels, {n_colors} unique colors')
    print(f'  First: {vals[0]} -> {fixed[0]}')

if __name__ == '__main__':
    src = sys.argv[1] if len(sys.argv) > 1 else 'splash.txt'
    dst = sys.argv[2] if len(sys.argv) > 2 else 'main/bwave_boot_screen.h'
    convert(src, dst)
