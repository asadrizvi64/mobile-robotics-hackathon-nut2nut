#!/usr/bin/env python3
"""square_map: crop a saved SLAM map (map_saver .pgm + .yaml) to a metric square and clean it.

    python3 square_map.py <map_without_ext> x0 x1 y0 y1 <out_without_ext>
    python3 square_map.py ~/maps/box_map -1.5 1.5 -1.5 1.5 ~/maps/square_map

Removes lone occupied cells (lidar speckle from an open room), writes <out>.pgm, <out>.yaml
(origin shifted to the crop) and <out>_preview.png (4x scaled, no PIL needed).
Used for docs/images/square_map_preview.png.
"""
import re, sys, zlib, struct
import numpy as np

src = sys.argv[1]
x0, x1, y0, y1 = map(float, sys.argv[2:6])
out = sys.argv[6]

yaml = open(src + '.yaml').read()
res = float(re.search(r'resolution:\s*([\d.]+)', yaml).group(1))
ox, oy = map(float, re.search(r'origin:\s*\[([^,]+),\s*([^,]+)', yaml).groups())

with open(src + '.pgm', 'rb') as f:
    assert f.readline().strip() == b'P5'
    line = f.readline()
    while line.startswith(b'#'):
        line = f.readline()
    w, h = map(int, line.split())
    f.readline()
    d = np.frombuffer(f.read(w * h), dtype=np.uint8).reshape(h, w)

img = d[::-1].copy()          # row 0 = lowest y
occ = img < 50
p = np.pad(occ.astype(np.uint8), 1)
n = sum(p[1 + dy:1 + dy + h, 1 + dx:1 + dx + w]
        for dy in (-1, 0, 1) for dx in (-1, 0, 1) if (dy, dx) != (0, 0))
speckle = occ & (n < 2)       # lone occupied cells are lidar noise
img[speckle] = 254

c0 = max(0, int(np.floor((x0 - ox) / res)))
c1 = min(w, int(np.ceil((x1 - ox) / res)))
r0 = max(0, int(np.floor((y0 - oy) / res)))
r1 = min(h, int(np.ceil((y1 - oy) / res)))
crop = img[r0:r1, c0:c1]

cocc = int((crop < 50).sum())
cfree = int((crop > 200).sum())
cunk = crop.size - cocc - cfree
print(f'square {(c1 - c0) * res:.2f} x {(r1 - r0) * res:.2f} m: '
      f'occupied {cocc}, free {cfree}, unknown {cunk} cells')
print(f'speckle removed: {int(speckle.sum())}')

cw, ch = c1 - c0, r1 - r0
top_first = crop[::-1]
with open(out + '.pgm', 'wb') as f:
    f.write(f'P5\n{cw} {ch}\n255\n'.encode())
    f.write(top_first.astype(np.uint8).tobytes())
with open(out + '.yaml', 'w') as f:
    f.write(f'image: {out}.pgm\nresolution: {res}\n'
            f'origin: [{ox + c0 * res:.4f}, {oy + r0 * res:.4f}, 0.0]\n'
            'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')

def chunk(t, dat):
    return struct.pack('>I', len(dat)) + t + dat + struct.pack('>I', zlib.crc32(t + dat) & 0xffffffff)
S = 4
big = np.repeat(np.repeat(top_first, S, axis=0), S, axis=1).astype(np.uint8)
raw = b''.join(b'\x00' + big[y].tobytes() for y in range(big.shape[0]))
png = (b'\x89PNG\r\n\x1a\n' + chunk(b'IHDR', struct.pack('>IIBBBBB', big.shape[1], big.shape[0], 8, 0, 0, 0, 0))
       + chunk(b'IDAT', zlib.compress(raw, 9)) + chunk(b'IEND', b''))
open(out + '_preview.png', 'wb').write(png)
print('written', out + '.pgm', out + '.yaml', out + '_preview.png')
