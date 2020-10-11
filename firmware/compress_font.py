#!/bin/env python3
import sys

input_shift = 0
input_bits = 16
block_code_len = 8

def padded_bin(v, p):
    b = bin(v)[2:]
    b = "0" * (p - len(b)) + b
    return b

def compress_geom(bits):
    # define bounding box
    print("bits", bits)


    x1 = input_bits
    x2 = 0
    y1 = len(bits)//input_bits
    y2 = 0

    bx = len(bin(x1)) - 2
    by = len(bin(y1)) - 2


    print("bx =", bx, " by =", by)

    for p in range(len(bits)):
        x = p % input_bits
        y = p // input_bits
        if bits[p] == '1':
            x1 = min(x1, x)
            x2 = max(x2, x)
            y1 = min(y1, y)
            y2 = max(y2, y)

    if x2 < x1:
        #empty
        x1 = 0
        x2 = -1
        y1 = 0
        y2 = -1
    print("bb = ", ((x1, y1), (x2, y2)))

    bitstream = padded_bin(x1, bx) + padded_bin(x2 - x1 + 1, bx)
    bitstream += padded_bin(y1, by) + padded_bin(y2 - y1 + 1, by)

    for p in range(len(bits)):
        x = p % input_bits
        y = p // input_bits
        if x >= x1 and x <= x2 and y >= y1 and y <= y2:
            bitstream += bits[p]

    bitstream = "".join(bitstream)
    print("compressed", bitstream)
    return bitstream


def bitstream_to_bytes(bs):
    bts = []

    if len(bs) % 8 != 0:
        bs += "0" * (8 - (len(bs) % 8))

    while len(bs) > 0:
        b = bs[:8]
        bs = bs[8:]

        bts += [int(b, 2)]
    return bts


chars = []

with open(sys.argv[1], "r") as f:
    for line in f:
        pf_lines = line.split(",")[:-1]
        #print(pf_lines)
        char_bitstream = ''
        for pl in pf_lines:
            v = int(pl, 0) >> input_shift
            char_bitstream += padded_bin(v, input_bits)

        chars += [char_bitstream]

bytestreams = []
ofs_tbl = []
ofs = 0
# now chars has all lines for all chars
for c in chars:
    #bis = compress_blk(c)
    bis = compress_geom(c)
    #bis = compress_ze(c)
    bys = bitstream_to_bytes(bis)
    bytestreams += [bys]
    ofs_tbl += [ofs]
    ofs += len(bys)

print("Total bytes:", ofs)

print("offsets")
print(ofs_tbl)

offset_bits = len(bin(ofs_tbl[-1])) - 2

ofs_bs = ""
for o in ofs_tbl:
    ofs_bs += padded_bin(o, offset_bits)

ofs_bys = bitstream_to_bytes(ofs_bs)

ofs_str = ""
for o in ofs_bys:
    ofs_str += "0x%02x" % o + ", "
print("ofs_data, bits =", offset_bits)
print(ofs_str)


print("table = ")

ch = 32
for bys in bytestreams:
    t = []
    for b in bys:
        t += "0x%02x" % b + ", "
    t += "// [" + chr(ch) + "]"

    ch = ch + 1
    print("".join(t))
