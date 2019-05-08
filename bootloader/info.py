#!/usr/bin/env python3

from intelhex import IntelHex
ih = IntelHex()
ih.fromfile("./bootloader.hex",format='hex')
a = ih.segments()

a_len = a[0][1]
b_len = a[0][0]
d = a_len - b_len

print(d)
