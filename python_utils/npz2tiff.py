# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-10-19 11:33:14
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-10-19 17:29:07

import PIL.Image as Image
import numpy as np

from matplotlib import pyplot as plt

# Load .npz file
fname = '../data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz'

file = open(fname, mode='r')
tz = np.load(file)
imageslice = tz[tz.files[0]]
tz.close()
file.close()
imageslice = np.flipud(imageslice)

imageslice = (imageslice / imageslice.max())

print type(imageslice)
print imageslice.dtype
print imageslice.max()
print imageslice.min()
img = Image.fromarray(imageslice, mode="F")
img.save("image123.tiff", "TIFF");
# plt.imshow(img, cmap='gray', interpolation='nearest');
# plt.show()

# create data
d = np.ndarray(shape=(10,20), dtype=np.float32)
d[()] = np.arange(200).reshape(10, 20).astype(np.float16)
print d
print type(d)
print d.dtype
print d.max()
print d.min()

im = Image.fromarray(d, mode='F') # float32
im.save("test2.tiff", "TIFF")
