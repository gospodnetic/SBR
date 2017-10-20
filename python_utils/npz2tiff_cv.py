# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-10-19 14:10:00
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-10-19 18:09:48

import numpy as np
import cv2

# Load .npz file
fname = '../data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz'
np.set_printoptions(suppress=False)
# fname = "float_test.npz"
d = np.ndarray(shape=(100,200), dtype=np.float32)
d[()] = np.arange(20000).reshape(100, 200)
# np.savez(fname, d)
print d

file = open(fname, mode='r')
tz = np.load(file)
imageslice = tz[tz.files[0]]
print type(imageslice)
tz.close()
file.close()
imageslice = np.flipud(imageslice)

cv2.Save("image.xml", cv2.fromarray(imageslice))

# print type(imageslice)
# print imageslice.dtype
# print imageslice.max()
# print imageslice.min()

# print type(imageslice[0][0])
# print "\n\n\n\n"

# cv2.normalize(imageslice, imageslice, 0, 1, cv2.NORM_MINMAX)
# cv2.imwrite("image123.jpg", imageslice)
# cv2.imshow("2", imageslice)
# cv2.waitKey(0)
# print type(d[0][0])

# print "\n\n\n\n"
# print new_imageslice
# print "\n\n\n\n"
# print d