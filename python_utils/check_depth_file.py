# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-11-01 13:17:55
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-11-01 13:23:35
import numpy as np
import PIL.Image
import zlib

filename = "/home/petra/Downloads/ParaView-5.4.1-Qt5-OpenGL2-MPI-Linux-64bit/bin/test_clipping_range.cdb/image/phi=0/theta=0/vis=0/colorCone1=0.Z"
shape = [400, 300] # Image size width, height

with open(filename, mode='rb') as file:
    compresseddata = file.read()
flatarr = np.fromstring(
    zlib.decompress(compresseddata),
    np.float32)

numpy_array = flatarr.reshape(shape)

print "depth shape: ", numpy_array.shape
print "depth range:"
print "min", numpy_array.min(), "max", numpy_array.max()
print "object depth range:"
print "min:", numpy_array.min(), "max:", np.where(numpy_array==numpy_array.max(), 0, numpy_array).max()

im = PIL.Image.fromarray(numpy_array)
im.show()