# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-11-01 13:17:55
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-11-02 15:30:04
import numpy as np
import PIL.Image
import zlib

# filename = "/home/petra/Desktop/SampleBasedReconstruction/python_utils/PCLDepthTest_box_side.Z"
filename = "/home/petra/Desktop/orthoBox_257.cdb/image/phi=0/theta=1/vis=0/colorBox1=3.Z"
shape = [257, 257] # Image size height, width

with open(filename, mode='rb') as file:
    compresseddata = file.read()
flatarr = np.fromstring(
    zlib.decompress(compresseddata),
    np.float32)

numpy_array = flatarr.reshape(shape)

np.set_printoptions(threshold=np.nan)

print "depth shape: ", numpy_array.shape
print "depth range:"
print "min", np.nanmin(numpy_array), "max", np.nanmax(numpy_array)
print "object depth range:"
print "min:", np.nanmin(numpy_array), "max:", np.where(numpy_array==np.nanmax(numpy_array), 0, numpy_array).max()

im = PIL.Image.fromarray(numpy_array)
im.show()