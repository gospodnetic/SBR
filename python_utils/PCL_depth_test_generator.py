import numpy
import zlib


filename = "PCLDepthTest.Z"
arr = numpy.full((257, 257), 256.0, numpy.float32)
arr = arr.reshape(257,257);
arr.itemset((0, 0), 0)
arr.itemset((0, 256), 0)
arr.itemset((256, 0), 0)
arr.itemset((256, 256), 0)
arr.itemset((128, 128), 128)

# print arr
with open(filename, mode='wb') as file:
            file.write(zlib.compress(arr))

filename = "PCLDepthTest_box_side.Z"
arr = numpy.full((257, 257), 256.0, dtype=numpy.float32)

arr[:, 0] = 0
arr[:, 256] = 0
arr[0][:] = 0
arr[256][:] = 0
arr.itemset((128, 128), 128)
print arr

# print arr
with open(filename, mode='wb') as file:
            file.write(zlib.compress(arr))
