# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-10-19 18:16:33
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-11-01 13:31:21

import sys
import getopt
import numpy as np
import yaml
import zlib

def convert(fname, shape):
    print "Converting..."
    with open(fname, mode='rb') as file:
        compresseddata = file.read()
    flatarr = np.fromstring(
        zlib.decompress(compresseddata),
        np.float32)

    numpy_array = flatarr.reshape(shape)
    print numpy_array.shape
    print "depth range: min:", numpy_array.min(), "max:", np.where(numpy_array==256, 0, numpy_array).max()

    with open(fname + '.yaml', 'w') as f:
        yaml.dump(numpy_array.tolist(), f)

def main(argv):
    print ("Here")
    fname = ""
    try:
        opts, args = getopt.getopt(argv,"pi:w:h:",["properties", "input=", "width=", "height="])
    except getopt.GetoptError:
        print 'z2yaml.py -i <inputfile> -w <imagewidth> -h <imageheight>'
        print 'Output file will have the same name with .yaml extension appended.'
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-p':
            print 'z2yaml.py -i <inputfile> -w <imagewidth> -h <imageheight>'
            sys.exit()
        elif opt in ("-i", "--input"):
            fname = arg
        elif opt in ("-w", "--width"):
            width = arg
        elif opt in ("-h", "--height"):
            height = arg


    convert(fname, [int(width), int(height)])
    

if __name__ == "__main__":
   main(sys.argv[1:])