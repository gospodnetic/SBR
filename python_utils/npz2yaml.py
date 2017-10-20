# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-10-19 18:16:33
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-10-20 14:48:04

import sys
import getopt
import numpy as np
import yaml

def convert(fname):
    file = open(fname, mode='r')
    tz = np.load(file)
    numpy_array = tz[tz.files[0]]
    tz.close()
    file.close()
    numpy_array = np.flipud(numpy_array)

    with open(fname + '.yaml', 'w') as f:
        yaml.dump(numpy_array.tolist(), f)

def main(argv):
    try:
        opts, args = getopt.getopt(argv,"i:",["input="])
    except getopt.GetoptError:
        print 'npz2yaml.py -i <inputfile>'
        print 'Output file will have the same name with .yaml extension appended.'
        sys.exit(2)
    
    for opt, arg in opts:
        if opt == '-h':
            print 'test.py -i <inputfile>'
            sys.exit()
        elif opt in ("-i", "--input"):
            fname = arg

    convert(fname)
    

if __name__ == "__main__":
   main(sys.argv[1:])