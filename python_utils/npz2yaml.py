# -*- coding: utf-8 -*-
# @Author: Petra Gospodnetic
# @Date:   2017-10-19 18:16:33
# @Last Modified by:   Petra Gospodnetic
# @Last Modified time: 2017-10-19 18:17:37

import numpy as np
import yaml


# Load .npz file
fname = '../data/rainbowsphere_C.cdb/image/phi=0/theta=0/vis=0/colorSphere1=0.npz'

file = open(fname, mode='r')
tz = np.load(file)
imageslice = tz[tz.files[0]]
tz.close()
file.close()
imageslice = np.flipud(imageslice)

imageslice = (imageslice / imageslice.max())

with open('image.yaml', 'w') as f:
    yaml.dump(imageslice.tolist(), f)