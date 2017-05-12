from __future__ import print_function

import numpy as np
from hw_interface import pyHW

address = [1,1,1,1]
i2cHW = pyHW()
print('get address', i2cHW.get_mtrAddr)
print(i2cHW.pytest())
