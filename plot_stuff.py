#! /usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

files = ['10.dat', '100.dat', '200.dat', '300.dat']

for f in files:
    array = np.loadtxt(f)
    plt.plot(array, label=f)

plt.legend()
plt.show()
