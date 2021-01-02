#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Need input file as arg.")
    sys.exit(1)

X=np.genfromtxt(sys.argv[1], delimiter=',')

plt.figure()
plt.plot(X[:,0]/1000/60, X[:,1])
plt.title("Battery voltage for iridium packet send test")
plt.show()
