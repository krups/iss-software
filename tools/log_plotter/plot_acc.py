#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Need input file as arg.")
    sys.exit(1)

X=np.genfromtxt(sys.argv[1], delimiter=',')

acc = X[:,1:4]
acc = (acc/1023 - 0.5) * 200;

plt.figure()
plt.plot(X[:,0]/1000/60/60, acc)
plt.xlabel('Time (hrs)')
plt.ylabel('g force')
plt.title("ACC data")
plt.legend(('x','y','z'))
plt.show()
