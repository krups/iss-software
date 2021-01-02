#!/usr/bin/env python3

import sys
import numpy as np
import matplotlib.pyplot as plt

if len(sys.argv) < 2:
    print("Need input file as arg.")
    sys.exit(1)

X=np.genfromtxt(sys.argv[1], delimiter=',')

plt.figure()
plt.plot(X[:,0]/1000/60/60, X[:,1:5])
plt.xlabel('Time (hrs)')
plt.ylabel('Temperature (deg. C)')
plt.title("Thermocouple temperature")
plt.show()
