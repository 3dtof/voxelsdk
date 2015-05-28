import argparse

import sys

import matplotlib.pyplot as plt

import numpy as np

#parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
#parser.add_argument("-f", "--file", type=str, help="Voxel file (.vxl)", required=True)

#args = parser.parse_args()

## Voxel-A lens
k1 = -0.1583968
k2 = 0.06113919
k3 = 0.09898978
p1 = 0.001591975
p2 = -0.0001962754

x = np.linspace(0, 0.9, 200)
y = 0
r2 = x*x
r4 = r2*r2
r6 = r4*r2
  
x1 = x*(1 + k1*r2 + k2*r4 + k3*r6) + 2*p1*x*y + p2*(r2 + 2*x*x)
y1 = y*(1 + k1*r2 + k2*r4 + k3*r6) + p1*(r2 + 2*y*y) + p2*x*y

## Tintin lens
k1 = 0.909882
k2 = -3.559455
k3 = 3.626591
p1 = 0.047604
p2 = -0.005546

x2 = x*(1 + k1*r2 + k2*r4 + k3*r6) + 2*p1*x*y + p2*(r2 + 2*x*x)
y2 = y*(1 + k1*r2 + k2*r4 + k3*r6) + p1*(r2 + 2*y*y) + p2*x*y

r2 = x1*x1 + y1*y1
r4 = r2*r2
r6 = r4*r2
  
x3 = x1*(1 + k1*r2 + k2*r4 + k3*r6) + 2*p1*x1*y1 + p2*(r2 + 2*x1*x1)
y3 = y1*(1 + k1*r2 + k2*r4 + k3*r6) + p1*(r2 + 2*y1*y1) + p2*x1*y1


plt.plot(x, x1, x, x2, 'r', x, x3, 'k')
plt.grid(True)
plt.legend(['Voxel-A', 'TintinCDK', 'Distorted to Corrected'])
plt.show()