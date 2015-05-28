import Voxel
import argparse

import sys

import matplotlib.pyplot as plt

import numpy as np

parser = argparse.ArgumentParser(formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument("-f", "--file", type=str, help="Voxel file (.vxl)", required=True)

args = parser.parse_args()


camsys = Voxel.CameraSystem()

r = Voxel.FrameStreamReader(args.file, camsys)

iAverage = None
qAverage = None
frameCount = 0

if not r.isStreamGood():
  print 'Could not open stream'
  sys.exit(1)
  
print 'Stream contains ', r.size(), ' frames'

count = r.size()

for i in range(0, count):
  if not r.readNext():
    print 'Failed to read frame ', i
    break
  
  tofFrame = Voxel.ToF16IQFrame.typeCast(r.frames[Voxel.DepthCamera.FRAME_RAW_FRAME_PROCESSED])
  
  if not tofFrame:
    print 'Frame %d not a ToF IQ frame?'%i
    continue
  
  frameCount = frameCount + 1
  if iAverage == None:
    print tofFrame._i.size()
    iAverage = np.array(tofFrame._i, copy = True).astype(float)
    qAverage = np.array(tofFrame._q, copy = True).astype(float)
    print iAverage, qAverage
  else:
    iAverage += np.array(tofFrame._i)
    qAverage += np.array(tofFrame._q)

iAverage /= frameCount
qAverage /= frameCount

img = np.reshape(iAverage, (tofFrame.size.height, tofFrame.size.width))
img.clip(0, 4096, out = img)
#print img

plt.imshow(img)
plt.show()

#np.save(args.npy, img)

del r
del tofFrame
del camsys