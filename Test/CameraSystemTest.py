#!/usr/bin/env python2.7
#
# TI Voxel Lib component.
# 
# Copyright (c) 2014 Texas Instruments Inc.
#

import Voxel

sys = Voxel.CameraSystem()

devices = sys.scan()
print devices

print "Got ", len(devices), " devices"

count = 0
count = count + 1
print count

gdepthCamera = None

pcframe = None

gframe = None

def callback(depthCamera, frame, type):
  print 'Hello'
  print depthCamera
  print frame
  #print frame.id, '@', frame.timestamp
  global count
  print count
  count = count + 1
  
  global gdepthCamera, pcframe, gframe

  #gdepthCamera.start()
  #gdepthCamera.stop()
  #gdepthCamera.start()

  if count > 100:
    gdepthCamera.stop()
    #del gdepthCamera
    pcframe = Voxel.PointCloudFrame.typeCast(frame)
    gframe = frame
    return

if len(devices) > 0:
  gdepthCamera = sys.connect(devices[0])
  
  frameSize = Voxel.FrameSize()
  frameSize.height = 240
  frameSize.width = 320
  gdepthCamera.setFrameSize(frameSize)
  
  gdepthCamera.registerCallback(Voxel.DepthCamera.FRAME_XYZI_POINT_CLOUD_FRAME, callback)
  
  gdepthCamera.start()
  gdepthCamera.wait()