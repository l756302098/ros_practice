#!/usr/bin/env python
# -*- coding:utf-8 -*- 

import random
 
import numpy as np
 
import matplotlib.pyplot as plt

class Robot(object):
 
  def __init__(self, length=20.0):
 
    """
      Creates robotand initializes location/orientation to 0, 0, 0.
    """
 
    self.x = 0.0
 
    self.y = 0.0
 
    self.orientation = 0.0
 
    self.length =length
 
    self.steering_noise = 0.0
 
    self.distance_noise = 0.0
 
    self.steering_drift = 0.0
 
 
 
  def set(self, x,y, orientation):
 
    """
      Sets a robotcoordinate.
    """
 
    self.x = x
 
    self.y = y
 
    self.orientation = orientation % (2.0 * np.pi)
 
 
 
  def set_noise(self, steering_noise, distance_noise):
 
    """
      Sets thenoise parameters.
    """
 
    # makes itpossible to change the noise parameters
 
    # this isoften useful in particle filters
 
    self.steering_noise = steering_noise
 
    self.distance_noise = distance_noise
 
 
 
  def set_steering_drift(self, drift):
 
    """
      Sets thesystematical steering drift parameter
    """
 
    self.steering_drift = drift
 
 
  def move(self,steering, distance, tolerance=0.001, max_steering_angle=np.pi / 4.0):
 
    """
      steering =front wheel steering angle, limited by max_steering_angle
      distance =total distance driven, most be non-negative
    """
 
    if steering> max_steering_angle:
 
      steering= max_steering_angle
 
    if steering <-max_steering_angle:
 
      steering= -max_steering_angle
 
    if distance< 0.0:
 
      distance= 0.0

    # apply noise
    steering2 =random.gauss(steering, self.steering_noise)
 
    distance2 =random.gauss(distance, self.distance_noise)
 

    # applysteering drift
    steering2 +=self.steering_drift
 
    # Execute motion
    turn =np.tan(steering2) * distance2 / self.length
 
 
    if abs(turn)< tolerance:
 
      #approximate by straight line motion
 
      self.x +=distance2 * np.cos(self.orientation)
 
      self.y +=distance2 * np.sin(self.orientation)
 
      self.orientation = (self.orientation + turn) % (2.0 * np.pi)
 
    else:
 
      #approximate bicycle model for motion
 
      radius =distance2 / turn
 
      cx =self.x - (np.sin(self.orientation) * radius)
 
      cy =self.y + (np.cos(self.orientation) * radius)
 
      self.orientation = (self.orientation + turn) % (2.0 * np.pi)
 
      self.x =cx + (np.sin(self.orientation) * radius)
 
      self.y =cy - (np.cos(self.orientation) * radius)
 
 
 
  def __repr__(self):
 
    return'[x=%.5f y=%.5f orient=%.5f]' % (self.x, self.y, self.orientation)
 

def run_p(robot, tau, n=100, speed=1.0):
 
  x_trajectory = []
 
  y_trajectory = []
  
  for i in range(n):
 
    cte = robot.y
 
    steer = -tau* cte
 
    robot.move(steer, speed)
 
    x_trajectory.append(robot.x)
 
    y_trajectory.append(robot.y)
 
  return x_trajectory, y_trajectory
 
   
 
robot = Robot()
 
robot.set(0, 1, 0)

robot.set_noise(0.1,0.05)
 
def run(robot, tau_p, tau_d, n=100, speed=1.0):
 
  x_trajectory = []
 
  y_trajectory = []
 
  #steering =-tau_p * CTE - tau_d * diff_CTE
 
  crosstrack_error= []
 
  crosstrack_error.append(0.0)
 
  diff_CTE = 0.0
 
  startX = robot.x
 
  startY = robot.y
 
  startOrientation= robot.orientation
 
  distance = 0.0
 
 
 
  for i in range(n):
 
    steering =-tau_p * crosstrack_error[i] - tau_d * diff_CTE
 
    distance =speed
 
    robot.move(steering, distance)
 
    x_trajectory.append(robot.x)
 
    y_trajectory.append(robot.y)
 
    # when in theoriginal path, x=robot.x ,caculate y.
 
    x1 = robot.x
 
    y1 = startY +(x1 - startX) * np.tan(startOrientation)
 
    crosstrack =(robot.y - y1) * np.cos(startOrientation)
 
    crosstrack_error.append(crosstrack)
 
    diff_CTE =crosstrack_error[i+1] - crosstrack_error[i]
 
    print("{} [{}, {}] {}, {}".format(i,robot.x, robot.y,steering, crosstrack))
 
       
 
  return x_trajectory, y_trajectory
 
   
x_trajectory, y_trajectory = run(robot, 0.1, 1.0)

n = len(x_trajectory)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

ax1.plot(x_trajectory, y_trajectory, 'g', label='PDcontroller')

ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

plt.show()
