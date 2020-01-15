#!/usr/bin/env python
# -*- coding:utf-8 -*- 

import random
 
import numpy as np
 
import matplotlib.pyplot as plt

# ------------------------------------------------
 
#
 
# this is the Robot class
 
#
 
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
 
 
 
############## ADD / MODIFY CODE BELOW ####################
 
#------------------------------------------------------------------------
 
#
 
# run - does a single control run
 

def make_robot():
  """
   Resets the robot back to the initial position and drift.
   You'll want to call this after you call `run`.
  """

  robot = Robot()

  robot.set(0.0, 1.0, 0.2)  # 原代码值0.0

  robot.set_steering_drift(10.0 / 180.0 * np.pi)

  #robot.set_noise(0.1,0.1) # 这行自己加了测试用的，加入噪声以后，轨迹会发生偏移，仅加入上面那行，也会发生偏移。

  return robot


def run(robot, params, n=100, speed=1.0):

  x_trajectory = []

  y_trajectory = []

  err = 0

  prev_cte = 0

  int_cte = 0

  cte = 0

  startX = robot.x

  startY = robot.y

  startOrientation = robot.orientation

  for i in range(2 * n):

    diff_cte = cte - prev_cte

    int_cte += cte

    prev_cte = cte

    steer = -params[0] * cte - params[1] * diff_cte - params[2] * int_cte

    robot.move(steer, speed)

    x_trajectory.append(robot.x)

    y_trajectory.append(robot.y)

    x1 = robot.x

    y1 = startY + (x1 - startX) * np.tan(startOrientation)

    cte = (robot.y - y1) * np.cos(startOrientation)

    if i >= n:

      err += cte ** 2

  return x_trajectory, y_trajectory, err / n


def twiddle(tol=0.2):

  # Don't forget to call `make_robot` before every call of `run`!

  p = [0.0, 0.0, 0.0]  # params

  dp = [1.0, 1.0, 1.0]  # dparams

  #dp[2] = 0 # for test, do not use I item params.

  robot = make_robot()

  x_trajectory, y_trajectory, best_err = run(robot, p)

  # TODO: twiddle loop here

  # Begin: From the video

  # 要调节的 PID 参数的个数 P I D

  #n_params = 3

  #dparams = [1.0 for row in range(n_params)]

  #params = [0.0 for row in range(n_params)]

  params = p   # params 里面存的是最优的各个PID参数。

  dparams = dp  # dparams 里面存的应该是调节params各参数对应的步长

  n = 0

  while sum(dparams) > tol:

    for i in range(len(params)):

      params[i] += dparams[i]

      # 这行是自己加的，因为视频上使用的是run(params)，可能它里面已经重新初始化了robot.
      robot = make_robot()

      x_trajectory, y_trajectory, b_err = run(robot, params)

      if b_err < best_err:

        best_err = b_err

        dparams[i] *= 1.1

      else:

        params[i] -= 2.0 * dparams[i]

        robot = make_robot()  # 这行是自己加的

        x_trajectory, y_trajectory, b_err = run(robot, params)

        if b_err < best_err:

          best_err = b_err

          dparams[i] *= 1.1

        else:

          params[i] += dparams[i]

          dparams[i] *= 0.9

    n += 1

    print('Twiddle #', n, params, ' ->', best_err)

  print(' ')

  p = params  # 这行自己加的，以对应下方的返回值。

  # End: From the video

  return p, best_err

   
params, err = twiddle()

print("Final twiddle error ={}".format(err))

print("Final params {}", params)

robot = make_robot()

x_trajectory, y_trajectory, err = run(robot, params)

n = len(x_trajectory)


fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 8))

ax1.plot(x_trajectory, y_trajectory, 'g', label='Twiddle PID controller')

ax1.plot(x_trajectory, np.zeros(n), 'r', label='reference')

plt.show()
