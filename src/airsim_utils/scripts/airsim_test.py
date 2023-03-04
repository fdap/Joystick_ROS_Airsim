import time
import math

import numpy as np
from inputs import get_gamepad
from threading import Thread, Lock
import airsim


max_v, max_turn_rate = 3, 45

class GamepadCtrl:
  def __init__(self):
    self.client = airsim.MultirotorClient()
    self.client.confirmConnection()
    self.client.reset()
    self.client.enableApiControl(True)
    self.client.armDisarm(True)

    print("Take off")
    self.client.takeoffAsync().join()
    self.client.hoverAsync().join()

    self.v, self.yaw_rate, self.vz = 0, 0, 0
    self.total_distance = 0
    self.finish = False
    self.lock = Lock()

  def readGamepad(self):
    print("Gamepad control")
    while True:
      events = get_gamepad()
      self.lock.acquire()
      for event in events:
        if event.code == 'ABS_Y': self.v = event.state / 32767.0 * max_v
        elif event.code == 'ABS_RX': self.yaw_rate = event.state / 32767.0 * max_turn_rate
        elif event.code == 'BTN_TR':
            self.vz = -3 if event.state == 1 else 0
        elif event.code == 'BTN_TL':
            self.vz = 3 if event.state == 1 else 0
        elif event.code == 'BTN_SELECT' or event.code == 'BTN_START': self.finish = True
        elif event.code == 'BTN_SOUTH' and event.state == 1:
          cur_pose = self.client.simGetVehiclePose()
          print(cur_pose.position)
          print(airsim.to_eularian_angles(cur_pose.orientation)[2])
          print("Cur distance: {:.2f}m".format(self.total_distance))
          print()
        elif event.code == 'BTN_EAST' and event.state == 1:
          self.total_distance = 0
      if self.finish:
        self.lock.release()
        break
      self.lock.release()

  def start(self):
    thread = Thread(target=self.readGamepad)
    old_pos = self.client.simGetVehiclePose().position
    thread.start()

    while True:
      self.lock.acquire()
      cur_pose = self.client.simGetVehiclePose()
      cur_pos = cur_pose.position
      self.total_distance += \
        np.sqrt(np.square(cur_pos.x_val-old_pos.x_val)+np.square(cur_pos.y_val-old_pos.y_val))
      old_pos = cur_pos
      _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
      vx = self.v * math.cos(cur_yaw)
      vy = self.v * math.sin(cur_yaw)
      self.client.moveByVelocityAsync(
        vx, vy, self.vz, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, self.yaw_rate))

      if self.finish:
        self.lock.release()
        break
      self.lock.release()
      time.sleep(0.05)

    print("Land")
    print("Total distance: {:.2f}m".format(self.total_distance))
    self.client.landAsync()
    self.client.enableApiControl(False)

if __name__ == '__main__':
  gamepadctrl = GamepadCtrl()
  gamepadctrl.start()