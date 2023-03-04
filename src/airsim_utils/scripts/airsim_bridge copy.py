#!/usr/bin/env python3
import airsim
import numpy as np
import math
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Image

# client = airsim.VehicleClient()
client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)

# take off
print("Take off")
client.takeoffAsync().join()
client.hoverAsync().join()

max_v, max_turn_rate = 3, 45
v, yaw_rate, vz = 0,0,0
total_distance = 0
finish = False
old_pos = client.simGetVehiclePose().position


vehicle_odometry = Odometry()

joy_data = Joy()

def odometry_callback(data):
    global vehicle_odometry
    vehicle_odometry = data
    vehicle_position = airsim.Vector3r(vehicle_odometry.pose.pose.position.x, -vehicle_odometry.pose.pose.position.y, -vehicle_odometry.pose.pose.position.z)
    vehicle_orientation = airsim.Quaternionr(vehicle_odometry.pose.pose.orientation.x, -vehicle_odometry.pose.pose.orientation.y, -vehicle_odometry.pose.pose.orientation.z, vehicle_odometry.pose.pose.orientation.w)
    client.simSetVehiclePose(airsim.Pose(vehicle_position, vehicle_orientation), True)

def joy_callback(data):
    global joy_data
    global finish
    global v
    global yaw_rate
    global vz
    global total_distance
    global old_pos

    joy_data = data
    if finish:
        return
    
    vz = 0

    if joy_data.buttons[7] == 1:
        print("Cur distance: {:.2f}m".format(total_distance))
    elif joy_data.buttons[8] == 1:
        print("Land")
        finish = True
    elif joy_data.axes[5] < 1.0:
        v = max_v*joy_data.axes[4]
        yaw_rate = -1*max_turn_rate*joy_data.axes[3]
    elif joy_data.buttons[4] == 1:
        vz = 3
    elif joy_data.buttons[5] == 1:
        vz = -3
        
    # if finish:
    #     return

    # cur_pose = client.simGetVehiclePose()
    # cur_pos = cur_pose.position
    # total_distance += \
    #         np.sqrt(np.square(cur_pos.x_val-old_pos.x_val)+np.square(cur_pos.y_val-old_pos.y_val))
    # old_pos = cur_pos
    # _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
    # print("velocity:%.3f, steering:%.3f" % (v, yaw_rate))
    # vx = v * math.cos(cur_yaw)
    # vy = v * math.sin(cur_yaw)
    # client.moveByVelocityAsync(vx, vy, vz, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, yaw_rate))


def image_callback(data):
    global finish
    global v
    global yaw_rate
    global vz
    global total_distance
    global old_pos

    if finish:
        print("Total distance: {:.2f}m".format(total_distance))
        client.landAsync()
        client.enableApiControl(False)

    cur_pose = client.simGetVehiclePose()
    cur_pos = cur_pose.position
    total_distance += \
            np.sqrt(np.square(cur_pos.x_val-old_pos.x_val)+np.square(cur_pos.y_val-old_pos.y_val))
    old_pos = cur_pos
    _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
    print("velocity:%.3f, steering:%.3f" % (v, yaw_rate))
    vx = v * math.cos(cur_yaw)
    vy = v * math.sin(cur_yaw)
    client.moveByVelocityAsync(vx, vy, vz, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, yaw_rate))






if __name__ == '__main__':
    rospy.init_node('airsim_bridge', anonymous=True)
    
    # rospy.Subscriber("/state_estimation", Odometry, odometry_callback)

    rospy.Subscriber("/joy", Joy, joy_callback, queue_size = 10)

    rospy.Subscriber("/airsim_node/drone0/cam/Scene", Image, image_callback, queue_size = 1)

    # print("the flight state:", finish)

    # if not finish:
    #     cur_pose = client.simGetVehiclePose()
    #     cur_pos = cur_pose.position
    #     total_distance += \
    #         np.sqrt(np.square(cur_pos.x_val-old_pos.x_val)+np.square(cur_pos.y_val-old_pos.y_val))
    #     old_pos = cur_pos
    #     _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
    #     print("velocity:%.3f, steering:%.3f" % (v, yaw_rate))
    #     vx = v * math.cos(cur_yaw)
    #     vy = v * math.sin(cur_yaw)
    #     client.moveByVelocityAsync(
    #         vx, vy, vz, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, yaw_rate))

    # else:
    #     pass
    rospy.spin()

    
