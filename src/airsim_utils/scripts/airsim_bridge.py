#!/usr/bin/env python3
import airsim
import numpy as np
import math
import time

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy, Image
import sys

drone0 = "drone0"
drone1 = "drone1"

# client = airsim.VehicleClient()
client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()

client.enableApiControl(True, drone0)
client.armDisarm(True, drone0)

client.enableApiControl(True, drone1)
client.armDisarm(True, drone1)

# take off
print(drone0, "take off")
client.takeoffAsync(vehicle_name = drone0).join()
client.hoverAsync(vehicle_name = drone0).join()

print(drone1, "take off")
client.takeoffAsync(vehicle_name = drone1).join()
client.hoverAsync(vehicle_name = drone1).join()


# client.moveToZAsync(-3, 1, drone0).join()
# client.moveToZAsync(-3, 1, drone1).join()


max_v, max_turn_rate = 3, 45
v, yaw_rate, vz = 0,0,0
total_distance = 0
finish = False
old_pos = client.simGetVehiclePose(vehicle_name = drone0).position


shift = 0
v1, yaw_rate1, vz1 = 0,0,0
old_pos_1 = client.simGetVehiclePose(vehicle_name = drone1).position
total_distance1 = 0

land_or_not = False

vehicle_odometry = Odometry()

joy_data = Joy()


def joy_callback(data):
    global joy_data
    global finish
    global v
    global yaw_rate
    global vz
    global total_distance

    global shift
    global v1
    global yaw_rate1
    global vz1
    global total_distance1



    joy_data = data
    if finish:
        return
    
    if joy_data.buttons[1] == 1:
        shift = 1
    if joy_data.buttons[0] == 1:
        shift = 0

    # drone0 steering
    if shift == 0:
        vz = 0
        if joy_data.buttons[7] == 1:
            print("Cur distance of drone0: {:.2f}m".format(total_distance))
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
    elif shift == 1:
        vz1 = 0
        if joy_data.buttons[7] == 1:
            print("Cur distance of drone1: {:.2f}m".format(total_distance1))
        elif joy_data.buttons[8] == 1:
            print("Land")
            finish = True
        elif joy_data.axes[5] < 1.0:
            v1 = max_v*joy_data.axes[4]
            yaw_rate1 = -1*max_turn_rate*joy_data.axes[3]
        elif joy_data.buttons[4] == 1:
            vz1 = 3
        elif joy_data.buttons[5] == 1:
            vz1 = -3

        
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

    global shift
    global land_or_not

    if shift == 1 or land_or_not:
        return

    if finish:
        print("Total distance of drone0: {:.2f}m".format(total_distance))
        print("Total distance of drone1: {:.2f}m".format(total_distance1))
        client.landAsync(vehicle_name = drone0).join()
        client.landAsync(vehicle_name = drone1).join()
        client.armDisarm(False, vehicle_name = drone0)
        client.enableApiControl(False, vehicle_name = drone0)
        client.armDisarm(False, vehicle_name = drone1)
        client.enableApiControl(False, vehicle_name = drone1)
        land_or_not = True
        return

    cur_pose = client.simGetVehiclePose(vehicle_name = drone0)
    cur_pos = cur_pose.position
    total_distance += \
            np.sqrt(np.square(cur_pos.x_val-old_pos.x_val)+np.square(cur_pos.y_val-old_pos.y_val))
    old_pos = cur_pos
    _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
    print("Drone 0: velocity:%.3f, steering:%.3f" % (v, yaw_rate))
    vx = v * math.cos(cur_yaw)
    vy = v * math.sin(cur_yaw)
    client.moveByVelocityAsync(vx, vy, vz, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, yaw_rate), vehicle_name = drone0)


def image_re_callback(data):
    global finish
    global v1
    global yaw_rate1
    global vz1
    global total_distance1
    global old_pos_1

    global land_or_not

    if shift == 0 or land_or_not:
        return

    if finish:
        print("Total distance of drone0: {:.2f}m".format(total_distance))
        print("Total distance of drone1: {:.2f}m".format(total_distance1))
        client.landAsync(vehicle_name = drone0)
        client.landAsync(vehicle_name = drone1).join()
        client.armDisarm(False, vehicle_name = drone0)
        client.enableApiControl(False, vehicle_name = drone0)
        client.armDisarm(False, vehicle_name = drone1)
        client.enableApiControl(False, vehicle_name = drone1)
        land_or_not = True
        return

    cur_pose = client.simGetVehiclePose(vehicle_name = drone0)
    cur_pos = cur_pose.position
    total_distance1 += \
            np.sqrt(np.square(cur_pos.x_val-old_pos_1.x_val)+np.square(cur_pos.y_val-old_pos_1.y_val))
    old_pos_1 = cur_pos
    _, _, cur_yaw = airsim.to_eularian_angles(cur_pose.orientation)
    print("Drone 1: velocity:%.3f, steering:%.3f" % (v1, yaw_rate1))
    vx = v1 * math.cos(cur_yaw)
    vy = v1 * math.sin(cur_yaw)
    client.moveByVelocityAsync(vx, vy, vz1, 999, airsim.DrivetrainType.MaxDegreeOfFreedom, airsim.YawMode(True, yaw_rate1), vehicle_name = drone1)




if __name__ == '__main__':
    rospy.init_node('airsim_bridge', anonymous=True)
    
    # rospy.Subscriber("/state_estimation", Odometry, odometry_callback)

    rospy.Subscriber("/joy", Joy, joy_callback, queue_size = 10)

    rospy.Subscriber("/airsim_node/drone0/cam/Scene", Image, image_callback, queue_size = 1)

    rospy.Subscriber("/airsim_node/drone1/cam1/Scene", Image, image_re_callback, queue_size = 1)

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

    if finish:
        sys.exit(0)

    
