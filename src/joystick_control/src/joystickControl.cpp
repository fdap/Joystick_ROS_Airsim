#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <gazebo_msgs/ModelState.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/Joy.h>


using namespace std;

const double PI = 3.1415926;

double realtimeFactor = 1.0;
double windCoeff = 0.05;
double maxRollPitchRate = 20.0;
double rollPitchSmoothRate = 0.1;
double sensorPitch = 0;

float vehicleX = 0;
float vehicleY = 0;
float vehicleZ = 1.0;

float vehicleVelX = 0;
float vehicleVelY = 0;
float vehicleVelZ = 0;
float vehicleVelXG = 0;
float vehicleVelYG = 0;
float vehicleVelZG = 0;

float vehicleRoll = 0;
float vehiclePitch = 0;
float vehicleYaw = 0;

float vehicleRollCmd = 0;
float vehiclePitchCmd = 0;
float vehicleYawRate = 0;

double lookAheadScale = 0.2;
double minSpeed = 0.5;
double maxSpeed = 2.0;
double desiredSpeed = minSpeed;
double velXYGain = 0.3;
double posXYGain = 0.05;
double stopVelXYGain = 0.2;
double stopPosXYGain = 0.2;
double smoothIncrSpeed = 0.75;
double maxRollPitch = 30.0;
double yawGain = 2.0;
double maxRateByYaw = 60.0;
double posZGain = 1.5;
double maxVelByPosZ = 0.5;
double manualSpeedXY = 2.0;
double manualSpeedZ = 1.0;
double manualYawRate = 60.0;
double stopDis = 0.5;
double slowDis = 2.0;

float joyFwd = 0;
float joyLeft = 0;
float joyUp = 0;
float joyYaw = 0;

double joyDeadband = 0.1;


void joystickHandler(const sensor_msgs::Joy::ConstPtr& joy)
{
  joyFwd = joy->axes[4];
  if (fabs(joyFwd) < joyDeadband) joyFwd = 0;
  joyLeft = joy->axes[3];
  if (fabs(joyLeft) < joyDeadband) joyLeft = 0;
  joyUp = joy->axes[1];
  if (fabs(joyUp) < joyDeadband) joyUp = 0;
  joyYaw = joy->axes[0];
  if (fabs(joyYaw) < joyDeadband) joyYaw = 0;

  if (desiredSpeed < maxSpeed * joyFwd) desiredSpeed = maxSpeed * joyFwd;
  else if (desiredSpeed > maxSpeed * (joyFwd + joyDeadband)) desiredSpeed = maxSpeed * (joyFwd + joyDeadband);
  if (desiredSpeed < minSpeed || joyFwd <= joyDeadband) desiredSpeed = minSpeed;
  else if (desiredSpeed > maxSpeed) desiredSpeed = maxSpeed;
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "vehicleSimulator");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  nhPrivate.getParam("realtimeFactor", realtimeFactor);
  nhPrivate.getParam("windCoeff", windCoeff);
  nhPrivate.getParam("maxRollPitchRate", maxRollPitchRate);
  nhPrivate.getParam("rollPitchSmoothRate", rollPitchSmoothRate);
  nhPrivate.getParam("sensorPitch", sensorPitch);
  nhPrivate.getParam("vehicleX", vehicleX);
  nhPrivate.getParam("vehicleY", vehicleY);
  nhPrivate.getParam("vehicleZ", vehicleZ);
  nhPrivate.getParam("vehicleYaw", vehicleYaw);

  ros::Subscriber subJoystick = nh.subscribe<sensor_msgs::Joy> ("/joy", 5, joystickHandler);

  ros::Publisher pubVehicleOdom = nh.advertise<nav_msgs::Odometry> ("/state_estimation", 5);

  nav_msgs::Odometry odomData;
  odomData.header.frame_id = "map";
  odomData.child_frame_id = "vehicle";

  printf("\nSimulation started.\n\n");

  ros::Rate rate(200 * realtimeFactor);
  bool status = ros::ok();
  while (status) {
    ros::spinOnce();

    float vehicleRecRoll = vehicleRoll;
    float vehicleRecPitch = vehiclePitch;

    float desiredRoll = -stopVelXYGain * (manualSpeedXY * joyLeft - vehicleVelY) - posXYGain * lookAheadScale * manualSpeedXY * joyLeft;
    if (desiredRoll > maxRollPitch * PI / 180.0) desiredRoll = maxRollPitch * PI / 180.0;
    else if (desiredRoll < -maxRollPitch * PI / 180.0) desiredRoll = -maxRollPitch * PI / 180.0;

    float desiredPitch = stopVelXYGain * (manualSpeedXY * joyFwd - vehicleVelX) + posXYGain * lookAheadScale * manualSpeedXY * joyFwd;
    if (desiredPitch > maxRollPitch * PI / 180.0) desiredPitch = maxRollPitch * PI / 180.0;
    else if (desiredPitch < -maxRollPitch * PI / 180.0) desiredPitch = -maxRollPitch * PI / 180.0;

    vehicleRollCmd = desiredRoll;
    vehiclePitchCmd = desiredPitch;
    vehicleYawRate = manualSpeedZ * joyUp;
    vehicleVelZG = manualYawRate * joyYaw * PI / 180.0;

    if (vehicleRollCmd - vehicleRoll > maxRollPitchRate / 200.0) vehicleRoll += maxRollPitchRate / 200.0;
    else if (vehicleRollCmd - vehicleRoll < -maxRollPitchRate / 200.0) vehicleRoll -= maxRollPitchRate / 200.0;
    else vehicleRoll = vehicleRollCmd;
    vehicleRoll = rollPitchSmoothRate * vehicleRoll + (1.0 - rollPitchSmoothRate) * vehicleRecRoll;

    if (vehiclePitchCmd - vehiclePitch > maxRollPitchRate / 200.0) vehiclePitch += maxRollPitchRate / 200.0;
    else if (vehiclePitchCmd - vehiclePitch < -maxRollPitchRate / 200.0) vehiclePitch -= maxRollPitchRate / 200.0;
    else vehiclePitch = vehiclePitchCmd;
    vehiclePitch = rollPitchSmoothRate * vehiclePitch + (1.0 - rollPitchSmoothRate) * vehicleRecPitch;

    float vehicleAccX = 9.8 * tan(vehiclePitch);
    float vehicleAccY = -9.8 * tan(vehicleRoll) / cos(vehiclePitch);

    if (vehicleVelXG < 0) vehicleVelXG += windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    else vehicleVelXG -= windCoeff * vehicleVelXG * vehicleVelXG  / 200.0;
    if (vehicleVelYG < 0) vehicleVelYG += windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;
    else vehicleVelYG -= windCoeff * vehicleVelYG * vehicleVelYG  / 200.0;

    vehicleVelXG += (vehicleAccX * cos(vehicleYaw) - vehicleAccY * sin(vehicleYaw)) / 200.0;
    vehicleVelYG += (vehicleAccX * sin(vehicleYaw) + vehicleAccY * cos(vehicleYaw)) / 200.0;

    float velX1 = vehicleVelXG * cos(vehicleYaw) + vehicleVelYG * sin(vehicleYaw);
    float velY1 = -vehicleVelXG * sin(vehicleYaw) + vehicleVelYG * cos(vehicleYaw);
    float velZ1 = vehicleVelZG;

    float velX2 = velX1 * cos(vehiclePitch) - velZ1 * sin(vehiclePitch);
    float velY2 = velY1;
    float velZ2 = velX1 * sin(vehiclePitch) + velZ1 * cos(vehiclePitch);

    vehicleVelX = velX2;
    vehicleVelY = velY2 * cos(vehicleRoll) + velZ2 * sin(vehicleRoll);
    vehicleVelZ = -velY2 * sin(vehicleRoll) + velZ2 * cos(vehicleRoll);

    vehicleX += vehicleVelXG / 200.0;
    vehicleY += vehicleVelYG / 200.0;
    vehicleZ += vehicleVelZG / 200.0;
    vehicleYaw += vehicleYawRate / 200.0;

    ros::Time timeNow = ros::Time::now();

    geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(vehicleRoll, vehiclePitch, vehicleYaw);

    // publish 200Hz odometry messages
    odomData.header.stamp = timeNow;
    odomData.pose.pose.orientation = geoQuat;
    odomData.pose.pose.position.x = vehicleX;
    odomData.pose.pose.position.y = vehicleY;
    odomData.pose.pose.position.z = vehicleZ;
    odomData.twist.twist.angular.x = 200.0 * (vehicleRoll - vehicleRecRoll);
    odomData.twist.twist.angular.y = 200.0 * (vehiclePitch - vehicleRecPitch);
    odomData.twist.twist.angular.z = vehicleYawRate;
    odomData.twist.twist.linear.x = vehicleVelX;
    odomData.twist.twist.linear.y = vehicleVelY;
    odomData.twist.twist.linear.z = vehicleVelZ;
    pubVehicleOdom.publish(odomData);

    status = ros::ok();
    rate.sleep();
  }

  return 0;
}
