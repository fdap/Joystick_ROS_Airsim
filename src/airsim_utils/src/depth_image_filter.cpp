#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>

#include <image_transport/image_transport.h>

namespace enc = sensor_msgs::image_encodings;

image_transport::Publisher pub_depth_;

float maxDepthValue = 100.0;
float minDepthValue = 0.2;

void depthImageHandler(const sensor_msgs::Image::ConstPtr& depthImageIn)
{
  if (depthImageIn->encoding == enc::TYPE_32FC1)
  {
    sensor_msgs::ImagePtr depth_msg( new sensor_msgs::Image );
    depth_msg->header   = depthImageIn->header;
    depth_msg->height   = depthImageIn->height;
    depth_msg->width    = depthImageIn->width;
    depth_msg->encoding = enc::TYPE_32FC1;
    depth_msg->step     = depthImageIn->width * (enc::bitDepth(depth_msg->encoding) / 8);
    depth_msg->data.resize(depth_msg->height * depth_msg->step);
    const float* raw_data = reinterpret_cast<const float*>(&depthImageIn->data[0]);
    float* depth_data = reinterpret_cast<float*>(&depth_msg->data[0]);
    for (unsigned index = 0; index < depth_msg->height * depth_msg->width; ++index)
    {
      float raw = raw_data[index];
      float depth_value = std::isnan(raw) ? maxDepthValue : raw;
      depth_value = std::min(depth_value, maxDepthValue);
      depth_value = std::max(depth_value, minDepthValue);
      depth_data[index] = depth_value;
    }
    pub_depth_.publish(depth_msg);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "depth_image_filter");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");
  nhPrivate.getParam("maxDepthValue", maxDepthValue);
  nhPrivate.getParam("minDepthValue", minDepthValue);
  
  ros::Subscriber subDepthImage = nh.subscribe<sensor_msgs::Image> ("/airsim_node/drone0/cam/DepthPerspective", 1, depthImageHandler);
  
  image_transport::ImageTransport it(nh);
  pub_depth_ = it.advertise("/airsim_node/drone0/cam/DepthPerspective/converted", 1);

  ros::spin();

  return 0;
}
