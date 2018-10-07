#include "find_road.h"
#include "move_car.h"
#include <math.h>  

int main(int argc, char** argv)
{
  float speed = 1.3;  
  float Ka = 1.0/200;
  float max_speed = 3;
  float min_speed = 0.1;
  float Ks = 0.06;
  std::vector<cv::Vec4i> lines_right;
  std::vector<cv::Vec4i> lines_left;
  
  if (argc > 1) { speed = atof(argv[1]); }
  if (argc > 2) { Ka = atof(argv[2]); }
  if (argc > 3) { max_speed = atof(argv[3]); }
  if (argc > 4) { min_speed = atof(argv[4]); }


  ros::init(argc, argv, "find_road");
  RoadFinder rf;
  AckermannMover am;
  ros::Rate rate(20);
  
  while(ros::ok()) {
    // The speed is constant, but the steering angle is proportional
    //to the distance of the middle of the road from the middle of the image
 
    ros::spinOnce();
    float error = rf.imageWidth()/2 - rf.getMidpoint().x; // Error in number of pixels
    float speed_command = speed - abs(error)*Ks; // Here you could do something smarter
    // speed_command = speed;

    std::vector<cv::Vec4i> lines = rf.getLines();

    float right_angle_average = rf.get_right_angle();
    float left_angle_average = rf.get_left_angle();

    speed_command = std::max(min_speed, std::min(speed_command, max_speed));
    // ROS_INFO("Sending command speed=%f, steering_angle=%f, error=%f", speed_command, error*Ka, error);
    // ROS_INFO("Angulo der: %f Ángulo izq: %f", right_angle_average, left_angle_average);
    am.go(speed_command, error*Ka);
    rate.sleep();
  }
  return 0;
}
