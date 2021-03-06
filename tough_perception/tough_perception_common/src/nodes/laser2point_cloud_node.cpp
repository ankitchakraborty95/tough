#include <tough_perception_common/laser2point_cloud.h>
#include <tough_perception_common/perception_common_names.h>
#include <tough_common/tough_common_names.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "laser2point_cloud");
  ros::NodeHandle nh;
  Laser2PointCloud laser2point(nh, 
		  PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_SCAN_TOPIC, 
		  TOUGH_COMMON_NAMES::WORLD_TF,
		  PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_CLOUD_TOPIC,
		  PERCEPTION_COMMON_NAMES::MULTISENSE_LASER_CLOUD_TOPIC2);

  ros::spin();

  return 0;
}
