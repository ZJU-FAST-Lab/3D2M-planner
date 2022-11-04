#include "planner_manager/planner_manager.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "planner_manager_node");
  ros::NodeHandle nh("~");

  PlannerManager scf_planner;
  scf_planner.init(nh);

  ros::Duration(1.0).sleep();
  ros::spin();

  return 0;
}