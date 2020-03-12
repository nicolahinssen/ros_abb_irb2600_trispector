#include <ros/ros.h>
#include <myworkcell_core/LocalizePart.h>
#include <tf/tf.h>
#include <moveit/move_group_interface/move_group_interface.h>

class ScanNPlan
{
public:
  ScanNPlan(ros::NodeHandle& nh)
  {
    vision_client_ = nh.serviceClient<myworkcell_core::LocalizePart>("localize_part");
  }

  void start(const std::string& base_frame)
  {
    ROS_INFO("Attempting to localize part");

    // Localize the part
    myworkcell_core::LocalizePart srv;
    srv.request.base_frame = base_frame;
    ROS_INFO_STREAM("Requesting pose in base frame: " << base_frame);

    if (!vision_client_.call(srv))
    {
      ROS_ERROR("Could not localize part");
      return;
    }
    ROS_INFO_STREAM("part localized: " << srv.response);

    geometry_msgs::Pose move_target = srv.response.pose;
    moveit::planning_interface::MoveGroupInterface move_group("manipulator_tcp");

    move_group.setPoseReferenceFrame(base_frame);
    move_group.setPoseTarget(move_target);
    move_group.move();

    move_target.position.x -= 1.00;
    move_target.position.y -= 0.40;
    move_target.position.z -= 0.60;

    move_group.setPoseTarget(move_target);
    move_group.move();

    // Cartesian
    std::vector<geometry_msgs::Pose> waypoints;

    move_target.position.x += 1.60;
    waypoints.push_back(move_target);

    move_target.position.x -= 1.60;
    waypoints.push_back(move_target);

    moveit::planning_interface::MoveGroupInterface::Plan move_plan;
    moveit_msgs::RobotTrajectory trajectory;

    const double jump_threshold = 0.20;
    const double eef_step = 0.10;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    ROS_INFO("Computing cartesian path... (%.2f%% achieved)", fraction * 100.0);

    sleep(5.0);

    move_plan.trajectory_ = trajectory;
    move_group.execute(move_plan);

//    int i;
//    for(i = 0; i <= 100; i++) {
//      move_target.position.x += 1.60;
//      move_group.setPoseTarget(move_target);
//      move_group.move();
//
//      move_target.position.x -= 1.60;
//      move_group.setPoseTarget(move_target);
//      move_group.move();
//    }
  }

private:
  // Planning components
  ros::ServiceClient vision_client_;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "myworkcell_node");
  ros::AsyncSpinner async_spinner(1);
  async_spinner.start();
  ros::NodeHandle nh;
  ros::NodeHandle private_node_handle ("~");

  std::string base_frame;
  private_node_handle.param<std::string>("base_frame", base_frame, "world");

  ROS_INFO("ScanNPlan node has been initialized");

  ScanNPlan app(nh);

  ros::Duration(.5).sleep();  // wait for the class to initialize
  app.start(base_frame);

  ros::waitForShutdown();
}
