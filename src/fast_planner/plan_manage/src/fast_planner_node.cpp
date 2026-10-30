
#include <fast_planner/fast_planner.h>
#include <plan_manage/kino_replan_fsm.h>

using namespace fast_planner;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FastPlanner>();

  node->declare_parameter<float>("bspline/limit_vel", 0);
  node->declare_parameter<float>("bspline/limit_acc", 0);
  node->declare_parameter<float>("bspline/limit_ratio", 0);

  node->declare_parameter<int>("planner_node/planner", -1);
  rclcpp::Parameter planner_param = node->get_parameter("planner_node/planner"); 

  int planner = planner_param.as_int();
  // RCLCPP_INFO(node->get_logger(), "The planner value is (int) : %s",
  //               planner_param.value_to_string().c_str());

  if (planner != 1) {
      RCLCPP_FATAL(node->get_logger(),
                   "Unsupported planner_node/planner=%d; this build supports planner 1 (kino)", planner);
      rclcpp::shutdown();
      return 2;
  }

  // # Initialize the kino Class
  std::shared_ptr<KinoReplanFSM> kino_replan = std::make_shared<KinoReplanFSM>();
  kino_replan->init(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
}
