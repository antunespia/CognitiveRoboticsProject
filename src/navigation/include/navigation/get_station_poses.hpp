#ifndef NAV2_PLUGINS_GET_STATION_POSES_HPP
#define NAV2_PLUGINS_GET_STATION_POSES_HPP

#include <memory>
#include <vector>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_behavior_tree/bt_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"

#include "behaviortree_cpp/action_node.h"

namespace nav2_behavior_tree
{

class /* ... */ : public BT::ActionNodeBase {
public:
	GetStationPoses(const std::string & xml_tag_name, const BT::NodeConfiguration & conf);

	static BT::PortsList providedPorts() {
		auto rv = {
			BT::/* ... */<std::string>("global_frame", "Global reference frame"),
			BT::InputPort</* ... */>("robot_base_frame", "robot base frame"),
			BT::/* ... */<geometry_msgs::msg::PoseStamped>("pickup_point_a", "Location of pickup point A"),
			BT::/* ... */("pickup_point_b", "Location of pickup point B"),
			BT::/* ... */("art_location", "Location the tourist wants to visit")
    		};
		return rv;
	}

private:
	void halt() override {}

	BT::NodeStatus /* ... */() override;

	std::string global_frame_, robot_base_frame_;
	std::shared_ptr<tf2_ros::Buffer> tf_;
};

}

#endif
