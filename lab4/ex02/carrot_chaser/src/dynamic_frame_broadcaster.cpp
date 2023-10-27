#include <chrono>
#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;

const double PI = 3.141592653589793238463;

class DynamicFrameBroadcaster : public rclcpp::Node
{
public:
	DynamicFrameBroadcaster()
	: Node("dynamic_frame_broadcaser")
	{
		radius_ = this->declare_parameter<std::string>("radius");
		direction_of_rotation_ = this->declare_parameter<std::string>("direction_of_rotation");
		
		tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
		timer_ = this->create_wall_timer(
			100ms, std::bind(&DynamicFrameBroadcaster::broadcast_timer_callback, this));
	}
private:
	rclcpp::TimerBase::SharedPtr timer_;
	std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
	
	std::string radius_;
	std::string direction_of_rotation_;
	
	void broadcast_timer_callback()
	{
		rclcpp::Time now = this->get_clock()->now();
		
		double x = 0.2 * now.seconds() * PI;
		float radius = std::atof(radius_.c_str());
		int direction_of_rotation = std::atoi(direction_of_rotation_.c_str());
		
		geometry_msgs::msg::TransformStamped t;
		
		t.header.stamp = now;
		t.header.frame_id = "turtle1";
		t.child_frame_id = "carrot1";
		
		t.transform.translation.x = radius * (direction_of_rotation == 1 ? sin(x) : cos(x));
    	t.transform.translation.y = radius * (direction_of_rotation == 1? cos(x) : sin(x));
    	t.transform.translation.z = 0.0;
    	
    	t.transform.rotation.x = 0.0;
    	t.transform.rotation.y = 0.0;
    	t.transform.rotation.z = 0.0;
    	t.transform.rotation.w = 1.0;
    	
    	RCLCPP_INFO(this->get_logger(), "%f, %d", radius, direction_of_rotation);
    	tf_broadcaster_->sendTransform(t);
	}
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<DynamicFrameBroadcaster>());
	rclcpp::shutdown();
	return 0;
}


