#ifndef GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP
#define GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP
#include <cstring>
#include <iostream>

#include "core/object/ref_counted.h"
#include "core/io/image.h"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class ViewPort : public RefCounted {
  GDCLASS(ViewPort, RefCounted);
public:
  ViewPort();
  ~ViewPort();

  void spin_some();
  void set_topic(const String& topic);
  void pubImage(const Ref<Image> & img);

protected:
  static void _bind_methods();

  std::shared_ptr<rclcpp::Node> m_node;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr m_pub;
  std::unique_ptr<sensor_msgs::msg::Image> m_msg;
};
#endif // GODOT__GODOT_ROS__DEMOS__VIEW_PORT_HPP