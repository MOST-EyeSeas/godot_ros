#include "godot_ros/demos/view_port.hpp"

ViewPort::ViewPort() {
  rclcpp::init(0, nullptr);
  m_node = std::make_shared<rclcpp::Node>("godot_image_node");
}

ViewPort::~ViewPort() {
  rclcpp::shutdown();
}

void ViewPort::spin_some() {
  rclcpp::spin_some(m_node);
}

void ViewPort::set_topic(const String& topic) {
  m_pub = m_node->create_publisher<sensor_msgs::msg::Image>(topic.utf8().get_data(), 10);
}

void ViewPort::pubImage(const Ref<Image> & img) {
  if (!m_pub) {
    WARN_PRINT("Publisher not initialized. Call set_topic() first.");
    return;
  }

  m_msg = std::make_unique<sensor_msgs::msg::Image>();
  m_msg->height = img->get_height();
  m_msg->width = img->get_width();
  m_msg->encoding = "rgb8";
  m_msg->is_bigendian = false;
  m_msg->step = img->get_data().size() / m_msg->height;
  m_msg->data.resize(img->get_data().size());
  std::memcpy(&m_msg->data[0], img->get_data().ptr(), img->get_data().size());

  m_pub->publish(std::move(m_msg));
}

void ViewPort::_bind_methods() {
  ClassDB::bind_method(D_METHOD("set_topic", "topic"), &ViewPort::set_topic);
  ClassDB::bind_method(D_METHOD("pubImage", "img"), &ViewPort::pubImage);
  ClassDB::bind_method(D_METHOD("spin_some"), &ViewPort::spin_some);
}