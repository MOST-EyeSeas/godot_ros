#include "godot_ros/demos/view_port.hpp"

ViewPort::ViewPort() {
  if (!rclcpp::ok()) {
    rclcpp::init(0, nullptr);
  }
}

ViewPort::~ViewPort() {
  // Note: We don't call rclcpp::shutdown() here to allow multiple ViewPort instances
}

void ViewPort::set_node_name(const String& node_name) {
  m_node = std::make_shared<rclcpp::Node>(node_name.utf8().get_data());
}

void ViewPort::spin_some() {
  if (m_node) {
    rclcpp::spin_some(m_node);
  } else {
    WARN_PRINT("Node not initialized. Call set_node_name() first.");
  }
}

void ViewPort::set_topic(const String& topic) {
  if (m_node) {
    m_pub = m_node->create_publisher<sensor_msgs::msg::Image>(topic.utf8().get_data(), 10);
  } else {
    WARN_PRINT("Node not initialized. Call set_node_name() first.");
  }
}

void ViewPort::pubImage(const Ref<Image> & img) {
  if (!m_pub) {
    WARN_PRINT("Publisher not initialized. Call set_topic() first.");
    return;
  }

  auto msg = std::make_unique<sensor_msgs::msg::Image>();
  msg->height = img->get_height();
  msg->width = img->get_width();
  msg->encoding = "rgb8";
  msg->is_bigendian = false;
  msg->step = img->get_data().size() / msg->height;
  msg->data.resize(img->get_data().size());
  std::memcpy(&msg->data[0], img->get_data().ptr(), img->get_data().size());

  m_pub->publish(std::move(msg));
}

void ViewPort::_bind_methods() {
  ClassDB::bind_method(D_METHOD("set_node_name", "node_name"), &ViewPort::set_node_name);
  ClassDB::bind_method(D_METHOD("set_topic", "topic"), &ViewPort::set_topic);
  ClassDB::bind_method(D_METHOD("pubImage", "img"), &ViewPort::pubImage);
  ClassDB::bind_method(D_METHOD("spin_some"), &ViewPort::spin_some);
}