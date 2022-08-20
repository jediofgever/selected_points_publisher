/*
 * Copyright 2019-2020 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ********************
 *  v0.1.0: drwnz (david.wong@tier4.jp) *
 *
 * selected_points_publisher.cpp
 *
 *  Created on: December 5th 2019
 */

#include "selected_points_publisher/selected_points_publisher.hpp"

namespace rviz_plugin_selected_points_publisher
{
  SelectedPointsPublisher::SelectedPointsPublisher()
  {
    node_ = std::make_shared<rclcpp::Node>("selected_points_publisher_node");
    rviz_cloud_topic_ = std::string("/rviz_selected_points");
    rviz_selected_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
      rviz_cloud_topic_, rclcpp::SystemDefaultsQoS());
    num_selected_points_ = 0;
    
    RCLCPP_INFO(
      node_->get_logger(),
      "SelectedPointsPublisher:: Started");
  }

  SelectedPointsPublisher::~SelectedPointsPublisher()
  {
  }


  int SelectedPointsPublisher::processKeyEvent(QKeyEvent * event, rviz_common::RenderPanel * panel)
  {
    if (event->type() == QKeyEvent::KeyPress) {
      if (event->key() == 'c' || event->key() == 'C') {
        RCLCPP_INFO(
          node_->get_logger(),
          "SelectedPointsPublisher::processKeyEvent Cleaning previous selection (selected area "
          "and points).");

        auto selection_manager = context_->getSelectionManager();
        auto selection = selection_manager->getSelection();
        selection_manager->removeHighlight();
        visualization_msgs::msg::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = context_->getFixedFrame().toStdString().c_str();
        marker.header.stamp = node_->now();
        marker.ns = "basic_shapes";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker.lifetime = rclcpp::Duration::from_seconds(0);
        num_selected_points_ = 0;
      } else if (event->key() == 'p' || event->key() == 'P') {
        RCLCPP_INFO(
          node_->get_logger(),
          "SelectedPointsPublisher.updateTopic selected points to topic ");
        rviz_selected_publisher_->publish(selected_points_);
      }
    }

    return 0;
  }

  int SelectedPointsPublisher::processMouseEvent(rviz_common::ViewportMouseEvent & event)
  {
    int flags = rviz_default_plugins::tools::SelectionTool::processMouseEvent(event);
    if (event.alt()) {
      selecting_ = false;
    } else {
      if (event.leftDown()) {
        selecting_ = true;
      }
    }

    if (selecting_) {
      if (event.leftUp()) {
        this->processSelectedArea();
      }
    }
    return flags;
  }

  int SelectedPointsPublisher::processSelectedArea()
  {
    auto selection_manager = context_->getSelectionManager();
    auto selection = selection_manager->getSelection();
    auto model = selection_manager->getPropertyModel();

    selected_points_.header.frame_id = context_->getFixedFrame().toStdString();
    selected_points_.height = 1;
    selected_points_.point_step = 4 * 4;
    selected_points_.is_dense = false;
    selected_points_.is_bigendian = false;
    selected_points_.fields.resize(4);

    selected_points_.fields[0].name = "x";
    selected_points_.fields[0].offset = 0;
    selected_points_.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[0].count = 1;

    selected_points_.fields[1].name = "y";
    selected_points_.fields[1].offset = 4;
    selected_points_.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[1].count = 1;

    selected_points_.fields[2].name = "z";
    selected_points_.fields[2].offset = 8;
    selected_points_.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[2].count = 1;

    selected_points_.fields[3].name = "intensity";
    selected_points_.fields[3].offset = 12;
    selected_points_.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
    selected_points_.fields[3].count = 1;

    int i = 0;
    while (model->hasIndex(i, 0)) {
      selected_points_.row_step = (i + 1) * selected_points_.point_step;
      selected_points_.data.resize(selected_points_.row_step);

      QModelIndex child_index = model->index(i, 0);

      auto child = model->getProp(child_index);
      auto subchild = (rviz_common::properties::VectorProperty *)child->childAt(0);
      Ogre::Vector3 point_data = subchild->getVector();

      uint8_t * data_pointer = &selected_points_.data[0] + i * selected_points_.point_step;
      *(float *)data_pointer = point_data.x;
      data_pointer += 4;
      *(float *)data_pointer = point_data.y;
      data_pointer += 4;
      *(float *)data_pointer = point_data.z;
      data_pointer += 4;

      // Search for the intensity value
      for (int j = 1; j < child->numChildren(); j++) {
        rviz_common::properties::Property * grandchild = child->childAt(j);
        QString nameOfChild = grandchild->getName();
        QString nameOfIntensity("intensity");

        if (nameOfChild.contains(nameOfIntensity)) {
          rviz_common::properties::FloatProperty * floatchild =
            (rviz_common::properties::FloatProperty *)grandchild;
          float intensity = floatchild->getValue().toFloat();
          *(float *)data_pointer = intensity;
          break;
        }
      }
      data_pointer += 4;
      i++;
    }
    num_selected_points_ = i;
    RCLCPP_INFO_STREAM(
      node_->get_logger(),
      "SelectedPointsPublisher._processSelectedAreaAndFindPoints");

    selected_points_.width = i;
    selected_points_.header.stamp = node_->now();
    return 0;
  }
}  // namespace rviz_plugin_selected_points_publisher

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  rviz_plugin_selected_points_publisher::SelectedPointsPublisher,
  rviz_common::Tool)
