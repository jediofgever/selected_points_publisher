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
 * selected_points_publisher.hpp
 *
 *  Created on: December 5th 2019
 */

#ifndef SELECTED_POINTS_PUBLISHER_HPP
#define SELECTED_POINTS_PUBLISHER_HPP

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
#include <QCursor>
#include <QObject>
#endif

#include <rclcpp/rclcpp.hpp>
#include "rviz_common/tool.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include "rviz_default_plugins/tools/select/selection_tool.hpp"
#include "rviz_common/viewport_mouse_event.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/interaction/forwards.hpp"
#include "rviz_common/interaction/selection_manager.hpp"
#include "rviz_common/properties/property_tree_model.hpp"
#include "rviz_common/properties/property.hpp"
#include "rviz_common/properties/color_property.hpp"
#include "rviz_common/properties/vector_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/view_manager.hpp"
#include "rviz_common/view_controller.hpp"
#include <QVariant>

namespace rviz_plugin_selected_points_publisher
{
class SelectedPointsPublisher;

class SelectedPointsPublisher : public rviz_default_plugins::tools::SelectionTool
{
  Q_OBJECT
public:
  SelectedPointsPublisher();
  virtual ~SelectedPointsPublisher();
  virtual int processMouseEvent(rviz_common::ViewportMouseEvent& event);
  virtual int processKeyEvent(QKeyEvent* event, rviz_common::RenderPanel* panel);

public Q_SLOTS:
 
protected:
  int processSelectedArea();

  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr rviz_selected_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_subscriber_;

  std::string tf_frame_;
  std::string rviz_cloud_topic_;
  std::string subscribed_cloud_topic_;

  sensor_msgs::msg::PointCloud2 selected_points_;

  bool selecting_;
  int num_selected_points_;
};
}  // namespace rviz_plugin_selected_points_publisher

#endif  // SELECTED_POINTS_PUBLISHER_HPP
