/// Software License Agreement (Apache 2.0 License)
///
/// Copyright (c) 2022-2023, The Ohio State University
/// Center for Design and Manufacturing Excellence (CDME)
/// The Artificially Intelligent Manufacturing Systems Lab (AIMS)
/// All rights reserved.
///
/// Author: Samantha Smith
///

#ifndef SELECT_3D_TOOL_HPP
#define SELECT_3D_TOOL_HPP

#include <rclcpp/rclcpp.hpp>

#include <rviz_common/tool.hpp>
#include <rclcpp/node.hpp>
#include <rviz_common/viewport_mouse_event.hpp>
#include <rviz_rendering/objects/line.hpp>

#include <rviz_common/display_context.hpp>
#include <rviz_common/render_panel.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/view_controller.hpp>
#include <rviz_common/tool_manager.hpp> 
#include <rviz_common/interaction/view_picker_iface.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <pluginlib/class_list_macros.hpp>

#include <OgreVector3.h>
#include <Ogre.h>

#include "rviz_selection_3d/msg/selection_region.hpp"
class SelectionToolPlugin : public rviz_common::Tool
{
public:
    SelectionToolPlugin();
    void onInitialize() override;
    void activate() override;
    void deactivate() override;
    int processMouseEvent(rviz_common::ViewportMouseEvent& event) override;
    void addToMarkerArray(Ogre::Vector3 point1, Ogre::Vector3 point2);
    void clearAllMarkers();
    void publishSelectedAreaInfo(Ogre::Camera* camera);

private:
    std::shared_ptr<rclcpp::Node> node;
    rclcpp::Publisher<rviz_selection_3d::msg::SelectionRegion>::SharedPtr selected_area_pub;
    std::vector<Ogre::Vector3> selected_points;
    std::vector<Ogre::Vector2> line_grid_points;
    std::vector<std::shared_ptr<rviz_rendering::Line>> selected_polygon_lines;
    bool currently_selecting = false;
    bool selection_complete = false;
    bool selection_mode = false;

    Ogre::Vector3 begin;
    Ogre::Vector3 current_first_point;
    Ogre::Vector3 current_second_point;
    Ogre::Vector3 end;
    rviz_common::RenderPanel* render_panel;
    float distance_threshold = 0.005;

};

PLUGINLIB_EXPORT_CLASS(::SelectionToolPlugin, rviz_common::Tool)

#endif // SELECT_3D_TOOL_HPP