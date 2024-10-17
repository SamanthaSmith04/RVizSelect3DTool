/// Software License Agreement (Apache 2.0 License)
///
/// Copyright (c) 2022-2023, The Ohio State University
/// Center for Design and Manufacturing Excellence (CDME)
/// The Artificially Intelligent Manufacturing Systems Lab (AIMS)
/// All rights reserved.
///
/// Author: Samantha Smith
///

#include "select_3d_tool.hpp"

SelectionToolPlugin::SelectionToolPlugin() : Tool()
, node(std::make_shared<rclcpp::Node>("select_3d_tool"))
{
    selected_area_pub = node->create_publisher<rviz_selection_3d::msg::SelectionRegion>("/select_3d_tool/region_points", 10);
}



void SelectionToolPlugin::onInitialize() {
    setName("Select 3D Tool");
    selected_points = std::vector<Ogre::Vector3>();

}

void SelectionToolPlugin::activate() {
    clearAllMarkers();
    currently_selecting = false;

}

void SelectionToolPlugin::deactivate() {
        currently_selecting = false;
    
}

int SelectionToolPlugin::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    Ogre::Vector3 pos;
    bool success = this->context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
    render_panel = event.panel;
    Ogre::Camera* camera = render_panel->getViewController()->getCamera();
    Ogre::Viewport* viewport = camera->getViewport();

    bool in_viewport = (event.x >= 0 && event.x <= viewport->getActualWidth() && 
                        event.y >= 0 && event.y <= viewport->getActualHeight());

    // RCLCPP_INFO(node->get_logger(), "Mouse Position: %d, %d", event.x, event.y);

    if ((event.leftDown() || event.rightDown()) && !currently_selecting && success && in_viewport){
        // Set selection mode
        if (event.leftDown()) {
            selection_mode = true;
        } else {
            selection_mode = false;
        }

        RCLCPP_INFO(node->get_logger(), "Mouse down");

        // if first point in selection
        if (success) {
            clearAllMarkers();
            selected_points.clear();
            selected_polygon_lines.clear();

            currently_selecting = true;
            begin = Ogre::Vector3();
            begin.x = pos.x;
            begin.y = pos.y;
            begin.z = pos.z;

            selected_points.push_back(begin);   

            current_first_point = Ogre::Vector3();
            current_first_point.x = pos.x;
            current_first_point.y = pos.y;
            current_first_point.z = pos.z;

            auto grid_pos = Ogre::Vector2();
            grid_pos.x = event.x;
            grid_pos.y = event.y;
            line_grid_points.push_back(grid_pos);
        } 
    } 
    // mouse has been un-clicked, stop tracking movement
    else if (((event.leftUp() && selection_mode) || (event.rightUp() && !selection_mode)) && success) {
        currently_selecting = false;
        if (line_grid_points.size() > 1) {    
            end = Ogre::Vector3();
            end.x = pos.x;
            end.y = pos.y;
            end.z = pos.z;

            selected_points.push_back(current_first_point);
            selected_points.push_back(end);

            auto grid_pos = Ogre::Vector2();
            grid_pos.x = event.x;
            grid_pos.y = event.y;
            line_grid_points.push_back(grid_pos);

            auto line = std::make_shared<rviz_rendering::Line>(context_->getSceneManager());   

            line->setPoints(end, current_first_point);
            if (selection_mode) {
                line->setColor(0.0, 1.0, 0.0, 1.0);
            } else {
                line->setColor(1.0, 0.0, 0.0, 1.0);
            }
            selected_polygon_lines.push_back(line);

            line->setPoints(end, begin);
            if (selection_mode) {
                line->setColor(0.0, 1.0, 0.0, 1.0);
            } else {
                line->setColor(1.0, 0.0, 0.0, 1.0);
            }
            selected_polygon_lines.push_back(line);
        }
        Ogre::Camera* camera = render_panel->getViewController()->getCamera();
        publishSelectedAreaInfo(camera);
    }
    // mouse is moving
    else if (success && currently_selecting) {
        current_second_point = Ogre::Vector3();
        current_second_point.x = pos.x;
        current_second_point.y = pos.y;
        current_second_point.z = pos.z;

        if (std::abs(current_second_point.distance(current_first_point)) >= distance_threshold) {        
            auto line = std::make_shared<rviz_rendering::Line>(context_->getSceneManager());   
            line->setPoints(current_first_point, current_second_point);
            if (selection_mode) {
                line->setColor(0.0, 1.0, 0.0, 1.0);
            } else {
                line->setColor(1.0, 0.0, 0.0, 1.0);
            }
            
            selected_polygon_lines.push_back(line);
            selected_points.push_back(current_first_point);
            current_first_point = current_second_point;

            auto grid_pos = Ogre::Vector2();
            grid_pos.x = event.x;
            grid_pos.y = event.y;
            line_grid_points.push_back(grid_pos);
        }

    }
}


void SelectionToolPlugin::clearAllMarkers() {

    for (auto& line: selected_polygon_lines) {
        line->setVisible(false);
    }

    selected_polygon_lines.clear();
    selected_points.clear();
    line_grid_points.clear();

}

void SelectionToolPlugin::publishSelectedAreaInfo(Ogre::Camera* camera) {
    rviz_selection_3d::msg::SelectionRegion msg;
    
    std::vector<geometry_msgs::msg::Point> selected_points_msg;
    for (auto& point: line_grid_points) {
        geometry_msgs::msg::Point point_msg;
        point_msg.x = point.x;
        point_msg.y = point.y;
        point_msg.z = 0.0;
        // RCLCPP_INFO(node->get_logger(), "Point: %f, %f", point.x, point.y);
        selected_points_msg.push_back(point_msg);
    }

    msg.points = selected_points_msg;

    msg.is_selecting = selection_mode;

    geometry_msgs::msg::Point camera_pos;
    auto camera_pos_ogre = camera->getPositionForViewUpdate();
    camera_pos.x = camera_pos_ogre.x;
    camera_pos.y = camera_pos_ogre.y;
    camera_pos.z = camera_pos_ogre.z;
    msg.camera.position = camera_pos;

    auto camera_proj = camera->getProjectionMatrix().transpose();
    auto camera_view = camera->getViewMatrix().transpose();
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            msg.camera.projection_matrix_4x4.push_back(camera_proj[i][j]);
            msg.camera.view_matrix_4x4.push_back(camera_view[i][j]);
        }
    }
    Ogre::Viewport* viewport = camera->getViewport();
    msg.viewport_width = viewport->getActualWidth();
    msg.viewport_height = viewport->getActualHeight();

    RCLCPP_INFO(node->get_logger(), "View width: %d, View height: %d", msg.viewport_width, msg.viewport_height);

    selected_area_pub->publish(msg);
}
