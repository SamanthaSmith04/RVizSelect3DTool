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
}



void SelectionToolPlugin::onInitialize() {
    setName("Select 3D Tool");
    selected_points = std::vector<Ogre::Vector3>();

}

void SelectionToolPlugin::activate() {
    

}

void SelectionToolPlugin::deactivate() {
}

int SelectionToolPlugin::processMouseEvent(rviz_common::ViewportMouseEvent& event) {
    Ogre::Vector3 pos;
    bool success = this->context_->getViewPicker()->get3DPoint(event.panel, event.x, event.y, pos);
    render_panel = event.panel;
    Ogre::Camera* camera = render_panel->getViewController()->getCamera();
    Ogre::Viewport* viewport = camera->getViewport();
    if ((event.leftDown() || event.rightDown()) && !currently_selecting && success){
        // Set selection mode
        if (event.leftDown()) {
            selection_mode = true;
        } else {
            selection_mode = false;
        }

        // if first point in selection
        if (success) {
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
        if (std::abs(pos.distance(current_first_point)) >= distance_threshold || line_grid_points.size() > 1) {    
            end = Ogre::Vector3();
            end.x = pos.x;
            end.y = pos.y;
            end.z = pos.z;

            selected_points.push_back(current_first_point);

            auto grid_pos = Ogre::Vector2();
            grid_pos.x = event.x;
            grid_pos.y = event.y;
            line_grid_points.push_back(grid_pos);

        }
    }
    else if (success && currently_selecting) {
        current_second_point = Ogre::Vector3();
        current_second_point.x = pos.x;
        current_second_point.y = pos.y;
        current_second_point.z = pos.z;

        if (std::abs(current_second_point.distance(current_first_point)) >= distance_threshold) {        
            auto line = std::make_shared<rviz_rendering::Line>(context_->getSceneManager());   
            line->setPoints(current_first_point, current_second_point);
            line->setColor(0.0, 1.0, 0.0, 1.0);
            
            selected_polygon_lines.push_back(line);
            current_first_point = current_second_point;

            auto grid_pos = Ogre::Vector2();
            grid_pos.x = event.x;
            grid_pos.y = event.y;
            line_grid_points.push_back(grid_pos);
        }

    }
}

void SelectionToolPlugin::addToMarkerArray(Ogre::Vector3 point1, Ogre::Vector3 point2) {
}

void SelectionToolPlugin::clearAllMarkers() {

}

void SelectionToolPlugin::publishSelectedAreaInfo() {
}
