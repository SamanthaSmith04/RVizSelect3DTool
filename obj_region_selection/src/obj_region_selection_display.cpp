#include "obj_region_selection_display.hpp"

ObjRegionSelectionDisplay::ObjRegionSelectionDisplay() : OGREMeshPlugin("obj_selection_display") {

}


void ObjRegionSelectionDisplay::regionCallback(const rviz_selection_3d::msg::SelectionRegion::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received region data");
}

void ObjRegionSelectionDisplay::onInitialize() {
    meshLoaded = true;

    region_sub = node_->create_subscription<rviz_selection_3d::msg::SelectionRegion>(
        "/select_3d_tool/region_points", 10, std::bind(&ObjRegionSelectionDisplay::regionCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Received region data");
    OGREMeshPlugin::onInitialize();
}