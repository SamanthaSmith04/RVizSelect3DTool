#include "obj_region_selection_display.hpp"

ObjRegionSelectionDisplay::ObjRegionSelectionDisplay() : OGREMeshPlugin("obj_selection_display") {
    selected_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>("/selected_points_in_region", 10);
}

/**
 * @brief Callback for the region selection. 
 * 
 * Gets the points in a drawn polygon and the camera data
 * 
 */
void ObjRegionSelectionDisplay::regionCallback(const rviz_selection_3d::msg::SelectionRegion::SharedPtr msg) {
    RCLCPP_INFO(node_->get_logger(), "Received region data");
    // glm::vec3 cameraPosition = glm::vec3(msg->camera_pos.x, msg->camera_pos.y, msg->camera_pos.z);
    auto pos = msg->camera.position;
    glm::vec3 cameraPosition = glm::vec3(pos.x, pos.y, pos.z);
    
    glm::mat4 viewMatrix = glm::mat4(1.0f);
    glm::mat4 projectionMatrix = glm::mat4(1.0f);
    for (int i = 0; i < msg->camera.view_matrix_4x4.size(); i++) {
        viewMatrix[i/4][i%4] = msg->camera.view_matrix_4x4[i];
        projectionMatrix[i/4][i%4] = msg->camera.projection_matrix_4x4[i];
    }

    glm::mat4 viewProjectionMatrix = projectionMatrix * viewMatrix;

    std::vector<geometry_msgs::msg::Point> points_in_polygon = getPointsInPolygon(obj_data.positions, msg->points, viewMatrix, projectionMatrix, msg->viewport_width, msg->viewport_height);

    RCLCPP_INFO(node_->get_logger(), "Number of points in polygon: %d", points_in_polygon.size());

    RCLCPP_INFO(node_->get_logger(), "Number of vertices: %d:", obj_data.numVertices);
}

/**
 * @brief Projects a 3D point to 2D screen coordinates
 * 
 * @param point The 3D point to project
 *  
 * @param viewMatrix The camera view matrix
 * 
 * @param projMatrix The camera projection matrix
 * 
 * @param viewport_width The width of the viewport
 * 
 * @param viewport_height The height of the viewport
 * 
 * @return The 2D screen coordinates of the point
 */
glm::vec2 ObjRegionSelectionDisplay::projectedPoint(glm::vec3 point, glm::mat4 viewMatrix, glm::mat4 projMatrix, int viewport_width, int viewport_height) {

    // Transform the point using the view and projection matrices
    glm::vec4 transformedPos = projMatrix * (viewMatrix * glm::vec4(point.x, point.y, point.z, 1.0f));

    if (transformedPos.w == 0) {
        return glm::vec2(-1,-1);
    }

    transformedPos /= transformedPos.w;
    
    float screenX = (transformedPos.x + 1.0f) * (viewport_width / 2.0f);
    float screenY = (1.0f - transformedPos.y) * (viewport_height / 2.0f); // Invert y-axis for top-left origin

    return glm::vec2(screenX, screenY);
}


/**
 * @brief Checks if a point is inside a polygon
 * 
 * @param point The 2D point to check
 * 
 * @param polygon_points The 2D points of the polygon
 * 
 * @return True if the point is inside the polygon, false otherwise
 */
bool ObjRegionSelectionDisplay::isPointInPolygon(glm::vec2 point, std::vector<geometry_msgs::msg::Point> polygon_points) {
    int crossings = 0;
    for (int i = 0; i < polygon_points.size(); i++) {
        glm::vec2 p1 = glm::vec2(polygon_points[i].x, polygon_points[i].y);
        glm::vec2 p2 = glm::vec2(polygon_points[(i+1)%polygon_points.size()].x, polygon_points[(i+1)%polygon_points.size()].y);
        
        if (p1.y <= point.y && p2.y > point.y || p2.y <= point.y && p1.y > point.y) {
            float vt = (point.y - p1.y) / (p2.y - p1.y);
            if (point.x < p1.x + vt * (p2.x - p1.x)) {
                crossings++;
            }
        }
    }

    return (crossings % 2 == 1);
}

/**
 * @brief Gets the points in a drawn polygon
 * 
 * @param points The 3D points to check
 * 
 * @param polygon_points The 2D points of the polygon
 * 
 * @param viewMatrix The camera view matrix
 * 
 * @param projMatrix The camera projection matrix
 * 
 * @param viewport_width The width of the viewport
 * 
 * @param viewport_height The height of the viewport
 * 
 * @return The 3D points inside the polygon
 */
std::vector<geometry_msgs::msg::Point> ObjRegionSelectionDisplay::getPointsInPolygon(std::vector<Ogre::Vector3> points, 
                                                                                    std::vector<geometry_msgs::msg::Point> polygon_points, 
                                                                                    glm::mat4 viewMatrix, glm::mat4 projMatrix, 
                                                                                    int viewport_width, 
                                                                                    int viewport_height) 
{
    std::vector<geometry_msgs::msg::Point> points_in_polygon;
    for (auto& point: points) {
        glm::vec2 screenPoint = projectedPoint(glm::vec3(point.x, point.y, point.z), viewMatrix, projMatrix, viewport_width, viewport_height);
        if (screenPoint == glm::vec2(-1,-1)) {
            continue;
        }
        if (isPointInPolygon(screenPoint, polygon_points)) {
            auto point_msg = geometry_msgs::msg::Point();
            point_msg.x = point.x;
            point_msg.y = point.y;
            point_msg.z = point.z;
            points_in_polygon.push_back(point_msg);
        }
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map"; // Set the frame ID according to your needs
    pose_array.poses.resize(points_in_polygon.size());

    for (size_t i = 0; i < points_in_polygon.size(); i++) {
        pose_array.poses[i].position.x = points_in_polygon[i].x;
        pose_array.poses[i].position.y = points_in_polygon[i].y;
        pose_array.poses[i].position.z = points_in_polygon[i].z;
        pose_array.poses[i].orientation.w = 1.0; // Set the orientation as needed
    }

    selected_pub->publish(pose_array);
    
    return points_in_polygon;
}

bool ObjRegionSelectionDisplay::isPointVisibleToUser(glm::vec3 point, glm::mat4 viewMatrix, glm::mat4 projMatrix, int viewport_width, int viewport_height) {
    glm::vec2 screenPoint = projectedPoint(point, viewMatrix, projMatrix, viewport_width, viewport_height);
    if (screenPoint == glm::vec2(-1,-1)) {
        return false;
    }
    return true;
}

/**
 * @brief Initializes the display plugin
 * 
 */
void ObjRegionSelectionDisplay::onInitialize() {
    meshLoaded = true;

    region_sub = node_->create_subscription<rviz_selection_3d::msg::SelectionRegion>(
        "/select_3d_tool/region_points", 10, std::bind(&ObjRegionSelectionDisplay::regionCallback, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(), "Received region data");
    OGREMeshPlugin::onInitialize();
}

