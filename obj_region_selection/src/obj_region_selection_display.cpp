#include "obj_region_selection_display.hpp"

ObjRegionSelectionDisplay::ObjRegionSelectionDisplay() : OGREMeshPlugin("obj_selection_display") {
    selected_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>("/selected_points_in_region", 10);
    normals_pub = node_->create_publisher<geometry_msgs::msg::PoseArray>("/normals", 10);
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

    std::vector<geometry_msgs::msg::Point> points_in_polygon = getPointsInPolygon(obj_data.positions, obj_data.normals, obj_data.facePositions, obj_data.faceNormals, msg->points, viewMatrix, projectionMatrix, cameraPosition, msg->viewport_width, msg->viewport_height);

    RCLCPP_INFO(node_->get_logger(), "Number of points in polygon: %d", points_in_polygon.size());

    RCLCPP_INFO(node_->get_logger(), "Number of vertices: %d:", obj_data.numVertices);

    auto normals_msg = geometry_msgs::msg::PoseArray();
    normals_msg.header.frame_id = "map"; 
    normals_msg.poses.resize(obj_data.numVertices);
    for (size_t i = 0; i < obj_data.numVertices; i++) {
        normals_msg.poses[i].position.x = obj_data.positions[i].x;
        normals_msg.poses[i].position.y = obj_data.positions[i].y;
        normals_msg.poses[i].position.z = obj_data.positions[i].z;
        normals_msg.poses[i].orientation.x = obj_data.normals[i].x;
        normals_msg.poses[i].orientation.y = obj_data.normals[i].y;
        normals_msg.poses[i].orientation.z = obj_data.normals[i].z;
        normals_msg.poses[i].orientation.w = 1.0; 
    }

    normals_pub->publish(normals_msg);
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
 * @brief Gets the points that are visible and selected by a drawn polygon
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
                                                                                    std::vector<Ogre::Vector3> normals,   
                                                                                    std::vector<Ogre::Vector3> face_positions,
                                                                                    std::vector<Ogre::Vector3> face_normals,  
                                                                                    std::vector<geometry_msgs::msg::Point> polygon_points, 
                                                                                    glm::mat4 viewMatrix, glm::mat4 projMatrix, 
                                                                                    glm::vec3 cameraPosition,
                                                                                    int viewport_width, 
                                                                                    int viewport_height) 
{
    std::vector<geometry_msgs::msg::Point> points_in_polygon;
    int count = 0;
    // check over every point in the obj
    for (auto& point: points) {
        glm::vec2 screenPoint = projectedPoint(glm::vec3(point.x, point.y, point.z), viewMatrix, projMatrix, viewport_width, viewport_height);
        // if point is not on the screen, skip
        if (screenPoint == glm::vec2(-1,-1)) {
            continue;
        }
        // check if the normal vector is pointing towards the camera (comes from the obj)
        // This only works if there is an equal number of normals and points in the obj
        // if there isnt, this confition can be commented out, it just will run less efficiently
        if (isPointNormalTowardCamera(glm::vec3(normals[count].x, normals[count].y, normals[count].z), 
                                glm::vec3(point.x, point.y, point.z), 
                                cameraPosition) 
                                == true) {
            // check if the 2d position on-screen of the 3d point is within the 2d drawn polygon on the screen
            if (isPointInPolygon(screenPoint, polygon_points)) {
                glm::vec3 p = glm::vec3(point.x, point.y, point.z);
                // check if the current point is visible from the camera (not being blocked by any face)
                if (isFaceVisible(p, face_positions, face_normals, points, normals, cameraPosition)) {
                    auto point_msg = geometry_msgs::msg::Point();
                    point_msg.x = point.x;
                    point_msg.y = point.y;
                    point_msg.z = point.z;

                    // Store the visible, valid point
                    points_in_polygon.push_back(point_msg);
                }
                
            }
        }
        count++;
    }

    geometry_msgs::msg::PoseArray pose_array;
    pose_array.header.frame_id = "map"; 
    pose_array.poses.resize(points_in_polygon.size());

    for (size_t i = 0; i < points_in_polygon.size(); i++) {
        pose_array.poses[i].position.x = points_in_polygon[i].x;
        pose_array.poses[i].position.y = points_in_polygon[i].y;
        pose_array.poses[i].position.z = points_in_polygon[i].z;
        pose_array.poses[i].orientation.w = 1.0; 
    }

    selected_pub->publish(pose_array);
    
    return points_in_polygon;
}

/**
 * @brief Checks if a face is visible from the camera (not blocked by any other face)
 * 
 * @param check_point The point to check
 * 
 * @param face_positions The positions indices of the faces
 * 
 * @param face_normals The normals indices of the faces
 * 
 * @param points The 3D points of the obj
 * 
 * @param normals The normals of the obj
 * 
 * @param cameraPosition The position of the camera
 * 
 * @return True if the face is visible, false otherwise
 * 
 */
bool ObjRegionSelectionDisplay::isFaceVisible(glm::vec3 check_point, std::vector<Ogre::Vector3> face_positions, std::vector<Ogre::Vector3> face_normals, std::vector<Ogre::Vector3> points, std::vector<Ogre::Vector3> normals, glm::vec3 cameraPosition) {
    float t;
    bool isVisible = true;
    float closest_t = 1000000;
    float distanceToCamera = glm::distance(check_point, cameraPosition);
    
    // loop over all faces in the obj
    // finds the closes intersection point of the point and any face from the camera
    for (int i = 0; i < face_positions.size(); i++) {
        Ogre::Vector3 v0 = points[face_positions[i][0]];
        Ogre::Vector3 v1 = points[face_positions[i][1]];
        Ogre::Vector3 v2 = points[face_positions[i][2]];

        // check if a ray cast from the camera to the point intersects with the face
        if (rayIntersectsTriangle(glm::vec3(cameraPosition.x, cameraPosition.y, cameraPosition.z), 
                                  glm::normalize(check_point - cameraPosition), 
                                  v0, v1, v2, t)) {
            // store the closest intersection point
            if (t < closest_t) {
                closest_t = t;
            }
        }
    }

    // if the closest intersection point is not the same as the distance to the camera (within a small threshold), the point is not visible
    if (std::abs(closest_t - distanceToCamera) > 0.01) {
        isVisible = false;
    }

    return isVisible; 
}


/**
 * @brief Checks if a ray intersects a triangle
 * 
 * @param rayOrigin The origin of the ray
 * 
 * @param rayDirection The direction of the ray
 * 
 * @param v0 The first vertex of the triangle
 * 
 * @param v1 The second vertex of the triangle
 * 
 * @param v2 The third vertex of the triangle
 * 
 * @param t The distance to the intersection point (pointer)
 * 
 * @return True if the ray intersects the triangle, false otherwise
 */
bool ObjRegionSelectionDisplay::rayIntersectsTriangle(const glm::vec3& rayOrigin, 
                           const glm::vec3& rayDirection, 
                           const Ogre::Vector3& v0, 
                           const Ogre::Vector3& v1, 
                           const Ogre::Vector3& v2, 
                           float& t) {

    // get the edges of the triangle
    glm::vec3 edge1 = glm::vec3(v1.x, v1.y, v1.z) - glm::vec3(v0.x, v0.y, v0.z);
    glm::vec3 edge2 = glm::vec3(v2.x, v2.y, v2.z) - glm::vec3(v0.x, v0.y, v0.z);

    // check if the ray is parallel to the triangle
    glm::vec3 h = glm::cross(rayDirection, edge2);
    float a = glm::dot(edge1, h);

    if (a > -1e-5 && a < 1e-5) return false; // Parallel

    float f = 1.0 / a;
    glm::vec3 s = rayOrigin - glm::vec3(v0.x, v0.y, v0.z);
    float u = f * glm::dot(s, h);
    if (u < 0.0 || u > 1.0) return false;

    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(rayDirection, q);
    if (v < 0.0 || u + v > 1.0) return false;

    // calculate the distance to the intersection point
    t = f * glm::dot(edge2, q);
    return t > 1e-5; // Intersection occurs
}

/**
 * @brief Checks if a point normal is towards the camera
 * 
 * @param pointNormal The normal of the point
 * 
 * @param pointPosition The position of the point
 * 
 * @param cameraPosition The position of the camera
 * 
 * @return True if the normal is towards the camera, false otherwise
 * 
 */
bool ObjRegionSelectionDisplay::isPointNormalTowardCamera(glm::vec3 pointNormal, glm::vec3 pointPosition, glm::vec3 cameraPosition) {

    bool isVisible = false;

    // get vector from camera to point
    glm::vec3 cameraToPoint = pointPosition - cameraPosition;

    // get angle between vector and normal
    float angle = glm::dot(glm::normalize(cameraToPoint), glm::normalize(pointNormal));

    // if angle is less than 90 degrees and greater than 0 degrees, point is visible
    const float zero_threshold = 0.0001;
    if (angle >= -zero_threshold) {
        isVisible = true;
    }

    return isVisible;
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

