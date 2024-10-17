#ifndef OBJ_REGION_SELECTION_DISPLAY_H
#define OBJ_REGION_SELECTION_DISPLAY_H

#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreTextureManager.h>

#include "ogre_mesh_plugin.hpp"

#include <sstream>
#include <rviz_selection_3d/msg/selection_region.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <OgreVector3.h>

#include <geometry_msgs/msg/pose_array.hpp>

/**
 * @brief Class for visually coloring the mesh based on region data
*/
class ObjRegionSelectionDisplay : public OGREMeshPlugin {
    public:
        explicit ObjRegionSelectionDisplay();

    private:
       rclcpp::Subscription<rviz_selection_3d::msg::SelectionRegion>::SharedPtr region_sub;

        /**
         * @brief Callback for the region selection topic
         * 
         * @param msg The message containing the region data
         */
        void regionCallback(const rviz_selection_3d::msg::SelectionRegion::SharedPtr msg);

        /**
         * @brief Initialize the display
         */
        void onInitialize() override;

        glm::vec2 projectedPoint(glm::vec3 point, glm::mat4 viewMatrix, glm::mat4 projMatrix, int viewport_width, int viewport_height);

        bool isPointInPolygon(glm::vec2 point, std::vector<geometry_msgs::msg::Point> polygon_points);

        bool isPointNormalTowardCamera(glm::vec3 pointNormal, glm::vec3 pointPosition, glm::vec3 cameraPosition);

        std::vector<geometry_msgs::msg::Point> getPointsInPolygon(std::vector<Ogre::Vector3> points, std::vector<Ogre::Vector3> normals, std::vector<Ogre::Vector3> face_positions, std::vector<Ogre::Vector3> face_normals, std::vector<geometry_msgs::msg::Point> polygon_points, glm::mat4 viewMatrix, glm::mat4 projMatrix, glm::vec3 cameraPosition, int viewport_width, int viewport_height);

        bool rayIntersectsTriangle(const glm::vec3& rayOrigin, const glm::vec3& rayDirection, const Ogre::Vector3& v0, const Ogre::Vector3& v1, const Ogre::Vector3& v2, float& t);

        bool isFaceVisible(glm::vec3 point, std::vector<Ogre::Vector3> face_positions, std::vector<Ogre::Vector3> face_normals, std::vector<Ogre::Vector3> points, std::vector<Ogre::Vector3> normals, glm::vec3 cameraPosition);
        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr selected_pub;

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr normals_pub;


};

PLUGINLIB_EXPORT_CLASS(::ObjRegionSelectionDisplay, rviz_common::Display)


#endif // OBJ_REGION_SELECTION_DISPLAY_H