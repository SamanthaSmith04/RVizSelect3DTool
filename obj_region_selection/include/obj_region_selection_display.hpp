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

        std::vector<geometry_msgs::msg::Point> getPointsInPolygon(std::vector<Ogre::Vector3> points, std::vector<geometry_msgs::msg::Point> polygon_points, glm::mat4 viewMatrix, glm::mat4 projMatrix, int viewport_width, int viewport_height);

        rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr selected_pub;


};

PLUGINLIB_EXPORT_CLASS(::ObjRegionSelectionDisplay, rviz_common::Display)


#endif // OBJ_REGION_SELECTION_DISPLAY_H