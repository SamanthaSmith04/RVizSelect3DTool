#ifndef OBJ_REGION_SELECTION_DISPLAY_H
#define OBJ_REGION_SELECTION_DISPLAY_H

#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreTextureManager.h>

#include "ogre_mesh_plugin.hpp"

#include <sstream>
#include <rviz_selection_3d/msg/selection_region.hpp>

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
};

PLUGINLIB_EXPORT_CLASS(::ObjRegionSelectionDisplay, rviz_common::Display)


#endif // OBJ_REGION_SELECTION_DISPLAY_H