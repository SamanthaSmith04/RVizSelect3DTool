#ifndef OBJ_REGION_SELECTION_DISPLAY_H
#define OBJ_REGION_SELECTION_DISPLAY_H

#include <OgreTexture.h>
#include <OgreImage.h>
#include <OgreTextureManager.h>


#include "obj_region_selection/srv/send_mesh_texture.hpp"
#include "obj_region_selection/srv/send_modelpack_editor_info.hpp"
#include "obj_region_selection/msg/modelpack_selection_stored_state.hpp"

#include "ogre_mesh_plugin.h"

#include <arp_ux/msg/region.hpp>

#include <sstream>

/**
 * @brief Class for visually coloring the mesh based on region data
*/
class ObjRegionSelectionDisplay : public OGREMeshPlugin {
    public:
        explicit ObjRegionSelectionDisplay();

        /**
         * @brief Reads in an OBJ file and adds all points to the Deault region
         * 
         * @param path The path to the OBJ file
        */
        void loadOBJ(std::string path);

        /**
         * @brief Reads in a modelpack file and adds all points to the regions from the file
         * 
         * @param path The path to the modelpack file
        */
        void loadModelpack(std::string path);

    private:
        // Service to update region info and returns regions back
        rclcpp::Service<obj_region_selection::srv::SendModelpackEditorInfo>::SharedPtr send_selected_area_service;

        // Recieves an updated state from the modelpack state handler when undo/redo are pressed
        rclcpp::Subscription<obj_region_selection::msg::ModelpackSelectionStoredState>::SharedPtr modelpack_selection_stored_state_sub;

        // the names of all existing materials
        std::vector<std::string> material_names;
        
        // the thicknesses of all existing materials (max, min)
        std::vector<std::vector<double>> material_thicknesses; // max, min

        // The regions of the part
        std::vector<obj_region_selection::msg::RegionStoredState> modelpack_regions;

        // The current region being added/removed to/from
        std::string current_region_material_name;

        // The index of the current region being added/removed to/from
        int current_region_index;

        // Holds service calls until the mesh is done being processed
        bool still_processing = false;

        /**
         * @brief Updates the current region
         * 
         * @param index The index of the region to update to
        */
        void updateCurrentRegion(int index);

        /**
         * @brief Recieves the mesh info
         * 
         * @param request The request
         * @param response The response
        */
        void recieveMeshTexture(const std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Request> request, std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Response> response) override;

        /**
         * @brief Generates a new RGB color
         * 
         * @return The RGB color
        */
        std::vector<int> generateRGB();

        /**
         * @brief Renders the mesh with the proper coloring
         * 
         * @param obj_file_path The path to the OBJ file
        */
        void constructMesh(std::string obj_file_path) override;

        /**
         * @brief Fills the mesh with the proper region coloring and face borders
        */
        void fillMeshWithRegions();

        /**
         * @brief Takes in a set of points and performs either removal or addition of the points to the current region depending on the request
         *        and returns the updated regions
         * 
         * @param request The request
         * @param response The response
        */
        void sendSelectedArea(const std::shared_ptr<obj_region_selection::srv::SendModelpackEditorInfo::Request> request, std::shared_ptr<obj_region_selection::srv::SendModelpackEditorInfo::Response> response);
        
        /**
         * @brief Returns true if the vertex exists in a region
         * 
         * @param vertex The vertex to check
         * 
         * @param region The region to check
        */
        bool vertexInRegion(int vertex, obj_region_selection::msg::RegionStoredState region);

        /**
         * @brief Processes a set of vertex indicies and adds/removes them from a region
         * 
         * @param incoming_selection The set of vertex indicies
        */
        void processIncomingSelection(std::vector<int> incoming_selection);

        /**
         * @brief Creates a new region
        */
        void createNewRegion();

        /**
         * @brief Deletes a region
         * 
         * @param regionIndex The index of the region to delete
        */
        void deleteRegion(int regionIndex);

        /**
         * @brief Updates the region data
         * 
         * @param regionIndex The index of the region to update
         * @param region The region data
        */
        void updateStateWithStoredState(obj_region_selection::msg::ModelpackSelectionStoredState state);
};

PLUGINLIB_EXPORT_CLASS(::ObjRegionSelectionDisplay, rviz_common::Display)


#endif // OBJ_REGION_SELECTION_DISPLAY_H