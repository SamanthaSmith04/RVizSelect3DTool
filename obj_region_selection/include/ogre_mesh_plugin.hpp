/// Software License Agreement (Apache 2.0 License)
///
/// Copyright (c) 2022-2023, The Ohio State University
/// Center for Design and Manufacturing Excellence (CDME)
/// The Artificially Intelligent Manufacturing Systems Lab (AIMS)
/// All rights reserved.
///
/// Author: Samantha Smith
///

#ifndef OGRE_MESH_PLUGIN_H
#define OGRE_MESH_PLUGIN_H

#include <rviz_common/display.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>

#include <OgreEntity.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <OgreQuaternion.h>
#include <OgreVector3.h>
#include <OgreResourceGroupManager.h>
#include <OgreMeshManager.h>
#include <OgreMesh.h>
#include <OgreSubMesh.h>
#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreMaterial.h>
#include <OgreTechnique.h>
#include <OgrePass.h>

#include <ament_index_cpp/get_package_share_directory.hpp>

#include "obj_region_selection/srv/toggle_service.hpp"
#include "obj_region_selection/srv/send_mesh_texture.hpp"
#include "obj_region_selection/msg/send_mesh_info.hpp"

#include <rviz_common/display_context.hpp>

#include <pluginlib/class_list_macros.hpp>


/**
 * @brief Base class for visualizing a textured mesh in RViz
*/
class OGREMeshPlugin : public rviz_common::Display {
    public:
        /**
         * @brief Constructor for the display plugin
         * 
         * @param name Name of the display plugin
        */
        OGREMeshPlugin(std::string name);

        // the display context
        rviz_common::DisplayContext* context;

        /**
         * @brief Initializes the display plugin and grabs the display context
        */
        void initialize(rviz_common::DisplayContext * context) override;

        /**
         * @brief Called when the display is enabled, sets up the resource group for the part 
        */
        void onInitialize() override;

        /**
         * @brief Called on a timer to render the mesh, refreshes the display if the mesh is loaded
        */
        void update(float wall_dt, float ros_dt) override;

        /**
         * @brief Sets up the mesh to display in the scene
        */
        virtual void reload();

        // Stores if the mesh has been altered
        bool new_mesh = false;

        // Stores if the mesh has been loaded in
        bool meshLoaded = false;

        // Stores if the part has been enabled visually
        bool enabled = true;

        // Path to the part obj file
        std::string mesh_path;

        // Path to the part modelpack file
        std::string modelpack_path;

        // The name of the node
        std::string node_name;

    protected:
        // The ROS node for general use
        rclcpp::Node::SharedPtr node_;

        // The ROS node to run services on
        rclcpp::Node::SharedPtr service_nodes;

        // Pointer to the RViz scene
        Ogre::SceneManager* scene_manager;

        // The Ogre Entity for the part
        Ogre::Entity* part_entity;

        // The Ogre object for the part
        Ogre::ManualObject* mesh;

        // The Ogre node for the scene
        Ogre::SceneNode* scene_node;

        // Holds the color resources
        Ogre::ResourceGroupManager* resource_group_manager;

        // The material for the part
        Ogre::MaterialPtr material;

        // The material manager for the part
        Ogre::MaterialManager* material_manager;

        // The topic of the node
        std::string node_topic;

        // The service to enable/disable the marker
        rclcpp::Service<obj_region_selection::srv::ToggleService>::SharedPtr enable_disable_marker_service;

        // The service to send the mesh texture
        rclcpp::Service<obj_region_selection::srv::SendMeshTexture>::SharedPtr send_mesh_texture_service;

        // The publisher to send the mesh info
        rclcpp::Publisher<obj_region_selection::msg::SendMeshInfo>::SharedPtr send_mesh_info_publisher;

        /**
         * @brief Constructs the mesh from the obj file
         * 
         * @param obj_file_path The path to the obj file
        */
        virtual void constructMesh(std::string obj_file_path);

        /**
         * @brief Runs the ROS nodes
        */
        void runRosNodes();

        /**
         * @brief Callback for the toggle service, enables/disables the marker
         * 
         * @param request The request sent to the service
         * 
         * @param response The response sent back to the service
        */
        void toggleMarker(const std::shared_ptr<obj_region_selection::srv::ToggleService::Request> request, std::shared_ptr<obj_region_selection::srv::ToggleService::Response> response);
        
        /**
         * @brief Recieves the mesh info
         * 
         * @param request The request
         * 
         * @param response The response
        */
        virtual void recieveMeshTexture(const std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Request> request, std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Response> response);

        /**
         * @brief Reads in the OBJ file
         * 
         * @param obj_file_path The path to the OBJ file
        */
        void readOBJ(std::string obj_file_path);

        /**
         * @brief Fills the mesh with the data from the OBJ file
        */
        virtual void fillMesh();

        /**
         * @brief Stores the OBJ Data
         * 
         * @param positions the vector of vertex positions
         * @param textureCoords the vector of texture coordinates
         * @param normals the vector of normals
         * @param facePositions the vector of face positions
         * @param faceTextureCoords the vector of face texture coordinates
         * @param faceNormals the vector of face normals
         * @param numVertices the number of Vertices
         * @param numTextureCoords the number of texture coordinates
         * @param numNormals the number of normals
         * @param numFaces the number of faces
        */
        struct OBJData {
            std::vector<Ogre::Vector3> positions;             // v
            std::vector<Ogre::Vector2> textureCoords;         // vt
            std::vector<Ogre::Vector3> normals;               // vn
            std::vector<Ogre::Vector3> facePositions;         // f (*/#/#)
            std::vector<Ogre::Vector3> faceTextureCoords;     // f (#/*/#)
            std::vector<Ogre::Vector3> faceNormals;           // f (#/#/*)
            int numVertices = 0;
            int numTextureCoords = 0;
            int numNormals = 0;
            int numFaces = 0;
        };

        // The OBJ data
        OBJData obj_data;
        
};  

#endif // OGRE_MESH_PLUGIN_H