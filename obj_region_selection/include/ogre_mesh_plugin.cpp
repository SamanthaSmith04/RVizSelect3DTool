/// Software License Agreement (Apache 2.0 License)
///
/// Copyright (c) 2022-2023, The Ohio State University
/// Center for Design and Manufacturing Excellence (CDME)
/// The Artificially Intelligent Manufacturing Systems Lab (AIMS)
/// All rights reserved.
///
/// Author: Samantha Smith
///

#include "ogre_mesh_plugin.hpp"

#define OBJ_NAME "liner_duct.obj"

OGREMeshPlugin::OGREMeshPlugin(std::string name) : rviz_common::Display()
, node_(rclcpp::Node::make_shared(name))
{
    node_name = name;
    service_nodes = rclcpp::Node::make_shared(node_name + "_service_nodes");
    this->setName(QString::fromStdString(node_name + "_marker"));

    send_mesh_info_publisher = node_->create_publisher<obj_region_selection::msg::SendMeshInfo>(node_name + "_ux_send_mesh_info", 1);

    //Services
    enable_disable_marker_service = service_nodes->create_service<obj_region_selection::srv::ToggleService>(node_name + "_ux_enable_disable_marker", 
        std::bind(&OGREMeshPlugin::toggleMarker, this, std::placeholders::_1, std::placeholders::_2));

    send_mesh_texture_service = service_nodes->create_service<obj_region_selection::srv::SendMeshTexture>(node_name + "_ux_send_mesh_texture", 
        std::bind(&OGREMeshPlugin::recieveMeshTexture, this, std::placeholders::_1, std::placeholders::_2));
}

void OGREMeshPlugin::initialize(rviz_common::DisplayContext * context) {
    this->context = context;
    rviz_common::Display::initialize(context);
    
}

void OGREMeshPlugin::onInitialize() {
    this->setEnabled(enabled);
    scene_manager = context->getSceneManager();
    material_manager = Ogre::MaterialManager::getSingletonPtr();
    resource_group_manager = Ogre::ResourceGroupManager::getSingletonPtr();
    if (!resource_group_manager->resourceGroupExists(node_name + "_resource_group")) {
        resource_group_manager->createResourceGroup(node_name + "_resource_group");
        resource_group_manager->addResourceLocation(ament_index_cpp::get_package_share_directory("obj_region_selection") + "/test_parts", "FileSystem", node_name + "_resource_group");
        resource_group_manager->initialiseResourceGroup(node_name + "_resource_group");
        resource_group_manager->loadResourceGroup(node_name + "_resource_group");
    }
    
    scene_node = scene_manager->getRootSceneNode()->createChildSceneNode();
    mesh_path = ament_index_cpp::get_package_share_directory("obj_region_selection") + "/test_parts/" + OBJ_NAME; 

    reload();
}

void OGREMeshPlugin::update(float wall_dt, float ros_dt) {
    (void) wall_dt;
    (void) ros_dt;  
    if (enabled && meshLoaded) {
        if (new_mesh) {
            new_mesh = false;
            reload();
        }
    }

    rclcpp::spin_some(node_);
    rclcpp::spin_some(service_nodes);
}

void OGREMeshPlugin::constructMesh(std::string obj_file_path) {
    if (!meshLoaded) {
        return;
    }
    
    if (scene_manager->hasManualObject(node_name + "generated_mesh")) {
        scene_manager->destroyManualObject(node_name + "generated_mesh");
    }

    obj_data = OBJData();

    readOBJ(obj_file_path);

    // CREATE MESH
    mesh = scene_manager->createManualObject(node_name + "generated_mesh");

    fillMesh();
    
}

void OGREMeshPlugin::reload() {
    if (meshLoaded) {
        RCLCPP_INFO(node_->get_logger(), "Reloading mesh");
        constructMesh(mesh_path); 
        scene_node->attachObject(mesh);
        scene_node->setPosition(Ogre::Vector3(0,0,0));
        scene_node->setScale(Ogre::Vector3(1,1,1));
        scene_node->setOrientation(Ogre::Quaternion(1,0,0,0));
    }
}

void OGREMeshPlugin::runRosNodes() {
    // rclcpp::executors::MultiThreadedExecutor executor;
    // executor.add_node(service_nodes);
    // executor.spin();
}

void OGREMeshPlugin::toggleMarker(const std::shared_ptr<obj_region_selection::srv::ToggleService::Request> request, std::shared_ptr<obj_region_selection::srv::ToggleService::Response> response) {
    enabled = request->enabled;
    this->setEnabled(enabled);
    scene_node->setVisible(enabled);

    response->success = true;
}

void OGREMeshPlugin::readOBJ(std::string obj_file_path) {

    std::ifstream objFile(obj_file_path);

    if (!objFile.is_open()) {
        std::cout << "ERROR: Could not open file " << obj_file_path << std::endl;
        return;
    }

    std::string line;
    Ogre::Vector3 tempPositionVector;
    Ogre::Vector2 tempTextureCoordVector;
    Ogre::Vector3 tempNormalVector;
    Ogre::Vector3 tempFacePositionVector;
    Ogre::Vector3 tempFaceTextureCoordVector;
    Ogre::Vector3 tempFaceNormalVector;

    // GET DATA FROM OBJ FILE
    while(std::getline(objFile, line)) {
        if (line.substr(0,2) == "vt") {
            double x, y;
            std::istringstream iss(line.substr(3));
            iss >> x >> y;
            tempTextureCoordVector = Ogre::Vector2(x, y);
            obj_data.textureCoords.push_back(tempTextureCoordVector);
            obj_data.numTextureCoords++;
        }
        else if (line.substr(0,2) == "vn") {
            double x, y, z;
            std::istringstream iss(line.substr(3));
            iss >> x >> y >> z;
            tempNormalVector = Ogre::Vector3(x, y, z);
            obj_data.normals.push_back(tempNormalVector);
            obj_data.numNormals++;
        }
        else if (line.substr(0,2) == "v ") { // Vertex data
            double x, y, z;
            std::istringstream iss(line.substr(2));
            iss >> x >> y >> z;
            tempPositionVector = Ogre::Vector3(x, y, z);
            obj_data.positions.push_back(tempPositionVector);
            obj_data.numVertices++;
        }
        else if (line.substr(0,2) == "f ") {
            int v1, v2, v3, t1, t2, t3, n1, n2, n3;
            char slash;
            std::istringstream iss(line.substr(2));
            iss >> v1 >> slash;
            if (iss.peek() != '/') {
                iss >> t1 >> slash;
            } else {
                t1 = 0;
                iss.ignore();
            }
            iss >> n1;

            iss >> v2 >> slash;
            if (iss.peek() != '/') {
                iss >> t2 >> slash;
            } else {
                t2 = 0;
                iss.ignore();
            }
            iss >> n2;


            iss >> v3 >> slash;
            if (iss.peek() != '/') {
                iss >> t3 >> slash;
            } else {
                t3 = 0;
                iss.ignore();
            }
            iss >> n3;


            tempFacePositionVector = Ogre::Vector3(v1-1, v2-1, v3-1);
            tempFaceTextureCoordVector = Ogre::Vector3(t1-1, t2-1, t3-1);
            tempFaceNormalVector = Ogre::Vector3(n1-1, n2-1, n3-1);
            obj_data.facePositions.push_back(tempFacePositionVector);
            obj_data.faceTextureCoords.push_back(tempFaceTextureCoordVector);
            obj_data.faceNormals.push_back(tempFaceNormalVector);
            obj_data.numFaces++;
        }
    }
    objFile.close();

    auto mesh_info_msg = obj_region_selection::msg::SendMeshInfo();
    auto temp = geometry_msgs::msg::Vector3();
    for (int i = 0; i < obj_data.faceNormals.size(); i++) {
        temp.x = obj_data.faceNormals[i][0];
        temp.y = obj_data.faceNormals[i][1];
        temp.z = obj_data.faceNormals[i][2];
        mesh_info_msg.normal_indices.push_back(temp);
    }

    temp = geometry_msgs::msg::Vector3();
    for (int i = 0; i < obj_data.normals.size(); i++) {
        temp.x = obj_data.normals[i][0];
        temp.y = obj_data.normals[i][1];
        temp.z = obj_data.normals[i][2];
        mesh_info_msg.normals.push_back(temp);
    }

    temp = geometry_msgs::msg::Vector3();
    for (int i = 0; i < obj_data.facePositions.size(); i++) {
        temp.x = obj_data.facePositions[i][0];
        temp.y = obj_data.facePositions[i][1];
        temp.z = obj_data.facePositions[i][2];
        mesh_info_msg.face_position_indices.push_back(temp);
    }

    send_mesh_info_publisher->publish(mesh_info_msg);

}

void OGREMeshPlugin::fillMesh() {
    mesh->begin("PartMeshMaterial", Ogre::RenderOperation::OT_TRIANGLE_LIST, node_name + "_resource_group");
    for (int i = 0; i < obj_data.numFaces; i++) {
        // Get the indices of the vertices, texture coordinates, and normals for this face
        int vertexIndex1 = obj_data.facePositions[i].x;
        int normalIndex1 = obj_data.faceNormals[i].x;

        int vertexIndex2 = obj_data.facePositions[i].y;
        int normalIndex2 = obj_data.faceNormals[i].y;

        int vertexIndex3 = obj_data.facePositions[i].z;
        int normalIndex3 = obj_data.faceNormals[i].z;

        // Calculate and set UV coordinates
        if (obj_data.faceTextureCoords[i].x != -1) {
            int textureCoordIndex1 = obj_data.faceTextureCoords[i].x;
            int textureCoordIndex2 = obj_data.faceTextureCoords[i].y;
            int textureCoordIndex3 = obj_data.faceTextureCoords[i].z;

            Ogre::Vector2 uv1(obj_data.textureCoords[textureCoordIndex1].x, 1 - obj_data.textureCoords[textureCoordIndex1].y);
            Ogre::Vector2 uv2(obj_data.textureCoords[textureCoordIndex2].x, 1 - obj_data.textureCoords[textureCoordIndex2].y);
            Ogre::Vector2 uv3(obj_data.textureCoords[textureCoordIndex3].x, 1 - obj_data.textureCoords[textureCoordIndex3].y);

            mesh->textureCoord(uv1); // Set UV coordinate
            mesh->textureCoord(uv2); // Set UV coordinate
            mesh->textureCoord(uv3); // Set UV coordinate
        }

        // Add vertex position, texture coordinate, and normal for each vertex
        mesh->position(obj_data.positions[vertexIndex1]);
        mesh->normal(obj_data.normals[normalIndex1]);

        mesh->position(obj_data.positions[vertexIndex2]);
        mesh->normal(obj_data.normals[normalIndex2]);

        mesh->position(obj_data.positions[vertexIndex3]);
        mesh->normal(obj_data.normals[normalIndex3]);

        // Add the triangle
        mesh->triangle(i * 3, i * 3 + 1, i * 3 + 2);
    }
    mesh->end();
}

void OGREMeshPlugin::recieveMeshTexture(const std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Request> request, std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Response> response) {
    response->success = true;
    
    //TEXTURE - NOT HANDLED HERE, this function must be overriden in implementing class
    //MESH
    if (request->mesh_filepath.size() == 0) {
        RCLCPP_INFO(node_->get_logger(), "Mesh filepath is empty");
        meshLoaded = false;
    } else {
        if (request->mesh_filepath == modelpack_path) {
            RCLCPP_INFO(node_->get_logger(), "Mesh is already loaded");
            meshLoaded = true;
            new_mesh = false;
            response->success = true;
        } else {
            std::ifstream file(request->mesh_filepath.c_str());
            int length = 0;
            while (file.get() != EOF) {
                length ++;
            }
            if (length > 1) {
                meshLoaded = true;
                modelpack_path = request->mesh_filepath;
            } else {
                meshLoaded = false;
                RCLCPP_ERROR(node_->get_logger(), "Mesh is not valid, try checking if it is the correct filepath");
                response->success = false;
            }
            file.close();
            new_mesh = meshLoaded;
        }

    }
}