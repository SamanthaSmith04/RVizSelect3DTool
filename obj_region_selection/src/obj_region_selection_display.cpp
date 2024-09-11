#include "obj_region_selection_display.hpp"

ObjRegionSelectionDisplay::ObjRegionSelectionDisplay() : OGREMeshPlugin("modelpack_visualization") {
    send_selected_area_service = service_nodes->create_service<obj_region_selection::srv::SendModelpackEditorInfo>("send_modelpack_editor_info", std::bind(&ObjRegionSelectionDisplay::sendSelectedArea, this, std::placeholders::_1, std::placeholders::_2));
    modelpack_selection_stored_state_sub = service_nodes->create_subscription<obj_region_selection::msg::ModelpackSelectionStoredState>("ux/modelpack_selection_stored_state", 10, std::bind(&ObjRegionSelectionDisplay::updateStateWithStoredState, this, std::placeholders::_1));
    auto ros_node_thread = std::thread(std::bind(&ObjRegionSelectionDisplay::runRosNodes, this));
    ros_node_thread.detach();
    
    // start with initial default region
    auto region = obj_region_selection::msg::RegionStoredState();
    region.material_name.data = "default";
    region.region.rgb.push_back(20);
    region.region.rgb.push_back(20);
    region.region.rgb.push_back(20);
    region.region.min_paint_thickness = 0.0;
    region.region.max_paint_thickness = 0.0;
    region.is_visible.data = true;
    modelpack_regions.push_back(region);
}

void ObjRegionSelectionDisplay::loadOBJ(std::string path) {
    modelpack_regions.clear();
    material_thicknesses.clear();
    material_names.clear();

    auto region = obj_region_selection::msg::RegionStoredState();
    for (int i = 0; i < obj_data.facePositions.size(); i++) {
        geometry_msgs::msg::Vector3 vec3 = geometry_msgs::msg::Vector3();
        vec3.x = obj_data.facePositions[i].x;
        vec3.y = obj_data.facePositions[i].y;
        vec3.z = obj_data.facePositions[i].z;
        region.face_vertex_positions.push_back(vec3);
    }
    for (int i = 0; i < obj_data.faceNormals.size(); i++) {
        geometry_msgs::msg::Vector3 vec3 = geometry_msgs::msg::Vector3();
        vec3.x = obj_data.faceNormals[i].x;
        vec3.y = obj_data.faceNormals[i].y;
        vec3.z = obj_data.faceNormals[i].z;
        region.face_vertex_normals.push_back(vec3);
    }
    for (int i = 0; i < obj_data.positions.size(); i++) {
        region.region.affected_verticies.push_back(i);
    }
    region.material_name.data = "default";
    region.region.rgb.push_back(20);
    region.region.rgb.push_back(20);
    region.region.rgb.push_back(20);
    region.region.min_paint_thickness = 0.0;
    region.region.max_paint_thickness = 0.0;
    region.is_visible.data = true;
    modelpack_regions.push_back(region);
}


void ObjRegionSelectionDisplay::loadModelpack(std::string path) {
    modelpack_regions.clear();
    material_thicknesses.clear();
    material_names.clear();

    auto default_region = obj_region_selection::msg::RegionStoredState();
    default_region.material_name.data = "default";
    default_region.region.rgb.push_back(20);
    default_region.region.rgb.push_back(20);
    default_region.region.rgb.push_back(20);
    default_region.region.min_paint_thickness = 0.0;
    default_region.region.max_paint_thickness = 0.0;
    default_region.is_visible.data = true;
    modelpack_regions.push_back(default_region);

    std::ifstream modelpack(path);
    RCLCPP_INFO(node_->get_logger(), "Loading modelpack from %s", path.c_str());

    if (!modelpack.is_open()) {
        RCLCPP_ERROR(node_->get_logger(), "Could not open file %s", path.c_str());
        return;
    }
    still_processing = true;
    std::vector<Ogre::Vector3> vertecies;
    std::vector<Ogre::Vector3> normals;
    std::string line;
    int regionCount = 0;
    int vertexIndex = 0;
    auto region = obj_region_selection::msg::RegionStoredState();
    while (std::getline(modelpack, line)) {
        // GET MATERIAL NAMES
        if (line.find("BEGIN_MTL_DATA") != std::string::npos) {
            while (line.find("END_MTL_DATA") == std::string::npos) {
                std::getline(modelpack, line);
                if (line.find("newmtl") != std::string::npos) {
                    material_names.push_back(line.substr(7));
                    std::vector<double> thicknesses;
                    thicknesses.push_back(0.0);
                    thicknesses.push_back(0.0);
                    material_thicknesses.push_back(thicknesses);
                    // TODO do we want to do something with the pre-defined materials????
                }
            }
        }
        // MATERIAL REQUIREMENTS
        else if (line.find("BEGIN_MODEL_MATERIAL_REQUIREMENTS") != std::string::npos) {
            while (line.find("END_MODEL_MATERIAL_REQUIREMENTS") == std::string::npos) {
                std::getline(modelpack, line);
                if (line.find("requirements") != std::string::npos) {
                    std::string name = line.substr(13);
                    while (line.find("endrequirements") == std::string::npos) {
                        std::getline(modelpack, line);
                        if (line.find("Thickness") != std::string::npos) {
                            auto found = line.find("ABOVE");
                            if (found != std::string::npos) {
                                std::istringstream iss(line.substr(static_cast<int>(found + 5)));
                                double t1, t2, t3;
                                iss >> t1 >> t2 >> t3;
                                for (int i = 0; i < material_names.size(); i++) {
                                    if (material_names[i] == name) {
                                        material_thicknesses[i][0] = t1;
                                    }
                                }
                            }

                            found = line.find("BELOW");
                            if (found != std::string::npos) {
                                std::istringstream iss(line.substr(static_cast<int>(found + 5)));
                                double t1, t2, t3;
                                iss >> t1 >> t2 >> t3;
                                for (int i = 0; i < material_names.size(); i++) {
                                    if (material_names[i] == name) {
                                        material_thicknesses[i][1] = t1;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // OBJ DATA
        else if (line.find("BEGIN_OBJ_DATA") != std::string::npos) {
            while (line.find("END_OBJ_DATA") == std::string::npos) {
                std::getline(modelpack, line);

                if (line.find("usemtl") !=std::string::npos) {
                    std::string name = line.substr(7);
                    // store and reset previous region
                    if (region.face_vertex_positions.size() > 0 && region.material_name.data != "") {
                        auto color = generateRGB();
                        region.region.rgb.push_back(color[0]);
                        region.region.rgb.push_back(color[1]);
                        region.region.rgb.push_back(color[2]);
                        for (int i = 0; i < material_names.size(); i++) {
                            if (material_names[i] == region.material_name.data) {
                                region.region.min_paint_thickness = material_thicknesses[i][0];
                                region.region.max_paint_thickness = material_thicknesses[i][1];
                            }
                        }
                        region.is_visible.data = true;
                        modelpack_regions.push_back(region);
                        region = obj_region_selection::msg::RegionStoredState();
                    }
                    region.material_name.data = name;
                    
                }
                else if (line.substr(0,2) == "f ") {
                    std::istringstream iss(line.substr(2));
                    int v1, v2, v3, t1, t2, t3, n1, n2, n3;
                    char slash;
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

                    // Check if vertex is not already part of the region
                    if (std::find(region.region.affected_verticies.begin(), region.region.affected_verticies.end(), v1-1) == region.region.affected_verticies.end()) {
                        region.region.affected_verticies.push_back(v1-1);
                    }
                    if (std::find(region.region.affected_verticies.begin(), region.region.affected_verticies.end(), v2-1) == region.region.affected_verticies.end()) {
                        region.region.affected_verticies.push_back(v2-1);
                    }
                    if (std::find(region.region.affected_verticies.begin(), region.region.affected_verticies.end(), v3-1) == region.region.affected_verticies.end()) {
                        region.region.affected_verticies.push_back(v3-1);
                    }
                    // Save face vertex indicies
                    geometry_msgs::msg::Vector3 faceIndices = geometry_msgs::msg::Vector3();
                    faceIndices.x = v1-1;
                    faceIndices.y = v2-1;
                    faceIndices.z = v3-1;
                    region.face_vertex_positions.push_back(faceIndices);

                    // Save face normal indicies
                    geometry_msgs::msg::Vector3 normalIndicies = geometry_msgs::msg::Vector3();
                    normalIndicies.x = n1-1;
                    normalIndicies.y = n2-1;
                    normalIndicies.z = n3-1;
                    region.face_vertex_normals.push_back(normalIndicies);
                }
                
            }
        }
    }

    if (region.material_name.data != "" && region.face_vertex_positions.size() > 0) {
        auto color = generateRGB();
        region.region.rgb.push_back(color[0]);
        region.region.rgb.push_back(color[1]);
        region.region.rgb.push_back(color[2]);
        for (int i = 0; i < material_names.size(); i++) {
            if (material_names[i] == region.material_name.data) {
                region.region.min_paint_thickness = material_thicknesses[i][0];
                region.region.max_paint_thickness = material_thicknesses[i][1];
            }
        }
        region.is_visible.data = true;
        modelpack_regions.push_back(region);
    }
    modelpack.close();

    still_processing = false;
}

void ObjRegionSelectionDisplay::recieveMeshTexture(const std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Request> request, std::shared_ptr<obj_region_selection::srv::SendMeshTexture::Response> response) {
    OGREMeshPlugin::recieveMeshTexture(request, response);
    if (new_mesh) {
        if (modelpack_path.substr(modelpack_path.size() - 10) == ".modelpack") {
            loadModelpack(modelpack_path);
        } else {
            RCLCPP_INFO(node_->get_logger(), "Loading OBJ : %s", modelpack_path.c_str());
            obj_data = OBJData();
            readOBJ(modelpack_path);
            loadOBJ(modelpack_path);
        }
    }
}


std::vector<int> ObjRegionSelectionDisplay::generateRGB() {
    std::vector<int> rgb;
    rgb.push_back((rand() % 235) + 20);
    rgb.push_back((rand() % 235) + 20);
    rgb.push_back((rand() % 235) + 20);
    return rgb;
}

void ObjRegionSelectionDisplay::constructMesh(std::string obj_file_path) {
    if (!meshLoaded) {
        return;
    }
    
    if (scene_manager->hasManualObject(node_name + "generated_mesh")) {
        scene_manager->destroyManualObject(node_name + "generated_mesh");
    }

    obj_data = OBJData();

    readOBJ(obj_file_path);
    RCLCPP_INFO(node_->get_logger(), "Read OBJ file");

    // CREATE MESH
    mesh = scene_manager->createManualObject(node_name + "generated_mesh");

    fillMeshWithRegions();

}

void ObjRegionSelectionDisplay::fillMeshWithRegions() {
    Ogre::MaterialPtr regionMaterial;
    Ogre::MaterialPtr borderMaterial;
    for (int regionIndex = 0; regionIndex < modelpack_regions.size(); regionIndex++) {
        if (modelpack_regions[regionIndex].is_visible.data == false) {
            continue;
        }
        if (material_manager->resourceExists(modelpack_regions[regionIndex].material_name.data + "_region_material", node_name + "_resource_group")) {
            material_manager->remove(modelpack_regions[regionIndex].material_name.data + "_region_material", node_name +"_resource_group");
        }
        
        regionMaterial = material_manager->create(modelpack_regions[regionIndex].material_name.data + "_region_material", node_name +"_resource_group");
        auto color = modelpack_regions[regionIndex].region.rgb;
        regionMaterial->setAmbient(Ogre::ColourValue(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0));
        regionMaterial->setDiffuse(Ogre::ColourValue(color[0]/255.0, color[1]/255.0, color[2]/255.0, 1.0));


        if (material_manager->resourceExists(modelpack_regions[regionIndex].material_name.data + "_face_border_material", node_name + "_resource_group")) {
            material_manager->remove(modelpack_regions[regionIndex].material_name.data + "_face_border_material", node_name +"_resource_group");
        }
        borderMaterial = material_manager->create(modelpack_regions[regionIndex].material_name.data + "_face_border_material", node_name +"_resource_group");
        borderMaterial->setAmbient(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));
        borderMaterial->setDiffuse(Ogre::ColourValue(0.0, 0.0, 0.0, 1.0));


        // Face colors
        mesh->begin(modelpack_regions[regionIndex].material_name.data + "_region_material", Ogre::RenderOperation::OT_TRIANGLE_LIST, node_name + "_resource_group");
        for (int i = 0; i < modelpack_regions[regionIndex].face_vertex_positions.size(); i++) {
            // Get the indices of the vertices, texture coordinates, and normals for this face
            int vertexIndex1 = modelpack_regions[regionIndex].face_vertex_positions[i].x;
            int normalIndex1 = modelpack_regions[regionIndex].face_vertex_normals[i].x;

            int vertexIndex2 = modelpack_regions[regionIndex].face_vertex_positions[i].y;
            int normalIndex2 = modelpack_regions[regionIndex].face_vertex_normals[i].y;

            int vertexIndex3 = modelpack_regions[regionIndex].face_vertex_positions[i].z;
            int normalIndex3 = modelpack_regions[regionIndex].face_vertex_normals[i].z;

            // Add vertex position, texture coordinate, and normal for each vertex
            mesh->position(obj_data.positions[vertexIndex1]);
            mesh->normal(obj_data.normals[normalIndex1]);

            mesh->position(obj_data.positions[vertexIndex2]);
            mesh->normal(obj_data.normals[normalIndex2]);

            mesh->position(obj_data.positions[vertexIndex3]);
            mesh->normal(obj_data.normals[normalIndex3]);

            // Add the triangles
            mesh->triangle(i * 3, i * 3 + 1, i * 3 + 2);
        }
        mesh->end();
        mesh->begin(modelpack_regions[regionIndex].material_name.data + "_face_border_material", Ogre::RenderOperation::OT_LINE_LIST, node_name + "_resource_group");
        for (int i = 0; i < modelpack_regions[regionIndex].face_vertex_positions.size(); i++) {
            int normalIndex1 = modelpack_regions[regionIndex].face_vertex_normals[i].x;

            int normalIndex2 = modelpack_regions[regionIndex].face_vertex_normals[i].y;

            int normalIndex3 = modelpack_regions[regionIndex].face_vertex_normals[i].z;

            float borderOffset = 0.0005;
            Ogre::Vector3 offset1 = obj_data.normals[normalIndex1] * borderOffset;
            Ogre::Vector3 offset2 = obj_data.normals[normalIndex2] * borderOffset;
            Ogre::Vector3 offset3 = obj_data.normals[normalIndex3] * borderOffset;

            int vertexIndex1 = modelpack_regions[regionIndex].face_vertex_positions[i].x;
            int vertexIndex2 = modelpack_regions[regionIndex].face_vertex_positions[i].y;
            int vertexIndex3 = modelpack_regions[regionIndex].face_vertex_positions[i].z;


            mesh->position(obj_data.positions[vertexIndex1] + offset1);
            mesh->position(obj_data.positions[vertexIndex2] + offset2);

            mesh->position(obj_data.positions[vertexIndex2] + offset2);
            mesh->position(obj_data.positions[vertexIndex3] + offset3);

            mesh->position(obj_data.positions[vertexIndex3] + offset3);
            mesh->position(obj_data.positions[vertexIndex1] + offset1);

        }
        mesh->end();
    }

}

void ObjRegionSelectionDisplay::sendSelectedArea(const std::shared_ptr<obj_region_selection::srv::SendModelpackEditorInfo::Request> request, std::shared_ptr<obj_region_selection::srv::SendModelpackEditorInfo::Response> response) {
    while (still_processing) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        RCLCPP_INFO(node_->get_logger(), "Waiting for modelpack to finish loading");
    }
    auto selected = request->selected_vertecies;
    std::vector<int> incoming_selection;

    if (request->new_color && request->current_region_index > 0) {
        auto color = generateRGB();
        modelpack_regions[request->current_region_index].region.rgb.clear();
        modelpack_regions[request->current_region_index].region.rgb.push_back(color[0]);
        modelpack_regions[request->current_region_index].region.rgb.push_back(color[1]);
        modelpack_regions[request->current_region_index].region.rgb.push_back(color[2]);
        new_mesh = true;
    } else if (request->new_name.data != "" && request->current_region_index > 0) {
        modelpack_regions[request->current_region_index].material_name.data = request->new_name.data;
        new_mesh = true;
    
    } else if (request->current_region_index >= 0) {
        // move verticies and faces to default and delete region
        if (request->delete_current_region_on_completion) {
            updateCurrentRegion(0);
            for (int i = 0; i < modelpack_regions[request->current_region_index].region.affected_verticies.size(); i++) {
                incoming_selection.push_back(modelpack_regions[request->current_region_index].region.affected_verticies[i]);
            }
            processIncomingSelection(incoming_selection);
            deleteRegion(request->current_region_index);
            new_mesh = true;
        } else { // move verticies and faces to new region
            updateCurrentRegion(request->current_region_index);
            if (selected.size() > 0) {
                for (int i = 0; i < selected.size(); i++) {
                    incoming_selection.push_back(selected[i]);
                }
                processIncomingSelection(incoming_selection);
                new_mesh = true;
            }
            
            if (modelpack_regions[request->current_region_index].is_visible.data != request->visible && request->visible != 0) {
                modelpack_regions[request->current_region_index].is_visible.data = request->visible;
                new_mesh = true;
            }
        }

    }

    for (int regionIndex = 0; regionIndex < modelpack_regions.size(); regionIndex++) {
        obj_region_selection::msg::RegionStoredState region;
        region.material_name.data = modelpack_regions[regionIndex].material_name.data;
        geometry_msgs::msg::Vector3 vec3;
        for (int i = 0; i < modelpack_regions[regionIndex].face_vertex_positions.size(); i++) {
            vec3.x = modelpack_regions[regionIndex].face_vertex_positions[i].x;
            vec3.y = modelpack_regions[regionIndex].face_vertex_positions[i].y;
            vec3.z = modelpack_regions[regionIndex].face_vertex_positions[i].z;
            region.face_vertex_positions.push_back(vec3);
        }

        for (int i = 0; i < modelpack_regions[regionIndex].face_vertex_normals.size(); i++) {
            vec3.x = modelpack_regions[regionIndex].face_vertex_normals[i].x;
            vec3.y = modelpack_regions[regionIndex].face_vertex_normals[i].y;
            vec3.z = modelpack_regions[regionIndex].face_vertex_normals[i].z;
            region.face_vertex_normals.push_back(vec3);
        }

        region.region = modelpack_regions[regionIndex].region;
        region.is_visible.data = modelpack_regions[regionIndex].is_visible.data;
        response->regions.push_back(region);
    }
}

void ObjRegionSelectionDisplay::processIncomingSelection(std::vector<int> incoming_selection) {
    for (int regionIndex = 0; regionIndex < modelpack_regions.size(); regionIndex++) {
        //skip the region to be added to
        if (regionIndex != current_region_index) {
            int faceIndex = 0;
            while (faceIndex < modelpack_regions[regionIndex].face_vertex_positions.size()){
            // for (int faceIndex = 0; faceIndex < modelpack_regions[regionIndex].facePositions.size(); faceIndex++) {
                bool face_found = false;
                // check if the face vertex is in the incoming selection
                int v1 = modelpack_regions[regionIndex].face_vertex_positions[faceIndex].x;
                int v2 = modelpack_regions[regionIndex].face_vertex_positions[faceIndex].y;
                int v3 = modelpack_regions[regionIndex].face_vertex_positions[faceIndex].z;

                auto vertex1 = std::find(incoming_selection.begin(), incoming_selection.end(), v1);
                auto vertex2 = std::find(incoming_selection.begin(), incoming_selection.end(), v2);
                auto vertex3 = std::find(incoming_selection.begin(), incoming_selection.end(), v3);
                if (vertex1 != incoming_selection.end() && vertex2 != incoming_selection.end() && vertex3 != incoming_selection.end()) {
                    face_found = true;
                }
                
                if (face_found) { 
                    std::vector<int> verticesRemoved;

                    verticesRemoved.push_back(modelpack_regions[regionIndex].face_vertex_positions[faceIndex].x);
                    verticesRemoved.push_back(modelpack_regions[regionIndex].face_vertex_positions[faceIndex].y);
                    verticesRemoved.push_back(modelpack_regions[regionIndex].face_vertex_positions[faceIndex].z);

                    auto normal1 = modelpack_regions[regionIndex].face_vertex_normals[faceIndex].x;
                    auto normal2 = modelpack_regions[regionIndex].face_vertex_normals[faceIndex].y;
                    auto normal3 = modelpack_regions[regionIndex].face_vertex_normals[faceIndex].z;

                    modelpack_regions[regionIndex].face_vertex_positions.erase(modelpack_regions[regionIndex].face_vertex_positions.begin() + faceIndex);
                    modelpack_regions[regionIndex].face_vertex_normals.erase(modelpack_regions[regionIndex].face_vertex_normals.begin() + faceIndex);

                    for (int i = 0; i < verticesRemoved.size(); i++) {
                        // if vertex is not used in any other face, remove it from the region
                        if (!vertexInRegion(verticesRemoved[i], modelpack_regions[regionIndex])) {
                            modelpack_regions[regionIndex].region.affected_verticies.erase(std::remove(modelpack_regions[regionIndex].region.affected_verticies.begin(), modelpack_regions[regionIndex].region.affected_verticies.end(), verticesRemoved[i]), modelpack_regions[regionIndex].region.affected_verticies.end());
                        }

                        // if vertex is not already in the new region, add it
                        if (std::find(modelpack_regions[current_region_index].region.affected_verticies.begin(), modelpack_regions[current_region_index].region.affected_verticies.end(), verticesRemoved[i]) == modelpack_regions[current_region_index].region.affected_verticies.end()) {
                            modelpack_regions[current_region_index].region.affected_verticies.push_back(verticesRemoved[i]);
                        }
                    }
                    geometry_msgs::msg::Vector3 face_vec3;
                    face_vec3.x = verticesRemoved[0];
                    face_vec3.y = verticesRemoved[1];
                    face_vec3.z = verticesRemoved[2];

                    geometry_msgs::msg::Vector3 normal_vec3;
                    normal_vec3.x = normal1;
                    normal_vec3.y = normal2;
                    normal_vec3.z = normal3;
                    modelpack_regions[current_region_index].face_vertex_positions.push_back(face_vec3);
                    modelpack_regions[current_region_index].face_vertex_normals.push_back(normal_vec3);
                } else {
                    faceIndex++;
                }
            }
            
        }
    }
    
}

bool ObjRegionSelectionDisplay::vertexInRegion(int vertex, obj_region_selection::msg::RegionStoredState region) {
    for (int faceIndex = 0; faceIndex < region.face_vertex_positions.size(); faceIndex++) {
        if (vertex == region.face_vertex_positions[faceIndex].x || vertex == region.face_vertex_positions[faceIndex].y || vertex == region.face_vertex_positions[faceIndex].z) {
            return true;
        }
    }
    return false;
}

void ObjRegionSelectionDisplay::updateCurrentRegion(int index) {
    if (index < modelpack_regions.size()) {
        current_region_index = index;
        current_region_material_name = modelpack_regions[index].material_name.data;
    } else {
        createNewRegion();
        current_region_index = modelpack_regions.size() - 1;
        current_region_material_name = modelpack_regions[current_region_index].material_name.data;
    }
}

void ObjRegionSelectionDisplay::createNewRegion() {
    auto color = generateRGB();
    auto region = obj_region_selection::msg::RegionStoredState();
    region.material_name.data = "Region" + std::to_string(modelpack_regions.size());
    region.region.rgb.push_back(color[0]);
    region.region.rgb.push_back(color[1]);
    region.region.rgb.push_back(color[2]);
    region.is_visible.data = true;
    modelpack_regions.push_back(region);
}

void ObjRegionSelectionDisplay::deleteRegion(int regionIndex) {
    if (!modelpack_regions.empty() && regionIndex < modelpack_regions.size()) {
        modelpack_regions.erase(modelpack_regions.begin() + regionIndex);
    }
}

void ObjRegionSelectionDisplay::updateStateWithStoredState(obj_region_selection::msg::ModelpackSelectionStoredState state) {
    if (state.region_states.size() == 0) {
        return;
    }
    still_processing = true;
    modelpack_regions.clear();
    for (int i = 0; i < state.region_states.size(); i++) {
        auto region = obj_region_selection::msg::RegionStoredState();
        region.material_name.data = state.region_states[i].material_name.data;
        region.region = state.region_states[i].region;
        for (int j = 0; j < state.region_states[i].face_vertex_positions.size(); j++) {
            geometry_msgs::msg::Vector3 vec3;
            vec3.x = state.region_states[i].face_vertex_positions[j].x;
            vec3.y = state.region_states[i].face_vertex_positions[j].y;
            vec3.z = state.region_states[i].face_vertex_positions[j].z;
            region.face_vertex_positions.push_back(vec3);
        }
        for (int j = 0; j < state.region_states[i].face_vertex_normals.size(); j++) {
            geometry_msgs::msg::Vector3 vec3;
            vec3.x = state.region_states[i].face_vertex_normals[j].x;
            vec3.y = state.region_states[i].face_vertex_normals[j].y;
            vec3.z = state.region_states[i].face_vertex_normals[j].z;
            region.face_vertex_normals.push_back(vec3);
        }
        region.is_visible.data = state.region_states[i].is_visible.data;
        modelpack_regions.push_back(region);
    }
    new_mesh = true;
    still_processing = false;
}