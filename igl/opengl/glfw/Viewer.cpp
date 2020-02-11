// This file is part of libigl, a simple c++ geometry processing library.
//
// Copyright (C) 2014 Daniele Panozzo <daniele.panozzo@gmail.com>
//
// This Source Code Form is subject to the terms of the Mozilla Public License
// v. 2.0. If a copy of the MPL was not distributed with this file, You can
// obtain one at http://mozilla.org/MPL/2.0/.

#include "Viewer.h"

//#include <chrono>
#include <thread>

#include <Eigen/LU>


#include <cmath>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <limits>
#include <cassert>

#include <igl/project.h>
//#include <igl/get_seconds.h>
#include <igl/readOBJ.h>
#include <igl/readOFF.h>
#include <igl/adjacency_list.h>
#include <igl/writeOBJ.h>
#include <igl/writeOFF.h>
#include <igl/massmatrix.h>
#include <igl/file_dialog_open.h>
#include <igl/file_dialog_save.h>
#include <igl/quat_mult.h>
#include <igl/axis_angle_to_quat.h>
#include <igl/trackball.h>
#include <igl/two_axis_valuator_fixed_up.h>
#include <igl/snap_to_canonical_view_quat.h>
#include <igl/unproject.h>
#include <igl/serialize.h>

// Assignment 2 Includes


// Assignment 3
#define NUM_OF_CYL 10
#define CYL_HEIGHT 1.6
#define CYL_HALF 0.8
#define SPHERE_ID 10
#define SNAKE_TAIL 0
#define SNAKE_HEAD 9
#define ARM_LENGTH (NUM_OF_CYL * CYL_HEIGHT)
#define delta 0.1

//#include <igl/opengl/glfw/imgui/ImGuiMenu.h>

// Internal global variables used for glfw event handling
//static igl::opengl::glfw::Viewer * __viewer;
static double highdpi = 1;
static double scroll_x = 0;
static double scroll_y = 0;


namespace igl
{
namespace opengl
{
namespace glfw
{

  IGL_INLINE void Viewer::init()
  {
	  //data().show_overlay_depth = false;
	  //data().point_size = 10;
	  //data().line_width = 2;
	  ik_animation = false;
  }

  //IGL_INLINE void Viewer::init_plugins()
  //{
  //  // Init all plugins
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->init(this);
  //  }
  //}

  //IGL_INLINE void Viewer::shutdown_plugins()
  //{
  //  for (unsigned int i = 0; i<plugins.size(); ++i)
  //  {
  //    plugins[i]->shutdown();
  //  }
  //}

  IGL_INLINE Viewer::Viewer():
    data_list(1),
    selected_data_index(0),
    next_data_id(1)
  {
    data_list.front().id = 0;

  

    // Temporary variables initialization
   // down = false;
  //  hack_never_moved = true;
    scroll_position = 0.0f;

    // Per face
    data().set_face_based(false);

    
#ifndef IGL_VIEWER_VIEWER_QUIET
    const std::string usage(R"(igl::opengl::glfw::Viewer usage:
  [drag]  Rotate scene
  A,a     Toggle animation (tight draw loop)
  F,f     Toggle face based
  I,i     Toggle invert normals
  L,l     Toggle wireframe
  O,o     Toggle orthographic/perspective projection
  T,t     Toggle filled faces
  [,]     Toggle between cameras
  1,2     Toggle between models
  ;       Toggle vertex labels
  :       Toggle face labels)"
);
    std::cout<<usage<<std::endl;
#endif
  }

  IGL_INLINE Viewer::~Viewer()
  {
  }

  IGL_INLINE bool Viewer::load_mesh_from_file(
      const std::string & mesh_file_name_string)
  {

    // Create new data slot and set to selected
    if(!(data().F.rows() == 0  && data().V.rows() == 0))
    {
      append_mesh();
    }
	
    data().clear();
	
    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }

    std::string extension = mesh_file_name_string.substr(last_dot+1);

    if (extension == "off" || extension =="OFF")
    {
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;
      if (!igl::readOFF(mesh_file_name_string, V, F))
        return false;
      data().set_mesh(V,F);
	  data().F_backup = F;
	  data().V_backup = V;

    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;
      Eigen::MatrixXd V;
      Eigen::MatrixXi F;

      if (!(
            igl::readOBJ(
              mesh_file_name_string,
              V, UV_V, corner_normals, F, UV_F, fNormIndices)))
      {
        return false;
      }

      data().set_mesh(V,F);
      data().set_uv(UV_V,UV_F);
	  data().F_backup = F;
	  data().V_backup = V;
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }

    data().compute_normals();
    data().uniform_colors(Eigen::Vector3d(51.0/255.0,43.0/255.0,33.3/255.0),
                   Eigen::Vector3d(255.0/255.0,228.0/255.0,58.0/255.0),
                   Eigen::Vector3d(255.0/255.0,235.0/255.0,80.0/255.0));

    // Alec: why?
    if (data().V_uv.rows() == 0)
    {
      data().grid_texture();
    }
    

    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->post_load())
    //    return true;

	
	// Assignment 2
	//data().decimationReset();

	// Assignment 3 changes
	/*
	data().show_overlay_depth = false;
	data().point_size = 50;
	data().line_width = 1;

	Eigen::Vector3d m = data().V.colwise().minCoeff();
	Eigen::Vector3d M = data().V.colwise().maxCoeff();

	// Corners of the bounding box
	Eigen::MatrixXd V_box(8, 3);
	V_box <<
		m(0), m(1), m(2),
		M(0), m(1), m(2),
		M(0), M(1), m(2),
		m(0), M(1), m(2),
		m(0), m(1), M(2),
		M(0), m(1), M(2),
		M(0), M(1), M(2),
		m(0), M(1), M(2);

	data().add_points(V_box, Eigen::RowVector3d(1, 0, 0));
	*/
    return true;
  }

  IGL_INLINE bool Viewer::save_mesh_to_file(
      const std::string & mesh_file_name_string)
  {
    // first try to load it with a plugin
    //for (unsigned int i = 0; i<plugins.size(); ++i)
    //  if (plugins[i]->save(mesh_file_name_string))
    //    return true;

    size_t last_dot = mesh_file_name_string.rfind('.');
    if (last_dot == std::string::npos)
    {
      // No file type determined
      std::cerr<<"Error: No file extension found in "<<
        mesh_file_name_string<<std::endl;
      return false;
    }
    std::string extension = mesh_file_name_string.substr(last_dot+1);
    if (extension == "off" || extension =="OFF")
    {
      return igl::writeOFF(
        mesh_file_name_string,data().V,data().F);
    }
    else if (extension == "obj" || extension =="OBJ")
    {
      Eigen::MatrixXd corner_normals;
      Eigen::MatrixXi fNormIndices;

      Eigen::MatrixXd UV_V;
      Eigen::MatrixXi UV_F;

      return igl::writeOBJ(mesh_file_name_string,
          data().V,
          data().F,
          corner_normals, fNormIndices, UV_V, UV_F);
    }
    else
    {
      // unrecognized file type
      printf("Error: %s is not a recognized file type.\n",extension.c_str());
      return false;
    }
    return true;
  }
 
  IGL_INLINE bool Viewer::load_scene()
  {
    std::string fname = igl::file_dialog_open();
    if(fname.length() == 0)
      return false;
    return load_scene(fname);
  }

  IGL_INLINE bool Viewer::load_scene(std::string fname)
  {
   // igl::deserialize(core(),"Core",fname.c_str());
    igl::deserialize(data(),"Data",fname.c_str());
    return true;
  }

  IGL_INLINE bool Viewer::save_scene()
  {
    std::string fname = igl::file_dialog_save();
    if (fname.length() == 0)
      return false;
    return save_scene(fname);
  }

  IGL_INLINE bool Viewer::save_scene(std::string fname)
  {
    //igl::serialize(core(),"Core",fname.c_str(),true);
    igl::serialize(data(),"Data",fname.c_str());

    return true;
  }

  IGL_INLINE void Viewer::open_dialog_load_mesh()
  {
    std::string fname = igl::file_dialog_open();

    if (fname.length() == 0)
      return;

    this->load_mesh_from_file(fname.c_str());
  }

  IGL_INLINE void Viewer::open_dialog_save_mesh()
  {
    std::string fname = igl::file_dialog_save();

    if(fname.length() == 0)
      return;

    this->save_mesh_to_file(fname.c_str());
  }

  IGL_INLINE ViewerData& Viewer::data(int mesh_id /*= -1*/)
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE const ViewerData& Viewer::data(int mesh_id /*= -1*/) const
  {
    assert(!data_list.empty() && "data_list should never be empty");
    int index;
    if (mesh_id == -1)
      index = selected_data_index;
    else
      index = mesh_index(mesh_id);

    assert((index >= 0 && index < data_list.size()) &&
      "selected_data_index or mesh_id should be in bounds");
    return data_list[index];
  }

  IGL_INLINE int Viewer::append_mesh(bool visible /*= true*/)
  {
    assert(data_list.size() >= 1);

    data_list.emplace_back();
    selected_data_index = data_list.size()-1;
    data_list.back().id = next_data_id++;
    //if (visible)
    //    for (int i = 0; i < core_list.size(); i++)
    //        data_list.back().set_visible(true, core_list[i].id);
    //else
    //    data_list.back().is_visible = 0;
    return data_list.back().id;
  }

  IGL_INLINE bool Viewer::erase_mesh(const size_t index)
  {
    assert((index >= 0 && index < data_list.size()) && "index should be in bounds");
    assert(data_list.size() >= 1);
    if(data_list.size() == 1)
    {
      // Cannot remove last mesh
      return false;
    }
    data_list[index].meshgl.free();
    data_list.erase(data_list.begin() + index);
    if(selected_data_index >= index && selected_data_index > 0)
    {
      selected_data_index--;
    
    }


    return true;
  }

  IGL_INLINE size_t Viewer::mesh_index(const int id) const {
    for (size_t i = 0; i < data_list.size(); ++i)
    {
      if (data_list[i].id == id)
        return i;
    }
    return 0;
  }

 
  // Assignment 2 functions


  IGL_INLINE void Viewer::load_meshes_from_config_file(const std::string& mesh_file_name_string) {
		std::ifstream file(mesh_file_name_string);
		std::string cylinder_path;
		if (file.is_open()) {
			getline(file, cylinder_path); // should be cylinder path
			getline(file, foodPath); // should be sphere path
			//while (getline(file, line)) {
			//	load_mesh_from_file(line);
			//}
			file.close();
		}
		else {
			std::cout << " -- configuration file not found --" << std::endl;
			return;
		}

		// Assignment 3
		for (int i = 0; i < NUM_OF_CYL; i++) {
			load_mesh_from_file(cylinder_path);
		}
        createFood();
        //erase_mesh(0);
        //load_mesh_from_file(sphere_path);
        //load_mesh_from_file(sphere_path);
        //erase_mesh(11);

        //data_list[0]
		// Assignment 2 Edge decimation
	    //for (size_t i = 0; i < data_list.size(); ++i) { 
		//    data_list[i].decimationReset();
	    //}

		// Assignment 3
		update_initial_positions();

  }

  IGL_INLINE void Viewer::update_initial_positions() {
	  //data_list[SPHERE_ID].MyTranslate(Eigen::Vector3f(5,0,0)); // Sphere positioning
	  data_list[SNAKE_TAIL].MyTranslate((Eigen::Vector3f(0, -CYL_HALF, 0)));
	  for (size_t i = 0; i < NUM_OF_CYL; i++) {
		  data_list[i].MyTranslate((Eigen::Vector3f(0, CYL_HEIGHT, 0)));
		  data_list[i].setCenterOfRot(Eigen::Vector3f(0, -CYL_HALF, 0));


		  // Points drawing related
		  data_list[i].show_overlay_depth = false;
		  data_list[i].point_size = 10;
		  data_list[i].line_width = 2;
		  data_list[i].add_points(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 0, 1));
		  if (i < SNAKE_HEAD) {
			data_list[i].add_edges(Eigen::RowVector3d(-1.6, +0.8, 0), Eigen::RowVector3d(1.6,+0.8,0), Eigen::RowVector3d(1, 0, 0)); // X axis - red
			data_list[i].add_edges(Eigen::RowVector3d(0, -0.8, 0), Eigen::RowVector3d(0, 2.4, 0), Eigen::RowVector3d(0, 1, 0)); // Y axis - green
			data_list[i].add_edges(Eigen::RowVector3d(0, +0.8, -1.6), Eigen::RowVector3d(0, +0.8, 1.6), Eigen::RowVector3d(0, 0, 1)); // Z axis - blue
		  }
		  if(i > 0){
			  Movable* parent = &data_list[i - 1];
			  data_list[i].setParent(parent);
          }
		  
	  }
  }
  IGL_INLINE bool Viewer::objectReachable(int object_id) {
	  Eigen::RowVector4f armOriginPos = (data_list[SNAKE_TAIL].MakeTrans() * Eigen::Vector4f(0, -0.8, 0, 1));
	  Eigen::RowVector4f objectPos = (data_list[object_id].MakeTrans() * Eigen::Vector4f(0, 0, 0, 1));
	  float distance = (objectPos - armOriginPos).norm();
	  return (distance <= ARM_LENGTH);
  }

  IGL_INLINE void Viewer::IKSolver(int animation_id) {
      if (animation_id < 0 || !objectReachable(animation_id)) { ik_animation = false; return; }
	  //Eigen::RowVector4f armTipPos = (data_list[NUM_OF_CYL].ParentTrans() * data_list[NUM_OF_CYL].MakeTrans() * Eigen::Vector4f(0, +0.8, 0, 1));
	  Eigen::RowVector4f spherePos = (data_list[animation_id].MakeTrans() * Eigen::Vector4f(0, 0, 0, 1));

	  for (int i = SNAKE_HEAD; i >= 0; i--) {
		  Eigen::RowVector4f E = (data_list[SNAKE_HEAD].ParentTrans() * data_list[SNAKE_HEAD].MakeTrans() * Eigen::Vector4f(0, +0.8, 0, 1));
		  Eigen::RowVector4f R = (data_list[i].ParentTrans() * data_list[i].MakeTrans() * Eigen::Vector4f(0, -0.8, 0, 1));
		  const Eigen::RowVector4f D = spherePos;
		  const Eigen::RowVector4f RE = (E - R).normalized();
		  const Eigen::RowVector4f RD = (D - R).normalized();

		  float cosAngle = RE.dot(RD);
		  if (cosAngle > 1) {
			  cosAngle = 1;
		  }
		  if (cosAngle < -1){
			  cosAngle = -1;
		  }
		  float distance = (spherePos - E).norm();
		  if (distance < delta) {
			  ik_animation = false;
              removeFood(animation_id);
              animation_id = -1;
			  return;
		  }
		  //std::cout << "distance " << distance << std::endl;
		  float angleBetween = acos(cosAngle);
		  Vector3f RE3 , RD3;
		  RE3 << RE(0), RE(1), RE(2);
		  RD3 << RD(0), RD(1), RD(2);
		  Eigen::Vector3f rotationAxis = (RE3.cross(RD3)).normalized();
		 
		  data_list[i].MyRotate(rotationAxis, angleBetween);
	  }
  }

  IGL_INLINE void Viewer::createFood() {
      load_mesh_from_file(foodPath);
      data_list[data_list.size() - 1].setParent(NULL); // TODO: delete this
      data_list[data_list.size() - 1].MyTranslate(Eigen::Vector3f(rand() % 10 - 10, rand() % 10 + 1, 0)); // Food positioning
  }

  IGL_INLINE void Viewer::removeFood(int food_id) {
      erase_mesh(food_id);
      // TODO: add sound, restore snake state(?), create some effect
  }

  IGL_INLINE void Viewer::foodAnimation() {
      for (auto& mesh : data_list) {
          if (mesh.id > SNAKE_HEAD) { // if not snake
              mesh.MyTranslate(mesh.direction);
              Eigen::RowVector4f objectPos = (mesh.MakeTrans() * Eigen::Vector4f(0, 0, 0, 1));
              if (objectPos(1) <= 0 || objectPos(1) > 16) { // y <= 0
                  mesh.direction << mesh.direction(0), -mesh.direction(1), mesh.direction(2);
              }
              if (objectPos(0) > 16 || objectPos(0) < -16) {
                  mesh.direction << -mesh.direction(0), mesh.direction(1), mesh.direction(2);
              }
          }
      }
  }


} // end namespace
} // end namespace
}
