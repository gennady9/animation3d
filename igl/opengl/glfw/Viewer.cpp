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
#include <igl/png/readPNG.h>
#include <igl\jet.h>

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

    // Project
    score = 0;

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
        std::string cylinder_path, snake_head_path;
		if (file.is_open()) {
			getline(file, cylinder_path); // should be cylinder path
			getline(file, foodPath); // should be sphere path
            getline(file, snake_head_path); // should be snake head path
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
		for (int i = 0; i < SNAKE_HEAD; i++) {
			load_mesh_from_file(cylinder_path);
		}
        load_mesh_from_file(snake_head_path);
        data_list[SNAKE_HEAD].tree.init(data_list[SNAKE_HEAD].V, data_list[SNAKE_HEAD].F);
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
      //data_list[SNAKE_HEAD].MyRotate(Vector3f(1,0,0), 90);
      //data_list[SNAKE_HEAD].MyScale(Eigen::Vector3f(0.5, 0.5, 0.5));//1 - 500 * 0.01, 1 - 500 * 0.01, 1 - 500 * 0.01));
      //data_list[SNAKE_HEAD].MyTranslate(Vector3f(0, 1, -0.7));
      
      // Texturing snake
      Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic> R, G, B, A;
      igl::png::readPNG("snakepng.png", R, G, B, A);


	  for (size_t i = 0; i < NUM_OF_CYL; i++) {
		  data_list[i].MyTranslate((Eigen::Vector3f(0, CYL_HEIGHT, 0)));
		  data_list[i].setCenterOfRot(Eigen::Vector3f(0, -CYL_HALF, 0));

          data_list[i].show_texture = true;
          data_list[i].set_texture(R, G, B);
          /* Rainbow color
          Eigen::MatrixXd C;
          Eigen::VectorXd Z = data_list[i].V.col(2);
          igl::jet(Z, true, C);
          data_list[i].set_colors(C);*/

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
	  Eigen::RowVector4f objectPos = (data_list[animation_id].MakeTrans() * Eigen::Vector4f(0, 0, 0, 1));

	  for (int i = SNAKE_HEAD; i >= 0; i--) {
		  Eigen::RowVector4f E = (data_list[SNAKE_HEAD].ParentTrans() * data_list[SNAKE_HEAD].MakeTrans() * Eigen::Vector4f(0, +0.8, 0, 1));
		  Eigen::RowVector4f R = (data_list[i].ParentTrans() * data_list[i].MakeTrans() * Eigen::Vector4f(0, -0.8, 0, 1));
		  const Eigen::RowVector4f D = objectPos;
		  const Eigen::RowVector4f RE = (E - R).normalized();
		  const Eigen::RowVector4f RD = (D - R).normalized();

		  float cosAngle = RE.dot(RD);
		  if (cosAngle > 1) {
			  cosAngle = 1;
		  }
		  if (cosAngle < -1){
			  cosAngle = -1;
		  }
		  //float distance = (objectPos - E).norm();
		  //if (distance < delta) {
          if(checkCollision(&data_list[SNAKE_HEAD].tree, &data_list[animation_id].tree)){
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
      int index = data_list.size() - 1;
      data_list[index].setParent(NULL); // TODO: delete this
      data_list[index].MyTranslate(Eigen::Vector3f(rand() % 10 - 10, rand() % 10 + 1, 0)); // Food positioning
      data_list[index].tree.init(data_list[index].V, data_list[index].F);

      colorFood(index);
  }
  IGL_INLINE void Viewer::colorFood(int food_id) {
      Eigen::MatrixXd C;
      Eigen::VectorXd Z = data_list[food_id].V.col(2);
      igl::jet(Z, true, C);
      data_list[food_id].set_colors(C);
  }

  IGL_INLINE void Viewer::removeFood(int food_id) {
      erase_mesh(food_id);
      // TODO: add sound, restore snake state(?), create some effect
  }

  IGL_INLINE void Viewer::foodAnimation() {
      for (auto& mesh : data_list) {
          if (mesh.id > SNAKE_HEAD) { // if not snake
              mesh.MyTranslate(mesh.direction);
              mesh.MyRotate(mesh.direction.normalized(), 0.1f);
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
  
  // Assignment 4 Collision detection


  IGL_INLINE bool Viewer::checkCollision(igl::AABB<Eigen::MatrixXd, 3>* Atree, igl::AABB<Eigen::MatrixXd, 3>* Btree) {
      Eigen::AlignedBox<double, 3> Abox = Atree->m_box;
      Eigen::AlignedBox<double, 3> Bbox = Btree->m_box;

      Vector4f DefaultCenterA = Vector4f(Abox.center()(0), Abox.center()(1), Abox.center()(2), 1);
      Vector4f DefaultCenterB = Vector4f(Bbox.center()(0), Bbox.center()(1), Bbox.center()(2), 1);

      Matrix4f transA = data_list[SNAKE_HEAD].ParentTrans() * data_list[SNAKE_HEAD].MakeTrans();
      Matrix4f transB = data_list[animation_id].MakeTrans();
      Matrix3f rotationA = transA.block<3, 3>(0, 0);
      Matrix3f rotationB = transB.block<3, 3>(0, 0);

      Eigen::Vector4f centerA = data_list[SNAKE_HEAD].ParentTrans() * data_list[SNAKE_HEAD].MakeTrans() * DefaultCenterA;
      Eigen::Vector4f centerB = data_list[animation_id].MakeTrans() * DefaultCenterB;
      Eigen::Vector3f D = Vector3f((centerB(0) - centerA(0)), (centerB(1) - centerA(1)), (centerB(2) - centerA(2))).cwiseAbs();
      Vector3f A0 = (rotationA * Vector3f(1, 0, 0));//.normalized();
      Vector3f A1 = (rotationA * Vector3f(0, 1, 0));//.normalized();
      Vector3f A2 = (rotationA * Vector3f(0, 0, 1));//.normalized();	  

      Vector3f B0 = (rotationB * Vector3f(1, 0, 0));//.normalized();
      Vector3f B1 = (rotationB * Vector3f(0, 1, 0));//.normalized();
      Vector3f B2 = (rotationB * Vector3f(0, 0, 1));//.normalized();
      Vector3f A[3] = { A0, A1, A2 };
      Vector3f B[3] = { B0, B1, B2 };
      Eigen::Vector3d a_s = Abox.sizes() / 2;
      Eigen::Vector3d b_s = Bbox.sizes() / 2;
      Eigen::Matrix3f c;
      Eigen::Matrix3f c_n_abs;
      for (size_t i = 0; i <= 2; i++)
      {
          for (size_t j = 0; j <= 2; j++)
          {
              c_n_abs(i, j) = A[i].dot(B[j]);
              c(i, j) = std::abs(A[i].dot(B[j]));
          }
      }
      bool collided = true;
      float R0 = 0;
      float R1 = 0;
      float R = 0;
      //------------------------ CHECK 1-6
      for (size_t i = 0; i <= 2; i++)
      {
          R0 = a_s(i);
          R1 = 0;
          for (size_t j = 0; j <= 2; j++)
          {
              R1 += b_s(i) * c(i, j);
          }
          R = std::abs(A[i].dot(D));
          if (R > R0 + R1) {
              collided = false;
          }
      }
      for (size_t i = 0; i <= 2; i++)
      {
          R0 = b_s(i);
          R1 = 0;
          for (size_t j = 0; j <= 2; j++)
          {
              R1 += a_s(i) * c(i, j);
          }
          R = std::abs(B[i].dot(D));
          if (R > R0 + R1) {
              collided = false;
          }
      }


      //---------------------------Checks 7-9

      for (size_t i = 0; i <= 2; i++)
      {
          R0 = a_s(1) * c(2, i) + a_s(2) * c(1, i);
          //R1 = b_s(1) * c(2, i) + a_s(2) * c(1, i);
          switch (i) {
          case 0: R1 = b_s(1) * c(0, 2) + b_s(2) * c(0, 1); break;
          case 1: R1 = b_s(0) * c(0, 2) + b_s(2) * c(0, 0); break;
          case 2: R1 = b_s(0) * c(0, 1) + b_s(1) * c(0, 0); break;
          }
          float R = std::abs(c_n_abs(1, i) * A[2].dot(D) - c_n_abs(2, i) * A[1].dot(D));
          if (R > R0 + R1) {
              collided = false;
          }
      }

      //--------------------------Checks 10-12
      for (size_t i = 0; i <= 2; i++)
      {
          R0 = a_s(0) * c(2, i) + a_s(2) * c(0, i);
          //R1 = b_s(1) * c(2, i) + a_s(2) * c(1, i);
          switch (i) {
          case 0: R1 = b_s(1) * c(1, 2) + b_s(2) * c(1, 1); break;
          case 1: R1 = b_s(0) * c(1, 2) + b_s(2) * c(1, 0); break;
          case 2: R1 = b_s(0) * c(1, 1) + b_s(1) * c(1, 0); break;
          }
          float R = std::abs(c_n_abs(2, i) * A[0].dot(D) - c_n_abs(0, i) * A[2].dot(D));
          if (R > R0 + R1) {
              collided = false;
          }
      }
      //------------------------Checks 13-15
      for (size_t i = 0; i <= 2; i++)
      {
          R0 = a_s(0) * c(1, i) + a_s(1) * c(0, i);
          //R1 = b_s(1) * c(2, i) + a_s(2) * c(1, i);
          switch (i) {
          case 0: R1 = b_s(1) * c(2, 2) + b_s(2) * c(2, 1); break;
          case 1: R1 = b_s(0) * c(2, 2) + b_s(2) * c(2, 0); break;
          case 2: R1 = b_s(0) * c(2, 1) + b_s(1) * c(2, 0); break;
          }
          float R = std::abs(c_n_abs(0, i) * A[1].dot(D) - c_n_abs(1, i) * A[0].dot(D));
          if (R > R0 + R1) {
              collided = false;
          }
      }

      if (collided) {
          if (Atree->is_leaf() && Btree->is_leaf()) {
              return true;
          }
          else if (Btree->is_leaf()) {
              return (checkCollision(Atree->m_left, Btree) || (checkCollision(Atree->m_right, Btree)));
          }
          else if (Atree->is_leaf()) {
              return (checkCollision(Atree, Btree->m_left) || (checkCollision(Atree, Btree->m_right)));
          }
          else {
              return (checkCollision(Atree->m_left, Btree->m_left) || checkCollision(Atree->m_left, Btree->m_right) || checkCollision(Atree->m_right, Btree->m_left) || checkCollision(Atree->m_right, Btree->m_right));
          }
      }
      return false;
  }

} // end namespace
} // end namespace
}
