#pragma once
#include <Eigen/core>
#include <Eigen/Geometry>


class Movable
{
public:
	Movable();

	// Assignment 3 changes
	Eigen::Matrix4f MakeTrans();
	Eigen::Matrix4f ParentTrans();
	void MyTranslate(Eigen::Vector3f amt);
	void MyRotate(Eigen::Vector3f rotAxis,float angle);
	void MyScale(Eigen::Vector3f amt);
	void setCenterOfRot(Eigen::Vector3f new_center);
	void setParent(Movable* parentRef);
	Movable* getParent();
	Eigen::Vector3f getCenterOfRotation();
	Eigen::Matrix3f getRotation() const { return Tout.rotation().matrix(); };
private:
	Eigen::Transform<float,3,Eigen::Affine> T;
	Eigen::Affine3f Tout, Tin;
	Movable* parent;
};

