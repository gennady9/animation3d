#include "Movable.h"
#include <iostream>

Movable::Movable()
{
	T = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tin = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	Tout = Eigen::Transform<float, 3, Eigen::Affine>::Identity();
	parent = NULL;
}

Eigen::Matrix4f Movable::MakeTrans()
{
	return Tout.matrix() * Tin.matrix();
}

Eigen::Matrix4f Movable::ParentTrans() {
	Eigen::Matrix4f parents = Eigen::Transform<float, 3, Eigen::Affine>::Identity().matrix();
	Movable* parentsIterator = parent;
	while (parentsIterator != NULL) {
		parents = parentsIterator->MakeTrans() * parents;
		parentsIterator = parentsIterator->parent;
	}

	return parents;
}

void Movable::setCenterOfRot(Eigen::Vector3f new_center)
{
	Tin.pretranslate(-new_center);
	Tout.pretranslate(new_center);
}
void Movable::setParent(Movable* parentRef)
{
	this->parent = parentRef;
}
Movable* Movable::getParent()
{
	return this->parent;
}
Eigen::Vector3f Movable::getCenterOfRotation()
{
	return -Tout.translation();
}

// Assignment 3 changes end
void Movable::MyTranslate(Eigen::Vector3f amt)
{
	Tout.pretranslate(amt);
}
//angle in radians
void Movable::MyRotate(Eigen::Vector3f rotAxis, float angle)
{
	Tout.rotate(Eigen::AngleAxisf(angle, rotAxis));
}


void Movable::MyScale(Eigen::Vector3f amt)
{
	Tin.scale(amt);
}

