#ifndef GEOMETRY_H
#define GEOMETRY_H

#include "Common/Matrix.h" 
#include "Rendering/Scene.h" 
#include <iostream>

#define EPS32 1e-6
#define EPS 1e-6

#define OP_NONE 0
#define OP_TRANSLATE 1
#define OP_ROTATE 2
#define OP_XAXIS 4
#define OP_YAXIS 8
#define OP_ZAXIS 16
#define OP_STEP .25f

class Ray{
public: 
	Pt3 p; 
	Vec3 dir; 
	bool inside; // If ray is currently inside an object
	Pt3 at(float t) const { return p+(dir*t); } 

	Ray() : p(0,0,0), dir(1,0,0,0), inside(false) {}; 

	Ray(const Pt3& p, const Vec3& d) {
		this->p = p;
		this->dir = d; 
		this->inside = false;
	}
}; 

class Plane{
public: 
	Pt3 p; 
	Vec3 n; 

	Plane() : p(0,0,0), n(1,0,0,0) {}
	Plane(const Vec3& p, const Vec3& n){
		this->p = p; 
		this->n = n; 
	}
}; 

class Sphere; 
class Ellipsoid; 
class Box; 
class Cylinder; 
class Cone; 
class Torus; 
class Operator; 

class GeometryVisitor{
public: 
	virtual void visit(Sphere* sphere, void* ret)=0; 
	virtual void visit(Ellipsoid* op, void* ret)=0;
	virtual void visit(Box* op, void* ret)=0;
	virtual void visit(Cylinder* op, void* ret)=0;
	virtual void visit(Cone* op, void* ret)=0;
	virtual void visit(Torus* op, void* ret)=0;
	virtual void visit(Operator* op, void* ret)=0;
}; 

class Geometry : public SceneObject{
protected: 
	float _glmat[16];
	float _glimat[16];
	Mat4 _mat;     // each geometry comes with an affine transformation on the shape
	Mat4 _imat;    // also store the inverse, so when updating the affine transformation, you also need to update the inverse
public: 
	Geometry() {}

	float* getGLInverseMat() { return _glimat; }
	float* getGLForwardMat() { return _glmat; }

	Mat4& getInverseMat() { return _imat; }
	Mat4& getForwardMat() { return _mat; }

	//virtual void updateTransform()=0; // update the transformation matrices
	virtual void updateTransform(){
		for(int j=0;j<16;j++)
			_glmat[j] = _mat[j/4][j%4];
		for(int j=0;j<16;j++)
			_glimat[j] = _imat[j/4][j%4];
	}

	virtual void accept(GeometryVisitor* visitor, void* ret)=0; 
	virtual void accept(SceneObjectVisitor* visitor, void* ret){}
	virtual string toString(){
		return "Geometry";
	};
}; 

class GeometryUtils{
public: 
	static double pointRayClosest(const Pt3& pt, const Ray& ray); 
	static double pointRayDist(const Pt3& pt, const Ray& ray); 
	static double rayRayDist(const Ray& r1, const Ray& r2); 
	static double lineSegRayDist(const Vec3& p0, const Vec3& p1, const Ray& r); 
	static double planeRay(const Plane& pl, const Ray& r); 
	static double planeRayDeg(const Plane& pl, const Vec3& xa, const Ray& r);
	static std::auto_ptr<Mat4> getRotateMatrix(const double d, const int axis);
};

#endif