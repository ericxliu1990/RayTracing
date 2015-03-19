#ifndef SHAPES_H
#define SHAPES_H

#include "Rendering/Operator.h" 
#include "Rendering/Geometry.h" 
#include "Rendering/Scene.h" 

class Sphere : public Geometry, public Operand{
protected: 
	Pt3 _center; 
	float _rad; 
public: 
	Sphere(){}; 

	Sphere(const Pt3& c, float r){
		_center = c; 
		_rad = r; 
	} 

	void setRadius(float r) { _rad = r; }
	float getRadius() { return _rad; }
	void setCenter(const Pt3& c) { _center = c; }

	Pt3 getCenter() { return _center; }
	void translate(const Vec3& trans); 
	void rotate(float d, int axis); 

	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
};

class Box : public Geometry, public Operand{
protected: 
	// I defined box with both a corner and center just for convenience.  This is redundant.  
	// You are free to implement the box or any other shape however you want.  Just make sure it works.
	Pt3 _corner; 
	Pt3 _center; 
	Vec3 _lengthv; 
	Vec3 _widthv; 
	Vec3 _heightv; 

	float _length;
	float _width; 
	float _height; 

public: 
	Box()  {}; 

	Box(const Pt3& corner, const Vec3& lv, const Vec3& wv, 
		const Vec3& hv, float l, float w, float h){
			_corner = corner; 
			_lengthv = lv; 
			_widthv = wv; 
			_heightv = hv; 
			_length = l; 
			_width = w; 
			_height = h; 
			updateTransform(); 
	}

	Pt3 getCenter() { return _center; }
	void translate(const Vec3& trans) ;
	void rotate(float d, int axis); 

	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	Pt3 getCorner(){ return _corner; } 

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCorner(const Pt3& v) { _corner = v; }

	float getLength() { return _length; }
	float getWidth() { return _width; }
	float getHeight() { return _height; }

	void setLength(float l) { _length = l; }
	void setWidth(float w) { _width = w; }
	void setHeight(float h) { _height = h; }

	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
}; 

// TODO: finish the definition of the following classes
class Ellipsoid : public Geometry, public Operand{
public: 
	Ellipsoid() {}; 

	Pt3 getCenter() { return Pt3(0,0,0); }
	void translate(const Vec3& trans); 
	void rotate(float d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
}; 

class Cylinder : public Geometry, public Operand{
public: 
	Cylinder() {}; 

	Pt3 getCenter() { return Pt3(); }
	void translate(const Vec3& trans); 
	void rotate(float d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
}; 

class Cone : public Geometry, public Operand{
public: 
	Cone() {}; 

	Pt3 getCenter() { return Pt3(); }
	void translate(const Vec3& trans); 
	void rotate(float d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
}; 


struct IsectData{
	bool hit; 
	float t; 
	Vec3 normal; 
}; 

struct IsectAxisData{
	bool hit; 
	int axis; 
}; 

class Intersector : public GeometryVisitor{
protected: 
	Ray _ray; 
public: 
	Intersector() : _ray(Pt3(0,0,0),Vec3(0,0,0,0)) {}
	void setRay(const Ray& r) { _ray = r; }
	virtual void visit(Sphere* sphere, void* ret); 
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
	virtual void visit(Operator* op, void* ret); 
}; 

#endif