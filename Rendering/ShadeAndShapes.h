#ifndef SHADE_AND_SHAPES_H
#define SHADE_AND_SHAPES_H

#include "Rendering/Operator.h" 
#include "Rendering/Geometry.h" 
#include "Rendering/Scene.h" 

#include "Common/Matrix.h" 
#include "Rendering/Scene.h" 

class Material : public SceneObject {
protected: 
	Color _ambient, _diffuse, _specular; 
	double _specExp, _reflective, _transp, _refractInd; 
public: 
	inline Material() {}; 

	inline const Color& getAmbient() const { return _ambient; }
	inline const Color& getDiffuse() const { return _diffuse; }
	inline const Color& getSpecular() const { return _specular; }
	inline double getSpecExponent() const{ return _specExp; }
	inline double getReflective() const { return _reflective; }
	inline double getTransparency() const{ return _transp; }
	inline double getRefractIndex() const{ return _refractInd; }

	inline void setAmbient(const Color& amb)  { _ambient = amb; }
	inline void setDiffuse(const Color& diff)  { _diffuse = diff; }
	inline void setSpecular(const Color& spec)  { _specular = spec; }
	inline void setSpecExponent(double s) { _specExp = s; }
	inline void setReflective(double r)  { _reflective = r; }
	inline void setTransparency(double t) { _transp = t; }
	inline void setRefractIndex(double r) { _refractInd = r; }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { 
		visitor->visit(this,ret); 
	}
	string toString(){
		return "Material";
	}
}; 

class Light : public  SceneObject{ 
protected: 
	unsigned int _id; 
	Pt3 _pos; 
	Color _color;
	Color _ambient; // keep ambient is easier to code than using a global ambient
public: 
	inline Light(){
		_color = Color(1,1,1); 
	}

	inline Light(const Pt3& pos, const Color& color)
		: _pos(pos), _color(color){}

	inline const Pt3& getPos() const{ return _pos; }
	inline const Color& getColor() const{ return _color; }
	inline const Color& getAmbient() const{ return _ambient; }
	inline unsigned int getId() { return _id; }

	inline void setPos(const Pt3& p) { _pos = p; }
	inline void setColor(const Color& c) { _color = c; }
	inline void setAmbient(const Color& c) { _ambient = c; }
	inline void setId(unsigned int id) { _id = id; }

	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this,ret); }
	string toString(){
		return "Light" + _pos.toString();
	}
};


class Sphere : public Geometry, public Operand{
protected: 
	Pt3 _center; 
	double _rad; 
public: 
	Sphere(){}; 

	Sphere(const Pt3& c, double r){
		_center = c; 
		_rad = r; 
	} 

	void setRadius(double r) { _rad = r; }
	double getRadius() { return _rad; }
	void setCenter(const Pt3& c) { _center = c; }

	Pt3 getCenter() { return _center; }
	void translate(const Vec3& trans); 
	void rotate(double d, int axis); 

	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Sphere" + getCenter().toString();
	}

};

class Box : public Geometry, public Operand{
protected: 
	// I defined box with both a corner and center just for convenience.  This is redundant.  
	// You are free to implement the box or any other shape however you want.  Just make sure it works.
	Pt3 _corner; 
	// Pt3 _center; 
	Vec3 _lengthv; 
	Vec3 _widthv; 
	Vec3 _heightv; 

	double _length;
	double _width; 
	double _height; 

public: 
	Box()  {}; 

	Box(const Pt3& corner, const Vec3& lv, const Vec3& wv, 
		const Vec3& hv, double l, double w, double h){
			_corner = corner; 
			_lengthv = lv; 
			_widthv = wv; 
			_heightv = hv; 
			_length = l; 
			_width = w; 
			_height = h; 
			updateTransform(); 
	}

	Pt3 getCenter() { return _corner; }
	void translate(const Vec3& trans) ;
	void rotate(double d, int axis); 

	Vec3 getLengthVec() { return _lengthv; }
	Vec3 getWidthVec() { return _widthv; }
	Vec3 getHeightVec() { return _heightv; }
	// Pt3 getCorner(){ return _corner; } 

	void setLengthVec(const Vec3& v) { _lengthv = v; }
	void setWidthVec(const Vec3& v) { _widthv = v; }
	void setHeightVec(const Vec3& v) { _heightv = v; }
	void setCenter(const Pt3& v) { _corner = v; }

	double getLength() { return _length; }
	double getWidth() { return _width; }
	double getHeight() { return _height; }

	void setLength(double l) { _length = l; }
	void setWidth(double w) { _width = w; }
	void setHeight(double h) { _height = h; }

	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Box" + getCenter().toString();
	}

}; 

// TODO: finish the definition of the following classes
class Ellipsoid : public Geometry, public Operand{
private:
	Pt3 _center;
	Vec3 _axis1;
	Vec3 _axis2;
	Vec3 _axis3;
	double _len1;
	double _len2;
	double _len3;

public: 
	Ellipsoid() {}; 
	Ellipsoid(const Pt3& center, const Vec3& axis1, const Vec3& axis2,
				const Vec3& axis3, double len1, double len2, double len3) {
		_center = center;
		_axis1 = axis1;
		_axis2 = axis2;
		_axis3 = axis3;
		_len1 = len1;
		_len2 = len2;
		_len3 = len3;
		updateTransform(); 
	};
	Pt3 getCenter() { return _center; }
	inline Vec3 getAxis1() const {return _axis1;}
	inline Vec3 getAxis2() const {return _axis2;}
	inline Vec3 getAxis3() const {return _axis3;}
	inline double getLen1() const { return _len1; }
	inline double getLen2() const { return _len2; }
	inline double getLen3() const { return _len3; }
	inline void setCenter(Pt3 p){_center = p;}
	inline void setAxis1(Vec3 v) {_axis1 = v;}
	inline void setAxis2(Vec3 v) {_axis2 = v;}
	inline void setAxis3(Vec3 v) {_axis3 = v;}
	inline void setLen1(double l) { _len1 = l;}
	inline void setLen2(double l) { _len2 = l;}
	inline void setLen3(double l) { _len3 = l;}

	void translate(const Vec3& trans); 
	void rotate(double d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Ellipsoid" + getCenter().toString();
	}

}; 

class Cylinder : public Geometry, public Operand{
private:
	Pt3 _baseCenter;
	Vec3 _baseAxis1;
	Vec3 _baseAxis2;
	Vec3 _centerAxis;
	double _lenAxis1;
	double _lenAxis2;
	double _height;

public: 
	Cylinder() {}; 
	Cylinder(const Pt3& baseCenter, const Vec3& baseAxis1, const Vec3& baseAxis2,
			const Vec3& centerAxis, double lenAxis1, double lenAxis2, double height){
		_baseCenter = baseCenter;
		_baseAxis1 = baseAxis1;
		_baseAxis2 = baseAxis2;
		_lenAxis1 = lenAxis1;
		_lenAxis2 = lenAxis2;
		_centerAxis = centerAxis;
		_height = height;
		updateTransform(); 
	};

	Pt3 getCenter() { return _baseCenter; }
	inline Vec3 getAxis1() const {return _baseAxis1;}
	inline Vec3 getAxis2() const {return _baseAxis2;}
	inline Vec3 getAxis3() const {return _centerAxis;}
	inline double getLen1() const { return _lenAxis1; }
	inline double getLen2() const { return _lenAxis2; }
	inline double getLen3() const { return _height; }
	inline void setCenter(Pt3 p){_baseCenter = p;}
	inline void setAxis1(Vec3 v) {_baseAxis1 = v;}
	inline void setAxis2(Vec3 v) {_baseAxis2 = v;}
	inline void setAxis3(Vec3 v) {_centerAxis = v;}
	inline void setLen1(double l) { _lenAxis1 = l;}
	inline void setLen2(double l) { _lenAxis2 = l;}
	inline void setLen3(double l) { _height = l;}

	void translate(const Vec3& trans); 
	void rotate(double d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Cylinder" + getCenter().toString();
	}

}; 

class Cone : public Geometry, public Operand{
private:
	Pt3 _baseCenter;
	Vec3 _baseAxis1;
	Vec3 _baseAxis2;
	Vec3 _centerAxis;
	double _lenAxis1;
	double _lenAxis2;
	double _height;

public: 
	Cone() {}; 
	Cone(const Pt3& baseCenter, const Vec3& baseAxis1, const Vec3& baseAxis2,
		const Vec3& centerAxis, double lenAxis1, double lenAxis2, double height){
		_baseCenter = baseCenter;
		_baseAxis1 = baseAxis1;
		_baseAxis2 = baseAxis2;
		_centerAxis = centerAxis;
		_lenAxis1 = lenAxis1;
		_lenAxis2 = lenAxis2;
		_height = height;
		updateTransform(); 
	};

	Pt3 getCenter() { return _baseCenter; }
	inline Vec3 getAxis1() const {return _baseAxis1;}
	inline Vec3 getAxis2() const {return _baseAxis2;}
	inline Vec3 getAxis3() const {return _centerAxis;}
	inline double getLen1() const { return _lenAxis1; }
	inline double getLen2() const { return _lenAxis2; }
	inline double getLen3() const { return _height; }
	inline void setCenter(Pt3 p){_baseCenter = p;}
	inline void setAxis1(Vec3 v) {_baseAxis1 = v;}
	inline void setAxis2(Vec3 v) {_baseAxis2 = v;}
	inline void setAxis3(Vec3 v) {_centerAxis = v;}
	inline void setLen1(double l) { _lenAxis1 = l;}
	inline void setLen2(double l) { _lenAxis2 = l;}
	inline void setLen3(double l) { _height = l;}

	void translate(const Vec3& trans); 
	void rotate(double d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Cone" + getCenter().toString();
	}

}; 

// TODO: finish the definition of the following classes
class Torus : public Geometry, public Operand{
private:
	Pt3 _center;
	Vec3 _axis1;
	Vec3 _axis2;
	Vec3 _axis3;
	double _len1;
	double _len2;
	double _len3;

public: 
	Torus() {}; 
	Torus(const Pt3& center, const Vec3& axis1, const Vec3& axis2,
				const Vec3& axis3, double len1, double len2, double len3) {
		_center = center;
		_axis1 = axis1;
		_axis2 = axis2;
		_axis3 = axis3;
		_len1 = len1;
		_len2 = len2;
		_len3 = len3;
		updateTransform(); 
	};
	Pt3 getCenter() { return _center; }
	inline Vec3 getAxis1() const {return _axis1;}
	inline Vec3 getAxis2() const {return _axis2;}
	inline Vec3 getAxis3() const {return _axis3;}
	inline double getLen1() const { return _len1; }
	inline double getLen2() const { return _len2; }
	inline double getLen3() const { return _len3; }
	inline void setCenter(Pt3 p){_center = p;}
	inline void setAxis1(Vec3 v) {_axis1 = v;}
	inline void setAxis2(Vec3 v) {_axis2 = v;}
	inline void setAxis3(Vec3 v) {_axis3 = v;}
	inline void setLen1(double l) { _len1 = l;}
	inline void setLen2(double l) { _len2 = l;}
	inline void setLen3(double l) { _len3 = l;}

	void translate(const Vec3& trans); 
	void rotate(double d, int axis); 
	void updateTransform(); 

	virtual void accept(GeometryVisitor* visitor, void* ret){ visitor->visit(this, ret); }
	virtual void accept(SceneObjectVisitor* visitor, void* ret) { visitor->visit(this, ret); }
	string toString(){
		return "Torus" + getCenter().toString();
	}

}; 

struct IsectData{
	// you can add more to this struct
	bool hit; 
	double t0; // the parameterized value of the interection
	double t1;
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
	virtual void visit(Torus* op, void* ret); 
	virtual void visit(Operator* op, void* ret); 
}; 

#endif