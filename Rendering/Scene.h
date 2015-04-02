#ifndef SCENE_H
#define SCENE_H

#include <vector> 
#include <map>
#include "Common/Matrix.h" 

class Material; 
class Light; 
class Geometry; 
class Sphere;
class Ellipsoid;  
class Box; 
class Cylinder; 
class Cone; 

class SceneObjectVisitor{
public: 
	virtual void visit(Material* mat, void* ret)=0; 
	virtual void visit(Light* light, void* ret)=0; 
	virtual void visit(Sphere* sphere, void* ret)=0; 
	virtual void visit(Ellipsoid* op, void* ret)=0;
	virtual void visit(Box* op, void* ret)=0;
	virtual void visit(Cylinder* op, void* ret)=0;
	virtual void visit(Cone* op, void* ret)=0;
}; 

class SceneObject{
public: 
	virtual void accept(SceneObjectVisitor* visitor, void* ret) = 0; 
	string toString(){
		return "SceneObject";
	};
};

class Scene{
protected: 
	std::vector<Geometry*> _objs; 
	std::vector<Light*> _lights; 
	std::map<Geometry*,Material*> _mats; 

	Matrix<double,4> _modelview; 
	// these two matrices are the two that are saved in the file
	Matrix<double,4> _translate; 	
	Matrix<double,4> _rotate; 

public: 
	Scene() { }

	void addLight(Light* light) { _lights.push_back(light); }
	void addObject(Geometry* obj){	_objs.push_back(obj); }
	void deleteObject(Geometry* obj);
	void attachMaterial(Geometry* obj, Material* mat);
	void attachMaterial(Geometry* obj);
	void deleteMaterial(Geometry* obj);
	int getNumObjects() { return _objs.size(); }
	int getNumLights() { return _lights.size(); }

	Light* getLight(int i) { return _lights[i]; }
	Geometry* getObject(int i) { return _objs[i]; }
	Material* getMaterial(Geometry* geom) { return _mats[geom]; }

	Matrix<double,4>* getModelview() { return &_modelview; }
	Matrix<double,4>* getTranslate() { return &_translate; }
	Matrix<double,4>* getRotate() { return &_rotate; }
}; 

class SceneUtils{
public: 
	static Scene* readScene(const std::string& fname); 
	static bool writeScene(const std::string& fname, Scene* scene); 
}; 

// Visitor for reading from an input stream, could be a file
class ReadSceneObjectVisitor : public SceneObjectVisitor{
protected: 
	std::istream* _stream; 
public: 

	void setStream(std::istream* i) { _stream = i; }
	virtual void visit(Material* mat, void* ret);
	virtual void visit(Light* light, void* ret);
	virtual void visit(Sphere* sphere, void* ret);
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
}; 

// Visitor for writing to an output stream, could be a file
class WriteSceneObjectVisitor : public SceneObjectVisitor{
protected: 
	std::ostream* _stream; 
public: 

	void setStream(std::ostream* i) { _stream = i; }
	virtual void visit(Material* mat, void* ret);
	virtual void visit(Light* light, void* ret);
	virtual void visit(Sphere* sphere, void* ret);
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
}; 


#endif