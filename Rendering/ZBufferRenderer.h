#ifndef ZBUFFER_RENDERER_H
#define ZBUFFER_RENDERER_H

#include <FL/gl.h>
#include <FL/glut.h> 
#include <FL/glu.h>

#include "Rendering/Renderer.h" 
#include "Rendering/ShadeAndShapes.h"
#include "Rendering/Scene.h" 

class ZBufferVisitor; 
class GLSceneObjectVisitor; 

class ZBufferRenderer : public Renderer{
protected: 
	ZBufferVisitor* _visitor; 
	GLSceneObjectVisitor* _glVisitor; 

	Geometry* _selected; 
	Geometry* _highlighted; 
	Operator* _op; 

	bool _drawGrid; 
public:
	ZBufferRenderer(); 

	void initScene(); 

	virtual void draw(); 
	void setSelected(Geometry* geom); 
	Geometry* getSelected() { return _selected; }
	void setHightlighted(Geometry* geom); 
	void setOperator(Operator* op, int mode); 

	int getOperatorMode(); 

	Operator* getOperator() { return _op; }
	void drawGrid(); 
}; 

#define OP_MODE_STANDARD 0
#define OP_MODE_TRANSLATE 1 
#define OP_MODE_ROTATE 2
#define OP_MODE_XAXIS 4
#define OP_MODE_YAXIS 8
#define OP_MODE_ZAXIS 16

class ZBufferVisitor : public GeometryVisitor{
protected: 
	GLUquadric* _quadric; 
	int _opMode; 
public: 
	ZBufferVisitor(); 

	int getOpMode(){ return _opMode; }
	void setOpMode(int mode) { _opMode = mode; }
	virtual void visit(Sphere* sphere, void* ret); 
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
	virtual void visit(Operator* op, void* ret); 
}; 

// specifiically for setting up lights and materials
class GLSceneObjectVisitor : public SceneObjectVisitor{
protected: 
	GLfloat _alpha; 
public: 
	void setAlpha(GLfloat alpha) { _alpha = alpha; }
	virtual void visit(Material* mat, void* ret); 
	virtual void visit(Light* light, void* ret);  
	virtual void visit(Sphere* sphere, void* ret); 
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
}; 


#endif