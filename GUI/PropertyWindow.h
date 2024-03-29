#ifndef PROPERTY_WINDOW_H
#define PROPERTY_WINDOW_H

#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h> 
#include "FL/Fl.H"
#include <FL/Fl_Widget.H> 
#include <FL/Fl_Counter.H> 
#include <FL/Fl_Float_Input.H> 
#include <FL/Fl_Button.H>

#include "Rendering/ShadeAndShapes.h" 
#include "Rendering/Operator.h"  

#define PROPERTY_POS        1
#define PROPERTY_RADIUS     2
#define PROPERTY_AXIS_0     4
#define PROPERTY_AXIS_1     8
#define PROPERTY_AXIS_2    16
#define PROPERTY_LENGTH_0  32
#define PROPERTY_LENGTH_1  64
#define PROPERTY_LENGTH_2 128 

#define PROP_X 0
#define PROP_Y 1
#define PROP_Z 2

class GeometryUpdater; 
class WidgetUpdater; 

class MaterialColorButton : public Fl_Button{
protected: 
	Color _color; 
public: 
	MaterialColorButton(int x, int y, const char* name); 
	void setColor(const Color& c); 
	const Color& getColor() { return _color; }
}; 

class MaterialWindow : public Fl_Window{
protected: 
	Material* _mat; 

	MaterialColorButton* _diffuse, *_specular, *_ambient; 
	Fl_Float_Input* _specExp, *_refl, *_trans, *_refr; 
public: 
	MaterialWindow(int x, int y); 

	void setMaterial(Material* mat); 
	Material* getMaterial() { return _mat; }

protected: 
	static void colorButton(Fl_Widget* widget, void* p); 
	static void valueChanged(Fl_Widget* widget, void* p); 
};

class PropertyWindow : public Fl_Window, public Operand{
private: 
	static PropertyWindow* _singleton; 
	static Operator* _op; 

	MaterialWindow* _matWindow; 

	Geometry* _geom; 
	GeometryUpdater* _geomUpdater; 
	WidgetUpdater* _widgetUpdater; 

	Fl_Counter* _radius;  
	Fl_Float_Input* _posX, *_posY, *_posZ; 

	Fl_Float_Input* _axis0X, *_axis0Y, *_axis0Z; 
	Fl_Float_Input* _axis1X, *_axis1Y, *_axis1Z; 
	Fl_Float_Input* _axis2X, *_axis2Y, *_axis2Z; 
	Fl_Float_Input* _lenX, *_lenY, *_lenZ; 

public: 
	PropertyWindow(); 
	static void openPropertyWindow(Geometry* geom, Operator* op, Material* mat); 
	static void closePropertyWindow(); 

	Fl_Counter* getRadius() { return _radius; }

	Fl_Float_Input* getPos(int c){
		switch(c){
			case PROP_X: return _posX; 
			case PROP_Y: return _posY; 
			default: return _posZ; 
		}
	}

	Fl_Float_Input* getAxis0(int c){
		switch(c){
			case PROP_X: return _axis0X; 
			case PROP_Y: return _axis0Y; 
			default: return _axis0Z; 
		}
	}

	Fl_Float_Input* getAxis1(int c){
		switch(c){
			case PROP_X: return _axis1X; 
			case PROP_Y: return _axis1Y; 
			default: return _axis1Z; 
		}
	}

	Fl_Float_Input* getAxis2(int c){
		switch(c){
			case PROP_X: return _axis2X; 
			case PROP_Y: return _axis2Y; 
			default: return _axis2Z; 
		}
	}

	Fl_Float_Input* getLength(int c){
		switch(c){
			case PROP_X: return _lenX; 
			case PROP_Y: return _lenY; 
			default: return _lenZ; 
		}
	}

	// subclass Operand so we can interactively update the values when the object
	//  is moved or rotated
	virtual Pt3 getCenter(); 
	virtual void translate(const Vec3& trans); 
	virtual void rotate(double d, int axis); 

	void draw(); 

protected: 
	static void handleAllCb(Fl_Widget* widget, void* win);
	static void escapeButtonCb(Fl_Widget* widget, void* win); 

	void setGeometry(Geometry* geom) { _geom = geom; }
	Geometry* getGeometry() { return _geom; }

	GeometryUpdater* getGeometryUpdater() { return _geomUpdater; }
	WidgetUpdater* getWidgetUpdater() { return _widgetUpdater; }
	MaterialWindow* getMaterialWindow() { return _matWindow; }
}; 

// The geometry updater updates the geometry objects when the fields in the property window are changed
class GeometryUpdater : public GeometryVisitor{
	PropertyWindow* _window; 
public: 
	GeometryUpdater(PropertyWindow* win) {_window = win; }

	virtual void visit(Sphere* sphere, void* ret); 
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
	virtual void visit(Torus* op, void* ret); 
	virtual void visit(Operator* op, void* ret){}
}; 

// The widget updater updates the fields when the geometry objects are changed (via operators).  
class WidgetUpdater : public GeometryVisitor{
protected: 
	PropertyWindow* _window; 
public: 
	WidgetUpdater(PropertyWindow* win) {_window = win; }
	virtual void visit(Sphere* sphere, void* ret); 
	virtual void visit(Ellipsoid* op, void* ret); 
	virtual void visit(Box* op, void* ret); 
	virtual void visit(Cylinder* op, void* ret); 
	virtual void visit(Cone* op, void* ret); 
	virtual void visit(Torus* op, void* ret); 
	virtual void visit(Operator* op, void* ret) {} 
}; 



#endif