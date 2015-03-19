#ifndef RAYTRACE_VIEWER_H
#define RAYTRACE_VIEWER_H

#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h> 
#include "FL/Fl.H"

#include "Rendering/Raytracer.h" 

#define REFRESH_RATE .001

class RaytraceViewer : public Fl_Gl_Window{
protected: 
	Raytracer* _tracer; 

public: 
	RaytraceViewer(int x, int y, int w, int h, const char* l=0);
	~RaytraceViewer(); 

	void trace(GLdouble modelview[16], GLdouble proj[16], GLint view[4]); 

	void draw(); 
	int handle(int flag); 
	void init();

	void resize(int x, int y, int width, int height);

	Raytracer* getRaytracer() { return _tracer; }
};


#endif