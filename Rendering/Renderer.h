#ifndef RENDERER_H
#define RENDERER_H

#include "Rendering/Scene.h" 

class Renderer{
protected: 
	Scene* _scene; 
public: 
	Renderer() { _scene = NULL; }
	void setScene(Scene* s) { _scene = s; }
	virtual void draw() =0; 
}; 

#endif