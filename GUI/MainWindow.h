#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <FL/Fl_Window.H>
#include <FL/Fl_Value_Slider.H>
#include <FL/Fl_Choice.H>
#include <FL/Fl_Menu_Bar.H> 
#include <FL/Fl_File_Chooser.H> 
#include <FL/Fl_Button.H> 

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>
#include <FL/gl.h>
#include <FL/glut.h> 
#include <FL/glu.h>

#include "Rendering/ArcBall.h" 
#include "Rendering/Scene.h" 
#include "Rendering/ZBufferRenderer.h" 
#include "Rendering/Operator.h" 

#include "GUI/RaytraceViewer.h" 

#include "Common/Matrix.h" 

#include <vector> 
#include <map> 


#define REFRESH_RATE .001

class MainWindow : public Fl_Window{
protected: 
	Fl_Button* _triButton; 

	static int _w,_h; 
	static int _frames; 
	static double _lastTime; 
	static double _fps; 

	static int _prevMx; 
	static int _prevMy; 
	static Pt3 _prevMPt; 
	static double _prevRot; 

	void init(); 
	static ArcBall::ArcBall_t* _arcBall; 

	static Mat4 _lastRot; 
	static Mat4 _proj; 	

	static int _buttonSt; 
	static int _button; 
	static int _inputMode; 
	static int _holdAxis; 

	static Vec3 _zoomVec; 
	static Vec3 _panVec; 

	static Scene* _scene; 
	static ZBufferRenderer* _zbuffer; 
	static std::map<Geometry*,Operator*> _geom2op; 
	static Intersector* _intersector;
	static Geometry* _selected; 
	static Geometry* _highlighted; 
	static bool _holdCtrl; 
	static bool _holdAlt; 
	static bool _holdShift; 

	static MainWindow* _singleton; 

	static RaytraceViewer* _rtviewer; 

	Fl_Menu_Button* _selectButton;

public: 
	MainWindow(int x, int y, int w, int h, const char* l=0); 
	~MainWindow();

	static Scene* getScene() { return _scene; }

protected: 
	static inline int getWidth() { return _w; } 
	static inline int getHeight() { return _h; } 

	static void display(); 
	static void visible(); 

	static void mouseMove(int x, int y);
	static void mouseEvent (int button, int state, int x, int y); 
	static void mouseEntered(int state);

	static void specialKey( int key, int x, int y );
	static void normalKey( unsigned char key, int x, int y );

	static void idle(); 

	static void updateModelView(); 
	static void handleZoom(int x, int y, bool b); 
	static void handleRot(int x, int y, bool b); 
	static void handleAxisTrans(int x, int y, bool b); 
	static void handleAxisRot(int x, int y, bool b); 
	static void handlePan(int x, int y, bool b); 

	static Ray getMouseRay(int x, int y); 
	static Pt3 getMousePoint(int x, int y); 

	static void escapeButtonCb(Fl_Widget* widget, void* win); 

	static void updateCb(void* userdata){
		MainWindow* viewer = (MainWindow*) userdata; 
		viewer->display();
		Fl::repeat_timeout(REFRESH_RATE,MainWindow::updateCb,userdata);
	}

	static void addSphereCb(Fl_Widget* widget);
	static void addBoxCb(Fl_Widget* widget);
	static void addEllipsoidCb(Fl_Widget* widget);
	static void addCylinderCb(Fl_Widget* widget);
	static void addConeCb(Fl_Widget* widget);
	static void selectButtonCb(Fl_Widget* widget, void* geom);
	static void delObjectCb(Fl_Widget* widget);
	static void openPropertyCb(Fl_Widget* widget);
	static void prepScene(); 
	static void openFile(const std::string& fname); 
	static void saveFile(const std::string& fname); 
	static void exitMenuCb(Fl_Widget* widget, void* win);
	static void openFileMenuCb(Fl_Widget* widget, void* win);
	static void saveFileMenuCb(Fl_Widget* widget, void* win);

	void resize(int x, int y, int w, int h); 
}; 

#endif