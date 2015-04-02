#include "GUI/MainWindow.h" 
#include "GUI/PropertyWindow.h"  
#include "Button.h"
#include <time.h>
#include <iostream> 
#include <sstream> 

#include <FL/Fl_Color_Chooser.H> 
#define _USE_MATH_DEFINES 
#include <cmath>
#ifndef M_PI
#define M_PI 3.1415926535897
#endif

using namespace std; 

#define frand() (rand()/(float)RAND_MAX) 

#define MENU_OPTION_COLOR 0
#define MENU_OPTION_DELETE 1

#define INPUT_VIEWING 0
#define INPUT_SELECTING 1
#define INPUT_EDITING 2
#define INPUT_TRANS 3
#define INPUT_ROT 4

int MainWindow::_frames = 0; 
int MainWindow::_w = 0; 
int MainWindow::_h = 0; 
double MainWindow::_lastTime = 0.f; 
double MainWindow::_fps = 0.f; 
Mat4 MainWindow::_proj; 
Mat4 MainWindow::_lastRot; 

int MainWindow::_prevMx =0; 
int MainWindow::_prevMy =0; 
double MainWindow::_prevRot = 0; 
Pt3 MainWindow::_prevMPt; 
int MainWindow::_buttonSt = -1;
int MainWindow::_button = -1;
bool MainWindow::_holdCtrl = false; 
bool MainWindow::_holdAlt = false; 
bool MainWindow::_holdShift = false; 
int MainWindow::_holdAxis = -1; 
ArcBall::ArcBall_t* MainWindow::_arcBall = NULL; 

Vec3 MainWindow::_panVec = Vec3(0,0,0,0); 
Vec3 MainWindow::_zoomVec = Vec3(0,0,0,0); 

Scene* MainWindow::_scene = NULL; 
ZBufferRenderer* MainWindow::_zbuffer = NULL; 
Intersector* MainWindow::_intersector = NULL; 
std::map<Geometry*,Operator*> MainWindow::_geom2op; 
int MainWindow::_inputMode = INPUT_VIEWING; 
Geometry* MainWindow::_highlighted = NULL; 

RaytraceViewer* MainWindow::_rtviewer = NULL; 
MainWindow* MainWindow::_singleton = NULL; 

const int WIN_LOWER_SPACE = 0; 
const int MENU_SPACE = 0;
const double MOUSE_TOL = 8.f; 
const int RIGHT_PANEL_WIDTH = 160;
const int Button_SPACE = 5;
const int Button_WIDTH = RIGHT_PANEL_WIDTH - Button_SPACE * 2;
const int BUTTON_HEIGHT = 20;

// this is the function to call if you need to update your scene view after some mouse or keyboard input
void MainWindow::updateModelView(){
	if(_scene){
		Mat4* mv = _scene->getModelview(); 
		Mat4* trans = _scene->getTranslate(); 
		Mat4* rot = _scene->getRotate(); 
		(*mv) = (*trans)*(*rot); 
	}
}

// the arcball code has its own matrix class
void convertMat(const ArcBall::Matrix3f_t& mi, Mat4& mout){
	mout[0][0] = mi.s.M00;
	mout[1][0] = mi.s.M10;
	mout[2][0] = mi.s.M20;
	mout[3][0] = 0; 
	mout[0][1] = mi.s.M01;
	mout[1][1] = mi.s.M11;
	mout[2][1] = mi.s.M21;
	mout[3][1] = 0; 
	mout[0][2] = mi.s.M02;
	mout[1][2] = mi.s.M12;
	mout[2][2] = mi.s.M22;
	mout[3][2] = 0; 
	mout[3][0] = 0; 
	mout[3][1] = 0; 
	mout[3][2] = 0; 
	mout[3][3] = 1; 
}

void mglLoadMatrix(const Mat4& mat){
	GLfloat m[16]; 
	for(int i=0;i<16;i++) 
		m[i]=mat[i>>2][i%4];
	glLoadMatrixf(m); 
}

void mglReadMatrix(GLenum glmat, Mat4& mat){
	GLfloat m[16]; 
	glGetFloatv(glmat, m);
	for(int i=0;i<16;i++) 
		mat[i>>2][i%4] = m[i];
}

void mLoadMatrix(const Mat4& mat, GLdouble m[16]){
	for(int i=0;i<16;i++) 
		m[i]=mat[i>>2][i%4];
}

void mReadMatrix(GLdouble m[16], Mat4& mat){
	for(int i=0;i<16;i++) 
		mat[i>>2][i%4] = m[i];
}

char* getChar(const std::string& s){
	char* c = new char[s.length()+1]; 
	for(unsigned int j=0;j<s.length();j++)
		c[j] = s[j]; 
	c[s.length()] = 0; 
	return c; 
}

MainWindow::MainWindow(int x, int y, int w, int h, const char* l) : 
Fl_Window(x,y,w + RIGHT_PANEL_WIDTH, h + MENU_SPACE + WIN_LOWER_SPACE, l){

	show();

	begin();

	// This is where the glut rendering window is added to the FLTK window
	glutInitWindowSize(w, h);
	glutInitWindowPosition(0,MENU_SPACE); // place it inside parent window
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH | GLUT_MULTISAMPLE);
	glutCreateWindow("Rendering Window");
	end();

	resizable(glut_window);

	// same old glut callbacks
	glutDisplayFunc(MainWindow::display);
	glutMotionFunc(MainWindow::mouseMove);			
	glutPassiveMotionFunc(MainWindow::mouseMove);	
	glutMouseFunc(MainWindow::mouseEvent);			
	glutKeyboardFunc(MainWindow::normalKey);		
	glutSpecialFunc(MainWindow::specialKey);		
	glutEntryFunc(MainWindow::mouseEntered);		
	glutIdleFunc(MainWindow::idle);
	end(); 

	_w = w; 
	_h = h; 
	_lastTime = clock()/((double)CLOCKS_PER_SEC); 

	_arcBall = new ArcBall::ArcBall_t((float)_w,(float)_h); 
	_zbuffer = new ZBufferRenderer(); 
	_intersector = new Intersector(); 
	_rtviewer = new RaytraceViewer(100,100,w,h,"Raytracer"); 
	// Use a smaller window to trace out a smaller picture can be much faster.  Good for testing purpose.
	// _rtviewer = new RaytraceViewer(100,100,400,400,"Raytracer"); 

	this->callback(escapeButtonCb,this);
	Fl::repeat_timeout(REFRESH_RATE,MainWindow::updateCb,this);

	openFile("files/default.ray"); 

	if(_scene){
		// must have scene file before you init
		MainWindow::init(); 
		_singleton = this; 
	}

	// the right panel, add objects and delete objects
	begin();

	Button* addSphere = new Button(w + Button_SPACE, 
									MENU_SPACE + Button_SPACE, 
									Button_WIDTH, 
									BUTTON_HEIGHT, 
									"Add Sphere");
	addSphere->callback(addSphereCb);

	Button* addBox = new Button(w + Button_SPACE, 
									MENU_SPACE + BUTTON_HEIGHT + Button_SPACE * 2, 
									Button_WIDTH, 
									BUTTON_HEIGHT, 
									"Add Box");
	addBox->callback(addBoxCb);


	Button* addEllipsoid = new Button(w + Button_SPACE, 
									MENU_SPACE + BUTTON_HEIGHT * 2 + Button_SPACE * 3, 
									Button_WIDTH, 
									BUTTON_HEIGHT, 
									"Add Ellipsoid");
	addEllipsoid->callback(addEllipsoidCb);

	Button* addCylinder = new Button(w + Button_SPACE, 
									MENU_SPACE + BUTTON_HEIGHT * 3 + Button_SPACE * 4, 
									Button_WIDTH, 
									BUTTON_HEIGHT, 
									"Add Cylinder");
	addCylinder->callback(addCylinderCb);

	Button* addCone = new Button(w + Button_SPACE, 
									MENU_SPACE + BUTTON_HEIGHT * 4 + Button_SPACE * 5, 
									Button_WIDTH, 
									BUTTON_HEIGHT, 
									"Add Cone");
	addCone->callback(addConeCb);
	
	_selectButton = new Fl_Menu_Button(w + Button_SPACE,
										MENU_SPACE + BUTTON_HEIGHT* 5  + Button_SPACE * 6, 
										Button_WIDTH,
										BUTTON_HEIGHT,
										"Selected");
	
	for(int j=0;j<_scene->getNumObjects();j++){
		Geometry* geom = _scene->getObject(j); 
		_selectButton->add(geom->toString().c_str(), 0, selectButtonCb, geom, 0);
	}


	Button* openPropertyWindow = new Button(w + Button_SPACE, 
										MENU_SPACE + BUTTON_HEIGHT* 6  + Button_SPACE * 7, 
										Button_WIDTH, 
										BUTTON_HEIGHT, 
										"Open Property Window");
	openPropertyWindow->callback(openPropertyCb);

	Button* delectObject = new Button(w + Button_SPACE, 
										MENU_SPACE + BUTTON_HEIGHT* 7  + Button_SPACE * 8, 
										Button_WIDTH, 
										BUTTON_HEIGHT, 
										"Delete Selected Object");
	delectObject->callback(delObjectCb);

	end();
}

void MainWindow::exitMenuCb(Fl_Widget* widget, void* win){
	exit(1);
}


// you can also use this as a button/menu callback
void MainWindow::openFileMenuCb(Fl_Widget* widget, void* win){
	char* newfile = fl_file_chooser("Open Scene", ".ray (*.ray)", "./files", 0);
	if(!newfile) return; 
	MainWindow* window = (MainWindow*) win;
	if(window) 
		window->openFile(string(newfile));
}

void MainWindow::saveFileMenuCb(Fl_Widget* widget, void* win){
	char* newfile = fl_file_chooser("Save Scene", ".ray (*.ray)", "./files", 0);
	if(!newfile) return; 
	MainWindow* window = (MainWindow*) win;
	if(window) 
		window->saveFile(string(newfile));
}

/**
 * @brief callback funttion of add object button
 * @details add a default sphere to the scene
 * 
 * @param widget required first parameter
 */
void MainWindow::addSphereCb(Fl_Widget* widget)
{
	// default sphere
	Sphere* geom = new Sphere(Pt3(0, 1, 0), 0.5f); 
	getScene()->addObject(geom);
	getScene()->attachMaterial(geom); 
	debugInfo("Add" + geom->toString());
	prepScene();
}
/**
 * @brief callback funttion of add object button
 * @details add a default Box to the scene
 * 
 * @param widget required first parameter
 */
void MainWindow::addBoxCb(Fl_Widget* widget)
{
	Box* geom = new Box(Pt3(0.0f, 0.0f, 0.5f), 
						Vec3(1.0f, 0.0f, 0.0f),
						Vec3(0.0f, 1.0f, 0.0f),
						Vec3(0.0f, 0.0f, 1.0f),
						0.6f, 0.6f, 0.6f);
	geom->updateTransform();
	getScene()->addObject(geom);
	getScene()->attachMaterial(geom); 
	debugInfo("Add" + geom->toString());
	prepScene();
}
/**
 * @brief callback funttion of add object button
 * @details add a default Ellipsoid to the scene
 * 
 * @param widget required first parameter
 */
void MainWindow::addEllipsoidCb(Fl_Widget* widget)
{
	Ellipsoid* geom = new Ellipsoid(Pt3(0.0f, 0.0f, 0.0f), 
						Vec3(1.0f, 0.0f, 0.0f),
						Vec3(0.0f, 1.0f, 0.0f),
						Vec3(0.0f, 0.0f, 1.0f),
						0.5f, 0.5f, 0.5f);
	getScene()->addObject(geom);
	getScene()->attachMaterial(geom); 
	debugInfo("Add" + geom->toString());
	prepScene();
}
/**
 * @brief callback funttion of add object button
 * @details add a default Cylinder to the scene
 * 
 * @param widget required first parameter
 */
void MainWindow::addCylinderCb(Fl_Widget* widget)
{
	Cylinder* geom = new Cylinder(Pt3(0.0f, 0.0f, 0.0f), 
						Vec3(1.0f, 1.0f, 1.0f),
						Vec3(0.0f, 1.0f, 0.0f),
						Vec3(0.0f, 0.0f, 1.0f),
						0.5f, 0.5f, 1.0f);
	getScene()->addObject(geom);
	getScene()->attachMaterial(geom); 
	debugInfo("Add" + geom->toString());
	prepScene();
}
/**
 * @brief callback funttion of add object button
 * @details add a default Cone to the scene
 * 
 * @param widget required first parameter
 */
void MainWindow::addConeCb(Fl_Widget* widget)
{
	Cone* geom = new Cone();
	getScene()->addObject(geom);
	getScene()->attachMaterial(geom); 
	debugInfo("Add" + geom->toString());
	prepScene();
}
/**
 * @brief callback function of the select button
 * @details select objects from the scene
 * 
 * @param widget [description]
 * @param gemo the selected object
 */
void MainWindow::selectButtonCb(Fl_Widget* widget, void* data){
	Geometry* geom = (Geometry*)data;
	debugInfo(geom->toString());
	_highlighted = geom;
	//display the highlighted line
	_zbuffer->setHightlighted(_highlighted); 
}
/**
 * @brief callback funtion of delete button
 * @details delete the selected object from the scene
 * 
 * @param widget require first parameter
 */
void MainWindow::delObjectCb(Fl_Widget* widget)
{
	if(_zbuffer->getSelected() || _highlighted)
	{
		Geometry* deleteobj;
		deleteobj = _highlighted ? _highlighted : _zbuffer->getSelected();
		// get the selected object and delect it
		debugInfo("Delete" + deleteobj->toString());
		getScene()->deleteObject(deleteobj);
		//clear operator and close the property window
		if (_zbuffer->getOperator())
		{
			_zbuffer->getOperator()->setState(OP_NONE);
			_zbuffer->setSelected(NULL); 
			_zbuffer->setOperator(NULL,0); 
		}
		PropertyWindow::closePropertyWindow(); 
		_highlighted = NULL;
		prepScene();
	}else{
		cout << "You Need select one object" << endl;
	}
}
/**
 * @brief callback funtion of open button
 * @details open the selected object from the scene
 * 
 * @param widget require first parameter
 */
void MainWindow::openPropertyCb(Fl_Widget* widget)
{
	// this is an example of how to activate and deactive an operator
	if(_highlighted){
		_zbuffer->setSelected(_highlighted);
		_zbuffer->setOperator(_geom2op[_highlighted],OP_MODE_TRANSLATE); 
		_zbuffer->getOperator()->setState(OP_TRANSLATE); 
		_inputMode = INPUT_EDITING; 
		PropertyWindow::openPropertyWindow(_highlighted,_geom2op[_highlighted],_scene->getMaterial(_highlighted)); 
		_highlighted = NULL;
		_zbuffer->setHightlighted(NULL);
	}

}

void MainWindow::prepScene(){
	if(_scene){
		// this is connecting/disconnecting operators to geometries
		if(_geom2op.size()>0){
			for(map<Geometry*,Operator*>::iterator i=_geom2op.begin();i!=_geom2op.end();i++)
				delete i->second; 
			_geom2op.clear(); 
		}
		if (_singleton &&_singleton->_selectButton)
		{
			_singleton->_selectButton->clear();
		}

		for(int j=0;j<_scene->getNumObjects();j++){
			Geometry* geom = _scene->getObject(j); 
			Operator* op = new Operator(dynamic_cast<Operand*>(geom)); 
			_geom2op[geom] = op; 
			if (_singleton &&_singleton->_selectButton)
			{
				_singleton->_selectButton->add(geom->toString().c_str(), 0, selectButtonCb, geom, 0);
			}
		}
		_zbuffer->setScene(_scene); 
		_zbuffer->initScene(); 

		_rtviewer->getRaytracer()->setScene(_scene); 
		updateModelView();
		debugInfo("Called prepScene!");

	}
}

void MainWindow::openFile(const string& fname){
	if(_scene)
		delete _scene; 

	_scene = SceneUtils::readScene(fname); 
	prepScene(); 
}

void MainWindow::saveFile(const string& fname){
	SceneUtils::writeScene(fname,_scene); 
}

// this is the main display function, calls the zbuffer renderer
void MainWindow::display(){
	if(_scene){
		glClear(GL_DEPTH_BUFFER_BIT|GL_COLOR_BUFFER_BIT); 
		glEnable(GL_LIGHTING); 
		glEnable(GL_DEPTH_TEST);
		glMatrixMode(GL_PROJECTION); 
		glPushMatrix(); 
		mglLoadMatrix(_proj);

		glMatrixMode(GL_MODELVIEW); 
		glPushMatrix(); 
		mglLoadMatrix(*_scene->getModelview()); 

		if(_zbuffer)
			_zbuffer->draw(); 

		glMatrixMode(GL_PROJECTION); 
		glPopMatrix(); 
		glMatrixMode(GL_MODELVIEW); 
		glPopMatrix(); 

		glDisable(GL_LIGHTING); 
		glutSwapBuffers(); 
	}
}

MainWindow::~MainWindow(){}
 
void MainWindow::resize(int x0, int y0, int w, int h){
	Fl_Window::resize(x0,y0,w,h); 
	_w = w - RIGHT_PANEL_WIDTH; 
	_h = h-(WIN_LOWER_SPACE+MENU_SPACE); 

	glViewport(0,0,_w,_h);
	if(_arcBall)
		_arcBall->setBounds((float)w,(float)h); 
}

void MainWindow::init(){
	glClearColor(1.f,1.f,1.f,1.f); 
	glEnable(GL_POLYGON_SMOOTH); 
	glEnable(GL_LINE_SMOOTH); 
	glEnable(GL_POINT_SMOOTH); 

	glEnable(GL_COLOR_MATERIAL);
	glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE); 
	glEnable(GL_CULL_FACE); 
	glEnable(GL_DEPTH_TEST);

	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	gluPerspective(45.f,1.f,.1f,200.f); 
	mglReadMatrix(GL_PROJECTION_MATRIX,_proj); 
}

// handles the camera rotation (arcball)
void MainWindow::handleRot(int x, int y, bool beginRot){
	if(beginRot){
		ArcBall::Tuple2f_t mousePt; 
		mousePt.s.X = (float) x; 
		mousePt.s.Y = (float) y; 

		_lastRot = (*_scene->getRotate()); 
		_arcBall->click(&mousePt);	
	}
	else{
		ArcBall::Tuple2f_t mousePt; 
		mousePt.s.X = (float)x; 
		mousePt.s.Y = (float)y; 

		ArcBall::Quat4fT thisQuat;
		ArcBall::Matrix3f_t thisRot; 
		_arcBall->drag(&mousePt, &thisQuat);					
		ArcBall::Matrix3fSetRotationFromQuat4f(&thisRot, &thisQuat);		

		Mat4* rot = _scene->getRotate(); 
		convertMat(thisRot,*rot); 
		(*rot) = _lastRot * (*rot); 
		updateModelView(); 
	}
}

// handles the camera zoom
void MainWindow::handleZoom(int x, int y, bool beginZoom){

	int dy = y - _prevMy;

	if(beginZoom){
		Ray r = getMouseRay(_w/2,_h/2); 
		_zoomVec = r.dir; 
	}
	else{
		Mat4* trans = _scene->getTranslate(); 
		(*trans)[3][0]+=_zoomVec[0]*dy*.03f; 
		(*trans)[3][1]+=_zoomVec[1]*dy*.03f; 
		(*trans)[3][2]+=_zoomVec[2]*dy*.03f; 
		updateModelView(); 
	}
}

// handles camera pan
void MainWindow::handlePan(int x, int y, bool beginPan){

	int dy = y - _prevMy;

	if(beginPan){
		Ray r = getMouseRay(x,y); 
		_panVec = r.p + r.dir*3.f; 
	}
	else{
		Ray r = getMouseRay(x,y); 
		Pt3 p = r.p + r.dir*3.f; 
		Vec3 v = (p-_panVec); 
		Mat4* trans = _scene->getTranslate(); 
		(*trans)[3][0]+=v[0];
		(*trans)[3][1]+=v[1]; 
		(*trans)[3][2]+=v[2]; 
		updateModelView(); 
	}
}

// gets the ray shooting into the scene corresponding to a position on the screen
Ray MainWindow::getMouseRay(int mx, int my){
	if(_scene){
		GLdouble mv[16],proj[16];
		GLint viewport[4]={0,0,_w,_h}; 
		
		mLoadMatrix(*_scene->getModelview(),mv); 
		mLoadMatrix(_proj,proj); 
		GLdouble ax,ay,az; 
		GLdouble bx,by,bz; 
		gluUnProject(mx,(_h-my),0.f,mv,proj,viewport,&ax,&ay,&az); 
		gluUnProject(mx,(_h-my),.5f,mv,proj,viewport,&bx,&by,&bz); 

		Ray ret(
			Pt3(ax,ay,az),
			Vec3((bx-ax),(by-ay),(bz-az),0)); 

		ret.dir.normalize(); 

		return ret; 
	}
	return Ray(); 
}

Pt3 MainWindow::getMousePoint(int mx, int my){
	if(_scene){
		GLdouble mv[16],proj[16];
		GLint viewport[4]={0,0,_w,_h}; 
		
		mLoadMatrix(*_scene->getModelview(),mv); 
		mLoadMatrix(_proj,proj); 
		GLdouble ax,ay,az; 
		gluUnProject(mx,(_h-my),0.f,mv,proj,viewport,&ax,&ay,&az); 

		return Pt3(ax,ay,az); 
	}
	return Pt3(0,0,0); 
}

// This handles the case when an operator translation is performed
void MainWindow::handleAxisTrans(int mx, int my, bool beginTrans){
	Operator* op = _zbuffer->getOperator(); 
	Ray r = getMouseRay(mx,my); 
	Pt3 np; 
	switch(_holdAxis){
		case OP_XAXIS: np = op->getDirX()+op->getPrimaryOp()->getCenter(); break; 
		case OP_YAXIS: np = op->getDirY()+op->getPrimaryOp()->getCenter(); break; 
		case OP_ZAXIS: np = op->getDirZ()+op->getPrimaryOp()->getCenter(); break; 
	}

	Pt3 cpt = r.at(GeometryUtils::pointRayClosest(np,r)); 
	if(!beginTrans){
		switch(_holdAxis){
			case OP_XAXIS: op->translate(Vec3(cpt[0]-_prevMPt[0],0,0,0)); break; 
			case OP_YAXIS: op->translate(Vec3(0,cpt[1]-_prevMPt[1],0,0)); break; 
			case OP_ZAXIS: op->translate(Vec3(0,0,cpt[2]-_prevMPt[2],0)); break; 
		}
	}
	_prevMPt = cpt; 
}

// This handles the case when an operator rotation occurs
void MainWindow::handleAxisRot(int mx, int my, bool beginRot){
	Operator* op = _zbuffer->getOperator(); 
	Ray r = getMouseRay(mx,my); 
	Pt3 center = op->getPrimaryOp()->getCenter(); 

	Vec3 normal, axis; 
	switch(_holdAxis){
		case OP_XAXIS: 
			normal = Vec3(1,0,0,0); 
			axis = Vec3(0,0,1,0); 
			break; 
		case OP_YAXIS: 
			normal = Vec3(0,1,0,0); 
			axis = Vec3(1,0,0,0); 
			break; 
		case OP_ZAXIS: 
			normal = Vec3(0,0,1,0); 
			axis = Vec3(0,1,0,0); 
			break; 
	}

	double deg = GeometryUtils::planeRayDeg(Plane(center,normal),axis,r); 
	if(!beginRot){
		double diff = _prevRot-deg; 

		if(diff > M_PI)
			diff-=(double)(2*M_PI); 
		else if(diff < -M_PI)
			diff+=(double)(2*M_PI); 

		op->rotate(diff,_holdAxis); 
	}
	_prevRot = deg;
}

// The mouse move event
void MainWindow::mouseMove(int x, int y){
	if(_inputMode==INPUT_VIEWING){
		if(_buttonSt==GLUT_DOWN){
			if(_button==GLUT_LEFT_BUTTON){
				if(_holdAlt) handlePan(x,y,false); 
				else         handleRot(x,y,false); 
			}
			else if(_button==GLUT_RIGHT_BUTTON)
				handleZoom(x,y,false); 
		}
	}
	else if(_inputMode==INPUT_TRANS){
		if(_holdAxis>=0){		
			handleAxisTrans(x,y,false); 
		}
	}
	else if(_inputMode==INPUT_ROT){
		if(_holdAxis>=0){		
			handleAxisRot(x,y,false); 
		}
	}
	else if(_inputMode==INPUT_EDITING){
		Operator* op = _zbuffer->getOperator(); 
		if(op){
			// debugInfo(op->toString());
			IsectAxisData data; 
			Ray r = getMouseRay(x,y); 
			_intersector->setRay(r); 
			op->accept(_intersector,&data); 

			int editMode = _holdShift? OP_MODE_ROTATE : OP_MODE_TRANSLATE; 
			// debugInfo("IsectAxisData.hit", data.hit);
			if(data.hit){
				if(_zbuffer->getOperatorMode()!=(editMode|data.axis))
					_zbuffer->setOperator(op,editMode|data.axis); 
				_holdAxis = data.axis;
			}
			else{
				if(_zbuffer->getOperatorMode()!=editMode)
					_zbuffer->setOperator(op,editMode); 
				_holdAxis = -1; 
			}

			if(_buttonSt==GLUT_DOWN){
				if(_button==GLUT_LEFT_BUTTON){
					if(_holdAlt) handlePan(x,y,false); 
					else         handleRot(x,y,false); 
				}
				else if(_button==GLUT_RIGHT_BUTTON)
					handleZoom(x,y,false); 
			}
		}
		else
			_inputMode=INPUT_VIEWING; 
	}
	// do raytracing to find the selected object
	else if(_inputMode==INPUT_SELECTING){
		IsectData data;

		Ray r = getMouseRay(x,y); 
		_intersector->setRay(r); 

        #define LARGE_T 1000000
		double t = LARGE_T;
		Geometry* hobj = NULL; 
		for(int j=0;j<_scene->getNumObjects();j++){
			_scene->getObject(j)->accept(_intersector,&data); 
			if(data.hit && t>data.t0){
				t = data.t0; 
				hobj = _scene->getObject(j); 
			}
		}

		_highlighted = hobj; 
	}

	_zbuffer->setHightlighted(_highlighted); 
	_prevMx = x;
	_prevMy = y;
}

// mouse event (button presses)
void MainWindow::mouseEvent (int button, int state, int x, int y){
	double w = (double) getWidth(); 
	double h = (double) getHeight(); 

	_button = button; 
	_buttonSt = state;

	_prevMx = x; 
	_prevMy = y; 

	if(_inputMode==INPUT_VIEWING){ 
		if(_buttonSt==GLUT_DOWN){
			if(!_highlighted){
				if(button==GLUT_LEFT_BUTTON){
					if(_holdAlt) handlePan(x,y,true); 
					else         handleRot(x,y,true); 
				}
				if(button==GLUT_RIGHT_BUTTON)
					handleZoom(x,y,true); 
			}
		}
	}
	else if(_inputMode==INPUT_TRANS){
		if(_buttonSt==GLUT_UP){
			Operator* op = _zbuffer->getOperator(); 
			_inputMode = INPUT_EDITING; 
			_holdAxis = -1; 
			_zbuffer->setOperator(op,OP_MODE_TRANSLATE); 
		}
	}
	else if(_inputMode==INPUT_ROT){
		if(_buttonSt==GLUT_UP){
			Operator* op = _zbuffer->getOperator(); 
			_inputMode = INPUT_EDITING; 
			_holdAxis = -1; 
			_zbuffer->setOperator(op,OP_MODE_ROTATE); 
		}
	}
	else if(_inputMode==INPUT_EDITING){
		if(_buttonSt==GLUT_DOWN)
			if(!_highlighted){
				if(button==GLUT_LEFT_BUTTON){
					if(_holdAxis>=0){
						if(!_holdShift){
							_inputMode = INPUT_TRANS; 
							handleAxisTrans(x,y,true); 
						}
						else{
							_inputMode = INPUT_ROT;
							handleAxisRot(x,y,true); 
						}
					}
					else{
						if(_holdAlt) handlePan(x,y,true); 
						else         handleRot(x,y,true); 
					}
				}
				else if(button==GLUT_RIGHT_BUTTON)
					handleZoom(x,y,true); 
			}
	}
	else if(_inputMode==INPUT_SELECTING){
		if(_buttonSt==GLUT_UP){
			if(button==GLUT_LEFT_BUTTON){
				_zbuffer->setSelected(_highlighted); 
				// this is an example of how to activate and deactive an operator
				if(_highlighted){
					_zbuffer->setOperator(_geom2op[_highlighted],OP_MODE_TRANSLATE); 
					_zbuffer->getOperator()->setState(OP_TRANSLATE); 
					_inputMode = INPUT_EDITING; 
					PropertyWindow::openPropertyWindow(_highlighted,_geom2op[_highlighted],_scene->getMaterial(_highlighted)); 
				}
				else{
					if(_zbuffer->getOperator()){
						_zbuffer->getOperator()->setState(OP_NONE);
						_zbuffer->setSelected(NULL); 
						_zbuffer->setOperator(NULL,0); 
						PropertyWindow::closePropertyWindow(); 
					}
				}

				_highlighted = NULL; 
			}
		}
	}
	_zbuffer->setHightlighted(_highlighted); 
}

// idle is a function that gets repeatedly called.  Here we test for various keyboard modifiers such as control or shift to 
// determine the state of the session (for example whether we are editing a shape or moving the camera, etc).  
void MainWindow::idle(){
	// here is where we hack holding down control
	bool pCtrl = _holdCtrl; 
	_holdCtrl = (glutGetModifiers() & GLUT_ACTIVE_CTRL)!=0; 

	if(_holdCtrl && !pCtrl){
		_inputMode=INPUT_SELECTING;
	}
	else if(!_holdCtrl && pCtrl){
		if(_inputMode==INPUT_SELECTING && _zbuffer->getSelected()==NULL){
			_inputMode=INPUT_VIEWING; 
		}
		else if(_zbuffer->getSelected()!=NULL){
			_inputMode=INPUT_EDITING; 
		}

		_highlighted = NULL; 
		_zbuffer->setHightlighted(_highlighted); 
	}

	_holdAlt = (glutGetModifiers() & GLUT_ACTIVE_ALT)!=0; 

	bool pShift = _holdShift; 
	_holdShift = (glutGetModifiers() & GLUT_ACTIVE_SHIFT)!=0; 

	if(_holdShift && !pShift){
		if(_inputMode==INPUT_EDITING){
			Operator* op = _zbuffer->getOperator(); 
			_zbuffer->setOperator(op,OP_MODE_ROTATE); 
			_zbuffer->getOperator()->setState(OP_ROTATE); 
		}
	}
	else if(!_holdShift && pShift){
		if(_inputMode==INPUT_EDITING){
			Operator* op = _zbuffer->getOperator(); 
			_zbuffer->setOperator(op,OP_MODE_TRANSLATE); 
			_zbuffer->getOperator()->setState(OP_TRANSLATE); 
			_holdAxis = -1; 
		}
	}
}

void MainWindow::specialKey(int key, int x, int y ){
	// press F5 to bring up the ray trace window
	if(key==GLUT_KEY_F5){
		if(_scene){
			GLdouble mv[16],proj[16];
			GLint viewport[4]={0,0,_rtviewer->w(),_rtviewer->h()}; 
		
			mLoadMatrix(*_scene->getModelview(),mv); 
			mLoadMatrix(_proj,proj); 

			_rtviewer->show(); 
			_rtviewer->trace(mv,proj,viewport); 
		}
	}
	else if(key==GLUT_KEY_F4){
		_rtviewer->hide(); 
	}
}

void MainWindow::escapeButtonCb(Fl_Widget* widget, void* win){ exit(0);}
void MainWindow::mouseEntered(int state){}

void MainWindow::normalKey( unsigned char key, int x, int y ){
	// Press control o to open file dialog
	if(_holdCtrl && key==15) // control 'o'
		openFileMenuCb(NULL,_singleton); 

	if(_holdCtrl && key==19) // control 's'
		saveFileMenuCb(NULL,_singleton); 

	int step = 15; 

	if(key>='A'&&key<='Z')
		key=key-'A'+'a';

	// e and q rotates the camera with respect to the forward looking axis
	if(key=='e' || key=='q'){
		Mat4 mat; 

		// using gl matrices out of laziness
		glMatrixMode(GL_MODELVIEW); 
		glPushMatrix(); 
		glLoadIdentity();
		if(key=='e')
			glRotatef(.5f,0,0,-1); 
		else
			glRotatef(-.5f,0,0,-1); 
		mglReadMatrix(GL_MODELVIEW_MATRIX,mat); 
		glPopMatrix(); 

		Mat4* rot = _scene->getRotate(); 
		(*rot) = (*rot) * mat; 
		updateModelView(); 
	}
	// pans left or right, 
	else if(key=='d' || key=='a'){
		Ray r0 = getMouseRay(_w/2,_h/2); 
		_panVec = r0.p + r0.dir*3.f; 

		Ray r1;
		if(key=='d')
			r1 = getMouseRay(_w/2-step,_h/2); 
		else
			r1 = getMouseRay(_w/2+step,_h/2); 

		Pt3 p = r1.p + r1.dir*3.f; 
		Vec3 v = (p-_panVec); 
		Mat4* trans = _scene->getTranslate(); 
		(*trans)[3][0]+=v[0]; 
		(*trans)[3][1]+=v[1]; 
		(*trans)[3][2]+=v[2]; 
		updateModelView(); 
	}
	// pans up or down
	else if(key=='w' || key=='s'){
		Ray r0 = getMouseRay(_w/2,_h/2); 
		_panVec = r0.p + r0.dir*3.f; 

		Ray r1;
		if(key=='w')
			r1 = getMouseRay(_w/2,_h/2+step); 
		else
			r1 = getMouseRay(_w/2,_h/2-step); 

		Pt3 p = r1.p + r1.dir*3.f; 
		Vec3 v = (p-_panVec); 
		Mat4* trans = _scene->getTranslate(); 
		(*trans)[3][0]+=v[0]; 
		(*trans)[3][1]+=v[1]; 
		(*trans)[3][2]+=v[2]; 
		updateModelView(); 
	}

}

