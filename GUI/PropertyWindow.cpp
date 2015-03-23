#include "GUI/PropertyWindow.h"
#include <FL/Fl_Color_Chooser.h> 
#include <FL/Fl_Menu_Bar.H> 
#include "Common/Common.h" 
#include "GUI/Button.h"

PropertyWindow* PropertyWindow::_singleton = NULL; 
Operator* PropertyWindow::_op = NULL; 

#define PW_WIDTH 350
#define PW_HEIGHT 250
#define MW_HEIGHT 80

MaterialColorButton::MaterialColorButton(int x, int y, const char* name)
: Fl_Button(x,y,20,20, name){
	this->align(Fl_Align::FL_ALIGN_LEFT); 
	this->box(FL_BORDER_BOX); 
}

void MaterialColorButton::setColor(const Color& c){
	_color = c; 
	this->color(fl_rgb_color((uchar)(c[0]*255),(uchar)(c[1]*255),(uchar)(c[2]*255))); 
	this->redraw(); 
}

MaterialWindow::MaterialWindow(int x, int y)
: Fl_Window(x,y,PW_WIDTH-10,MW_HEIGHT){
	int bskip = 60; 
	int bwidth = 20; 
	int bspace = 5; 
	int sty = 5; 

	int iskip = 65; 
	int iwidth = 80; 
	int iheight = 20; 
	int ispace = 5; 

	begin(); 
	_diffuse = new MaterialColorButton(bskip, sty, "Diffuse"); 
	_specular = new MaterialColorButton(bskip*2 + bwidth + bspace, sty, "Specular"); 
	_ambient = new MaterialColorButton(bskip*3 + bwidth*2 + bspace*2, sty, "Ambient"); 

	_diffuse->callback(colorButton,this); 
	_specular->callback(colorButton,this); 
	_ambient->callback(colorButton,this); 

	sty+=25; 
	_specExp = new Fl_Float_Input(iskip,sty,iwidth,iheight,"SpecExp"); 
	_refl = new Fl_Float_Input(iskip*2 + iwidth + ispace,sty,iwidth,iheight,"Reflect"); 

	sty+=25; 
	_trans = new Fl_Float_Input(iskip,sty,iwidth,iheight,"Transp"); 
	_refr = new Fl_Float_Input(iskip*2 + (iwidth + ispace),sty,iwidth,iheight,"Refract"); 

	_specExp->callback(valueChanged,this); 
	_refl->callback(valueChanged,this); 
	_trans->callback(valueChanged,this); 
	_refr->callback(valueChanged,this); 

	_specExp->box(FL_BORDER_BOX); 
	_refl->box(FL_BORDER_BOX); 
	_trans->box(FL_BORDER_BOX);
	_refr->box(FL_BORDER_BOX); 


	end(); 

	box(FL_BORDER_BOX); 
	this->color(WIN_COLOR); 
}

void MaterialWindow::setMaterial(Material* mat){
	if(mat){
		_mat = mat; 
		_diffuse->setColor(_mat->getDiffuse()); 
		_specular->setColor(_mat->getSpecular()); 
		_ambient->setColor(_mat->getAmbient()); 
		_specExp->value(Str::toString(_mat->getSpecExponent()).c_str()); 
		_refl->value(Str::toString(_mat->getReflective()).c_str()); 
		_trans->value(Str::toString(_mat->getTransparency()).c_str()); 
		_refr->value(Str::toString(_mat->getRefractIndex()).c_str()); 
	}
}

void MaterialWindow::colorButton(Fl_Widget* widget, void* p){
	MaterialWindow* win = (MaterialWindow*) p; 

	MaterialColorButton* button = (MaterialColorButton*) widget; 
	Color color = button->getColor(); 
	double r = color[0]; 
	double g = color[1]; 
	double b = color[2]; 

	if(fl_color_chooser("Choose Triangle Color",r,g,b)){
		color = Color(r,g,b); 
		button->setColor(color); 

		if(button==win->_diffuse)
			win->_mat->setDiffuse(color); 
		else if(button==win->_specular)
			win->_mat->setSpecular(color); 
		else if(button==win->_ambient)
			win->_mat->setAmbient(color); 
	}
} 

void MaterialWindow::valueChanged(Fl_Widget* widget, void* p){
	MaterialWindow* win = (MaterialWindow*) p; 

	Fl_Float_Input* input = (Fl_Float_Input*) widget; 

	float nv = (float)atof(input->value()); 
	if(input==win->_specExp)
		win->_mat->setSpecExponent(nv); 
	else if(input==win->_refl)
		win->_mat->setReflective(nv); 
	else if(input==win->_trans)
		win->_mat->setTransparency(nv); 
	else if(input==win->_refr)
		win->_mat->setRefractIndex(nv); 
}


PropertyWindow::PropertyWindow() 
: Fl_Window(1400,200,PW_WIDTH,PW_HEIGHT,"Property"){
	begin(); 

	int sty=5; 
	{
		int stSpace= PW_WIDTH/8;
		int intSpace = stSpace;
		int compSpace = PW_WIDTH/5;
		int compH = 20; 

		_posX = new Fl_Float_Input(stSpace,sty,compSpace,compH,"Pos X"); 
		_posY = new Fl_Float_Input(stSpace+compSpace+intSpace, sty, compSpace,compH,"Pos Y"); 
		_posZ = new Fl_Float_Input(stSpace+2*(compSpace+intSpace),sty,compSpace, compH,"Pos Z"); 

		_posX->callback(handleAllCb,this); 
		_posY->callback(handleAllCb,this); 
		_posZ->callback(handleAllCb,this); 

		_posX->box(FL_BORDER_BOX); 
		_posY->box(FL_BORDER_BOX); 
		_posZ->box(FL_BORDER_BOX); 

		sty+=compH + 10; 
	}

	{
		int compSpace = PW_WIDTH/2;
		int compH = 20; 

		_radius = new Fl_Counter(compSpace/2,sty,compSpace,compH,"Radius"); 
		_radius->align(Fl_Align::FL_ALIGN_LEFT); 
		_radius->type(FL_SIMPLE_COUNTER); 
		_radius->step(.05); 
		_radius->color(WIN_COLOR); 

		_radius->callback(handleAllCb,this); 
		_radius->box(FL_BORDER_BOX); 

		sty+=compH +10; 
	}

	{
		int stSpace= PW_WIDTH/8;
		int intSpace = stSpace;
		int compSpace = PW_WIDTH/5;
		int compH = 20; 

		_axis0X = new Fl_Float_Input(stSpace,sty,compSpace,compH,"A0.X"); 
		_axis0Y = new Fl_Float_Input(stSpace+compSpace+intSpace, sty, compSpace,compH,"A0.Y"); 
		_axis0Z = new Fl_Float_Input(stSpace+2*(compSpace+intSpace),sty,compSpace, compH,"A0.Z"); 

		_axis0X->callback(handleAllCb,this); 
		_axis0Y->callback(handleAllCb,this); 
		_axis0Z->callback(handleAllCb,this); 

		_axis0X->box(FL_BORDER_BOX); 
		_axis0Y->box(FL_BORDER_BOX); 
		_axis0Z->box(FL_BORDER_BOX); 

		sty+=compH + 10; 
	}

	{
		int stSpace= PW_WIDTH/8;
		int intSpace = stSpace;
		int compSpace = PW_WIDTH/5;
		int compH = 20; 

		_axis1X = new Fl_Float_Input(stSpace,sty,compSpace,compH,"A1.X"); 
		_axis1Y = new Fl_Float_Input(stSpace+compSpace+intSpace, sty, compSpace,compH,"A1.Y"); 
		_axis1Z = new Fl_Float_Input(stSpace+2*(compSpace+intSpace),sty,compSpace, compH,"A1.Z"); 

		_axis1X->callback(handleAllCb,this); 
		_axis1Y->callback(handleAllCb,this); 
		_axis1Z->callback(handleAllCb,this); 

		_axis1X->box(FL_BORDER_BOX); 
		_axis1Y->box(FL_BORDER_BOX); 
		_axis1Z->box(FL_BORDER_BOX); 

		sty+=compH + 10; 
	}

	{
		int stSpace= PW_WIDTH/8;
		int intSpace = stSpace;
		int compSpace = PW_WIDTH/5;
		int compH = 20; 

		_axis2X = new Fl_Float_Input(stSpace,sty,compSpace,compH,"A2.X"); 
		_axis2Y = new Fl_Float_Input(stSpace+compSpace+intSpace, sty, compSpace,compH,"A2.Y"); 
		_axis2Z = new Fl_Float_Input(stSpace+2*(compSpace+intSpace),sty,compSpace, compH,"A2.Z"); 

		_axis2X->callback(handleAllCb,this); 
		_axis2Y->callback(handleAllCb,this); 
		_axis2Z->callback(handleAllCb,this); 

		_axis2X->box(FL_BORDER_BOX); 
		_axis2Y->box(FL_BORDER_BOX); 
		_axis2Z->box(FL_BORDER_BOX); 

		sty+=compH + 10; 
	}


	{
		int stSpace= PW_WIDTH/8;
		int intSpace = stSpace;
		int compSpace = PW_WIDTH/5;
		int compH = 20; 

		_lenX = new Fl_Float_Input(stSpace,sty,compSpace,compH,"L.X"); 
		_lenY = new Fl_Float_Input(stSpace+compSpace+intSpace, sty, compSpace,compH,"L.Y"); 
		_lenZ = new Fl_Float_Input(stSpace+2*(compSpace+intSpace),sty,compSpace, compH,"L.Z"); 

		_lenX->callback(handleAllCb,this); 
		_lenY->callback(handleAllCb,this); 
		_lenZ->callback(handleAllCb,this); 

		_lenX->box(FL_BORDER_BOX); 
		_lenY->box(FL_BORDER_BOX); 
		_lenZ->box(FL_BORDER_BOX); 

		sty+=compH + 10; 
	}

	{
		_matWindow = new MaterialWindow(5,sty); 
	}

	this->callback(escapeButtonCb,this); 

	end(); 

	_geom = NULL; 
	_geomUpdater = new GeometryUpdater(this); 
	_widgetUpdater = new WidgetUpdater(this); 

	this->color(WIN_COLOR); 
}

void PropertyWindow::openPropertyWindow(Geometry* geom, Operator* op, Material* mat){

	if(!_singleton)
		_singleton = new PropertyWindow(); 

	_singleton->show(); 
	if(geom && op){
		_op = op; 
		_singleton->setGeometry(geom); 
		_op->setSecondaryOp(_singleton); 
		_singleton->getGeometry()->accept(_singleton->getWidgetUpdater(),NULL); 
		_singleton->getMaterialWindow()->setMaterial(mat); 
	}
}

void PropertyWindow::draw() { 
	Fl_Window::draw(); 
}

Pt3 PropertyWindow::getCenter(){ return Pt3(); }

void PropertyWindow::translate(const Vec3& trans){
	if(_geom)
		_geom->accept(_widgetUpdater,NULL); 
}

void PropertyWindow::rotate(double d, int axis){
	if(_geom)
		_geom->accept(_widgetUpdater,NULL); 
}

void PropertyWindow::closePropertyWindow(){
	if(_singleton){
		_singleton->hide(); 
		if(_op){
			_op->setSecondaryOp(NULL); 
			_op = NULL; 
		}

		if(_singleton->getGeometry())
			_singleton->setGeometry(NULL); 
	}
}

void PropertyWindow::handleAllCb(Fl_Widget* widget, void* w){
	PropertyWindow* win = (PropertyWindow*) w; 
	win->getGeometry()->accept(win->getGeometryUpdater(),NULL); 
}

void PropertyWindow::escapeButtonCb(Fl_Widget* widget, void* win){
	closePropertyWindow(); 
}

void GeometryUpdater::visit(Sphere* sphere, void* ret){
	Fl_Float_Input* cx = _window->getPos(PROP_X); 
	Fl_Float_Input* cy = _window->getPos(PROP_Y); 
	Fl_Float_Input* cz = _window->getPos(PROP_Z); 
	Fl_Counter* r = _window->getRadius(); 

	Pt3 ncenter = Pt3(atof(cx->value()), atof(cy->value()), atof(cz->value())); 

	sphere->setCenter(ncenter); 
	sphere->setRadius(r->value()); 
} 

void GeometryUpdater::visit(Box* box, void* ret){
	Pt3 corner(0,0,0,1); 
	Vec3 vs[3]; 
	Vec3 ds; 

	int dirs[3]={PROP_X,PROP_Y,PROP_Z}; 
	for(int j=0;j<3;j++){
		Fl_Float_Input* c = _window->getPos(dirs[j]); 
		Fl_Float_Input* a0 = _window->getAxis0(dirs[j]); 
		Fl_Float_Input* a1 = _window->getAxis1(dirs[j]); 
		Fl_Float_Input* a2 = _window->getAxis2(dirs[j]); 
		Fl_Float_Input* len = _window->getLength(dirs[j]); 

		corner[j] = atof(c->value()); 
		vs[0][j] = atof(a0->value()); 
		vs[1][j] = atof(a1->value()); 
		vs[2][j] = atof(a2->value()); 
		ds[j] = atof(len->value()); 
	}
	vs[0][3]=vs[1][3]=vs[2][3]=0;
	corner[3]=1;

	box->setCorner(corner); 
	box->setLengthVec(vs[0]); 
	box->setWidthVec(vs[1]); 
	box->setHeightVec(vs[2]); 
	box->setLength(ds[0]); 
	box->setWidth(ds[1]); 
	box->setHeight(ds[2]); 
	box->updateTransform(); 
} 
// TODO: complete these functions
void GeometryUpdater::visit(Ellipsoid* op, void* ret){}
void GeometryUpdater::visit(Cylinder* op, void* ret){}
void GeometryUpdater::visit(Cone* op, void* ret){} 

// Here is the sphere example
void WidgetUpdater::visit(Sphere* sphere, void* ret){
	Pt3 center = sphere->getCenter(); 
	double radius = sphere->getRadius(); 

	int dirs[3]={PROP_X,PROP_Y,PROP_Z}; 

	for(int j=0;j<3;j++){
		Fl_Float_Input* c = _window->getPos(dirs[j]); 
		Fl_Float_Input* a0 = _window->getAxis0(dirs[j]); 
		Fl_Float_Input* a1 = _window->getAxis1(dirs[j]); 
		Fl_Float_Input* a2 = _window->getAxis2(dirs[j]); 
		Fl_Float_Input* len = _window->getLength(dirs[j]); 

		c->activate(); 
		c->value(Str::toString(center[j]).c_str()); 

		a0->deactivate(); 
		a1->deactivate(); 
		a2->deactivate(); 
		len->deactivate(); 
	}

	Fl_Counter* r = _window->getRadius(); 
	r->activate(); 
	r->value(radius); 
} 

// Here is the box example
void WidgetUpdater::visit(Box* box, void* ret){
	Pt3 corner = box->getCorner(); 
	Vec3 vs[3]; 
	vs[0] = box->getLengthVec(); 
	vs[1] = box->getWidthVec(); 
	vs[2] = box->getHeightVec(); 

	double ds[3] = {box->getLength(), box->getWidth(), box->getHeight()}; 

	int dirs[3]={PROP_X,PROP_Y,PROP_Z}; 

	for(int j=0;j<3;j++){
		Fl_Float_Input* c = _window->getPos(dirs[j]); 
		Fl_Float_Input* a0 = _window->getAxis0(dirs[j]); 
		Fl_Float_Input* a1 = _window->getAxis1(dirs[j]); 
		Fl_Float_Input* a2 = _window->getAxis2(dirs[j]); 
		Fl_Float_Input* len = _window->getLength(dirs[j]); 

		c->activate(); 
		a0->activate(); 
		a1->activate(); 
		a2->activate(); 
		len->activate(); 

		c->value(Str::toString(corner[j]).c_str()); 
		a0->value(Str::toString(vs[0][j]).c_str()); 
		a1->value(Str::toString(vs[1][j]).c_str()); 
		a2->value(Str::toString(vs[2][j]).c_str()); 
		len->value(Str::toString(ds[j]).c_str()); 
	}

	Fl_Counter* r = _window->getRadius(); 
	r->deactivate(); 
} 

// TODO: complete these functions
void WidgetUpdater::visit(Ellipsoid* op, void* ret){}
void WidgetUpdater::visit(Cylinder* op, void* ret){}
void WidgetUpdater::visit(Cone* op, void* ret){} 

