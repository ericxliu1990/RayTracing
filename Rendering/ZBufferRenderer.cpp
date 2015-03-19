#include "Rendering/ZBufferRenderer.h" 
#include "Rendering/Shading.h" 
#include "GUI/MainWindow.h" 

ZBufferRenderer::ZBufferRenderer(){
	_visitor = new ZBufferVisitor(); 
	_glVisitor = new GLSceneObjectVisitor(); 

	_highlighted = NULL; 
	_selected = NULL; 
	_op = NULL; 
	_drawGrid = true; 
}

// Do various set up here
void ZBufferRenderer::initScene(){
	_highlighted = NULL; 
	_selected = NULL; 
	_op = NULL; 

	glEnable(GL_LINE_SMOOTH); 
	glEnable(GL_POINT_SMOOTH); 
	glEnable(GL_POLYGON_SMOOTH); 
}

void ZBufferRenderer::draw() { 
	glEnable(GL_LIGHTING); 
	for(int j=0;j<_scene->getNumLights();j++)
		_scene->getLight(j)->accept(_glVisitor,NULL);

	if(_drawGrid){
		drawGrid(); 
		glRotatef(90,1,0,0); 
		drawGrid(); 
		glRotatef(-90,1,0,0); 
	}

	glDisable(GL_COLOR_MATERIAL); 
	for(int j=0;j<_scene->getNumObjects();j++){
		if(_scene->getObject(j)!=_selected && _scene->getObject(j)!=_highlighted){
			Geometry* geom = _scene->getObject(j); 
			Material* mat = _scene->getMaterial(geom); 
			_glVisitor->setAlpha(1.f); 
			mat->accept(_glVisitor,NULL); 
			geom->accept(_visitor,NULL); 
		}
	}

	if(_highlighted){
		Material* mat = _scene->getMaterial(_highlighted); 
		mat->accept(_glVisitor,NULL); 

		glEnable(GL_CULL_FACE); 
		glCullFace(GL_BACK); 
		_highlighted->accept(_visitor,NULL); 

		glDisable(GL_LIGHTING); 
		glEnable(GL_COLOR_MATERIAL); 
		glPolygonOffset(1,1); 
		glColor4f(1,0,0,.3f); 
		glLineWidth(6.f); 
		glCullFace(GL_FRONT); 
		glPolygonMode(GL_FRONT_AND_BACK,GL_LINE); 
		_highlighted->accept(_visitor,NULL); 
		glPolygonMode(GL_FRONT_AND_BACK,GL_FILL); 
		glPolygonOffset(0,0); 
		glLineWidth(1.f); 
		glCullFace(GL_BACK); 
		glEnable(GL_LIGHTING); 
	}

	if(_op){
		glEnable(GL_COLOR_MATERIAL); 
		_op->accept(_visitor,NULL); 
	}

	if(_selected){
		glEnable(GL_COLOR_MATERIAL); 
		glEnable(GL_BLEND); 
		glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
		Material* mat = _scene->getMaterial(_selected); 
		Color diffuse = mat->getDiffuse(); 
		glColor4f(diffuse[0],diffuse[1],diffuse[2],.8f); 
		_selected->accept(_visitor,NULL); 
		glDisable(GL_BLEND); 
	}
}

void ZBufferRenderer::drawGrid(){
	float stx=-3.f, edx=3.f; 
	float sty=-3.f, edy=3.f; 

	int steps=40; 
	float stepx = (edx-stx)/steps; 
	float stepy = (edy-sty)/steps; 

	glDisable(GL_LIGHTING); 
	glColor3f(0.3f,0.3f,0.3f);
	glBegin(GL_LINES); 
	for(int j=0;j<=steps;j++){
		float x = stx + j*stepx; 
		glVertex3f(x,sty,0.f); 
		glVertex3f(x,edy,0.f); 
	}
	for(int j=0;j<=steps;j++){
		float y = sty + j*stepy; 
		glVertex3f(stx,y,0.f); 
		glVertex3f(edx,y,0.f); 
	}
	glEnd(); 
	glEnable(GL_LIGHTING); 
}

void ZBufferRenderer::setSelected(Geometry* geom){
	_selected = geom; 
}

void ZBufferRenderer::setHightlighted(Geometry* geom){
	_highlighted = geom; 
}

void ZBufferRenderer::setOperator(Operator* op, int mode){
	_op = op; 
	if(_op!=NULL)
		_visitor->setOpMode(mode); 
}

int ZBufferRenderer::getOperatorMode(){
	if(_visitor!=NULL)
		return _visitor->getOpMode(); 
	return -1; 
}

ZBufferVisitor::ZBufferVisitor(){
	_quadric = gluNewQuadric(); 
}

void ZBufferVisitor::visit(Sphere* sphere, void* ret){
	glMatrixMode(GL_MODELVIEW); 
	glPushMatrix(); 
	Pt3 c = sphere->getCenter(); 
	glTranslatef(c[0],c[1],c[2]); 
	float r = sphere->getRadius(); 
	gluSphere(_quadric,r,50,50); 
	glPopMatrix(); 
}


void ZBufferVisitor::visit(Box* op, void* ret){
	glMatrixMode(GL_MODELVIEW); 
	glPushMatrix(); 
	glMultMatrixf(op->getGLForwardMat()); 

	glBegin(GL_QUADS); 
	glNormal3f(0,0,-1); 
	glVertex3f(0,0,0); 
	glVertex3f(0,1,0); 
	glVertex3f(1,1,0); 
	glVertex3f(1,0,0); 

	glNormal3f(0,0,1); 
	glVertex3f(0,0,1); 
	glVertex3f(1,0,1); 
	glVertex3f(1,1,1); 
	glVertex3f(0,1,1); 

	glNormal3f(-1,0,0); 
	glVertex3f(0,0,0); 
	glVertex3f(0,0,1); 
	glVertex3f(0,1,1); 
	glVertex3f(0,1,0); 

	glNormal3f(1,0,0); 
	glVertex3f(1,0,0); 
	glVertex3f(1,1,0); 
	glVertex3f(1,1,1); 
	glVertex3f(1,0,1); 

	glNormal3f(0,-1,0); 
	glVertex3f(0,0,0); 
	glVertex3f(1,0,0); 
	glVertex3f(1,0,1); 
	glVertex3f(0,0,1); 

	glNormal3f(0,1,0); 
	glVertex3f(0,1,0); 
	glVertex3f(0,1,1); 
	glVertex3f(1,1,1); 
	glVertex3f(1,1,0); 

	glEnd(); 

	glPopMatrix(); 
}

// TODO: fill in this function
void ZBufferVisitor::visit(Ellipsoid* op, void* ret){
	cout<<"fill in ellipsoid function"<<endl;
}

// TODO: fill in this function
void ZBufferVisitor::visit(Cylinder* op, void* ret){
	cout<<"fill in cylinder function"<<endl;
}

// TODO: fill in this function
void ZBufferVisitor::visit(Cone* op, void* ret){
	cout<<"fill in cone function"<<endl;
}

void drawAxis(GLUquadric* quad){
	glPushMatrix(); 
	float step = OP_STEP; 
	gluCylinder(quad,.015,.015,step,20,20); 
	glTranslatef(0.f,0.f,step);
	gluCylinder(quad,.05,.0,step/2.5,20,20); 
	glPopMatrix(); 
}

void drawRing(GLUquadric* quad){
	glPushMatrix(); 
	float step = OP_STEP; 
	glutSolidTorus(.02,step,30,40);
	glPopMatrix(); 
}

void ZBufferVisitor::visit(Operator* op, void* ret){
	Pt3 center = op->getPrimaryOp()->getCenter(); 

	glDisable(GL_CULL_FACE); 
	glMatrixMode(GL_MODELVIEW); 
	glPushMatrix();
	glTranslatef(center[0],center[1],center[2]); 

	if(_opMode & OP_MODE_TRANSLATE){
		glColor3f(.7f,.7f,.7f); 
		gluSphere(_quadric,.04,20,20); 

		if(_opMode & OP_MODE_ZAXIS)
			glColor3f(1.f,1.f,0.f); 
		else
			glColor3f(0.f,0.f,1.f); 
		drawAxis(_quadric); 

		glRotatef(90.f,0.f,1.f,0.f);
		if(_opMode & OP_MODE_XAXIS)
			glColor3f(1.f,1.f,0.f); 
		else
			glColor3f(1.f,0.f,0.f); 
		drawAxis(_quadric); 
		glRotatef(-90.f,0.f,1.f,0.f);

		glRotatef(-90.f,1.f,0.f,0.f);
		if(_opMode & OP_MODE_YAXIS)
			glColor3f(1.f,1.f,0.0f); 
		else
			glColor3f(0.f,1.f,0.f); 
		drawAxis(_quadric); 
	}
	if(_opMode & OP_MODE_ROTATE){
		glColor3f(.7f,.7f,.7f); 
		gluSphere(_quadric,.04,20,20); 

		if(_opMode & OP_MODE_ZAXIS)
			glColor3f(1.f,1.f,0.f); 
		else
			glColor3f(0.f,0.f,1.f); 
		drawRing(_quadric); 

		glRotatef(90.f,0.f,1.f,0.f);
		if(_opMode & OP_MODE_XAXIS)
			glColor3f(1.f,1.f,0.f); 
		else
			glColor3f(1.f,0.f,0.f); 
		drawRing(_quadric); 
		glRotatef(-90.f,0.f,1.f,0.f);

		glRotatef(-90.f,1.f,0.f,0.f);
		if(_opMode & OP_MODE_YAXIS)
			glColor3f(1.f,1.f,0.0f); 
		else
			glColor3f(0.f,1.f,0.f); 
		drawRing(_quadric); 
	}


	glEnable(GL_CULL_FACE); 

	glPopMatrix();  
}

void GLSceneObjectVisitor::visit(Material* mat, void* ret){
	float amb[4], diff[4], spec[4]; 
	for(int j=0;j<3;j++){
		amb[j] = mat->getAmbient()[j]; 
		diff[j] = mat->getDiffuse()[j]; 
		spec[j] = mat->getSpecular()[j]; 
	}
	amb[3] = diff[3] = spec[3] = _alpha; 

	float shininess = (float)mat->getSpecExponent(); 
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, amb); 
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diff); 
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, spec); 
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess); 
}


void GLSceneObjectVisitor::visit(Light* light, void* ret){
	Mat4 mv = !(*MainWindow::getScene()->getModelview()); 	
	GLenum id = light->getId(); 

	float amb[4], diff[4], spec[4], pos[4]; 
	for(int j=0;j<3;j++){
		amb[j] = light->getAmbient()[j]; 
		diff[j] = light->getColor()[j]; 
		spec[j] = light->getColor()[j]; 
		pos[j] = light->getPos()[j]; 
	}
	amb[3] = diff[3] = spec[3] = pos[3] = 1.f; 

	glLightfv(id,GL_AMBIENT,amb); 
	glLightfv(id,GL_DIFFUSE,diff); 
	glLightfv(id,GL_SPECULAR,spec); 
	glLightfv(id,GL_POSITION,pos); 
	glEnable(id); 
}  

// Ignore the rest
void GLSceneObjectVisitor::visit(Sphere* sphere, void* ret){} 
void GLSceneObjectVisitor::visit(Ellipsoid* op, void* ret){}
void GLSceneObjectVisitor::visit(Box* op, void* ret){}
void GLSceneObjectVisitor::visit(Cylinder* op, void* ret){}
void GLSceneObjectVisitor::visit(Cone* op, void* ret){}
