#include "GUI/RaytraceViewer.h"

#include <FL/gl.h> 
#include <GL/glu.h>
#include <time.h>
#include <FL/Fl_File_Chooser.H> 

extern "C"{
#include "Common/bmpfile.h"
}

GLUquadric* gQuadric = NULL;

RaytraceViewer::RaytraceViewer(int x, int y, int w, int h, const char* l)
: Fl_Gl_Window(x,y,w,h,l){
	_tracer = new Raytracer(); 
}

RaytraceViewer::~RaytraceViewer(){
	delete _tracer; 
}

void RaytraceViewer::trace(GLdouble modelview[16], GLdouble proj[16], GLint view[4]){
	_tracer->drawInit(modelview,proj,view); 

	make_current(); 

	int stepSize = 6000*5; 

	clock_t begin=clock();

	while(!_tracer->draw(stepSize))
		draw(); 
	draw(); 

	clock_t end=clock();
	cout<<"time: "<<(end-begin)/(1.*CLOCKS_PER_SEC)<<" secs"<<endl;

}

void RaytraceViewer::draw(){
	if(!valid())
		init();

	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT); 
	float* pixels = _tracer->getPixels();
	glRasterPos2f(-1.,-1.);  
	if(pixels)
		glDrawPixels(_tracer->getWidth(), _tracer->getHeight(), GL_RGBA, GL_FLOAT, pixels); 
	swap_buffers(); 
}

int RaytraceViewer::handle(int ev){
	if(ev==FL_KEYUP){
		if(Fl::event_key()=='s' && Fl::event_ctrl()){
			char* newfile = fl_file_chooser("Save Image", ".bmp (*.bmp)", "./images", 0);

			if(!newfile) return 0; 

			int h = _tracer->getHeight(); 
			int w = _tracer->getWidth(); 
			float* pixels = _tracer->getPixels();

			bmpfile_t* bfile = bmp_create(w,h,32); 
			for(int j=0;j<h;j++){
				for(int i=0;i<w;i++){
					rgb_pixel_t pix = {
						(unsigned char)(pixels[(j*w+i)*4+2]*255),
						(unsigned char)(pixels[(j*w+i)*4+1]*255),
						(unsigned char)(pixels[(j*w+i)*4]*255),
						(unsigned char)(pixels[(j*w+i)*4+3]*255)}; 
					bmp_set_pixel(bfile,i,(h-1)-j,pix); 
				}
			}
			bmp_save(bfile,newfile); 
			bmp_destroy(bfile); 

			return 1; 
		}
	}

	return Fl_Gl_Window::handle(ev);
}

void RaytraceViewer::init(){
	glClearColor(1,1,1,1); 
	glMatrixMode(GL_PROJECTION); 
	glOrtho(0,this->w(),0,this->h(),-100,100); 
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity(); 

	glMatrixMode(GL_PROJECTION); 
	glLoadIdentity(); 
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity(); 

}

void RaytraceViewer::resize(int x, int y, int width, int height){
	make_current();
	Fl_Gl_Window::resize(x,y,width,height);
}