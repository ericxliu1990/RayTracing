#include "GUI/MainWindow.h" 
#include <FL/Fl.H>
#include <iostream>
using namespace std; 

int main(int argc, char** argv) { 
	MainWindow m(200,100,600,600,"Raytracer Project"); 
	return Fl::run(); 
}