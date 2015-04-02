#ifndef COMMON_H
#define COMMON_H

#include <Fl/Fl.H>
#include <string> 
#include <vector>
#include <sstream> 

#ifndef M_PI
#define M_PI 3.14159265359
#endif

#ifndef FINF32
#define FINF32 1000000000.f
#endif
#ifndef DEBUG_MODE
#define DEBUG_MODE true
#endif
using namespace std; 

const Fl_Color WIN_COLOR = fl_rgb_color(244,247,251); 

namespace Str{
	double parseDouble(const string& str); 
	double parseInt(const string& str); 
	string toString(int i); 
	string toString(double  d); 
	bool isLetter(char c); 
	bool isNumber(char c); 
	vector<string> split(const string& s, char c); 
}

void debugInfo(const string& str);
void debugInfo(const string& str, bool var);

#endif