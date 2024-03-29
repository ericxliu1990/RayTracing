#include "Rendering/Geometry.h" 
#include "Common/Common.h"
#define _USE_MATH_DEFINES 
#include <cmath>

double GeometryUtils::pointRayClosest(const Pt3& pt, const Ray& ray){
    double t = (pt-ray.p)*ray.dir / (ray.dir*ray.dir); 
    return t; 
}

double GeometryUtils::pointRayDist(const Pt3& pt, const Ray& ray){
	float qdotv = (pt-ray.p)*ray.dir; 
    float t = (qdotv)/ (ray.dir*ray.dir); 
	Pt3 np = ray.at(t); 
	return mag(np-pt); 
}

double GeometryUtils::rayRayDist(const Ray& r1, const Ray& r2){
	Vec3 v12 = cross(r1.dir,r2.dir); 
	Vec3 p21 = (r2.p-r1.p);
	double num = fabs(p21*v12); 
	double denom = mag(v12); 
	if(denom==0){
		double tt = (p21*r1.dir); 
		return p21*p21 - tt*tt/(r1.dir*r1.dir); 
	}
	return num/denom; 
}

double GeometryUtils::planeRay(const Plane& pl, const Ray& r){
	Vec3 w = pl.p-r.p; 
	double dist = (pl.n*w);
	double denom = (pl.n*r.dir); 

	if(denom<1e-6 && denom>-1e-6)
		return -FINF32; 
	return dist/denom; 
}

double GeometryUtils::planeRayDeg(const Plane& pl, const Vec3& xa, const Ray& r){
	double t = planeRay(pl,r); 
	Pt3 p = r.at(t); 
	Vec3 v = p-pl.p; 
	v.normalize(); 

	Vec3 ya = cross(pl.n,xa); 

	double px = xa*v; 
	double py = ya*v; 

	double rr = atan2(py,px); 
	//return r<0 ? r+M_PI : r; 
	return rr<0 ? (rr+2*M_PI) : rr; 
}
std::auto_ptr<Mat4> GeometryUtils::getRotateMatrix(const double d, const int axis){
	
	std::auto_ptr<Mat4> m(new Mat4);
	switch(axis){
		case OP_XAXIS: 
			(*m)[1][1] = cos(d); 
			(*m)[1][2] = -sin(d); 
			(*m)[2][1] = sin(d); 
			(*m)[2][2] = cos(d); 
			break; 
		case OP_YAXIS: 
			(*m)[0][0] = cos(d); 
			(*m)[0][2] = sin(d); 
			(*m)[2][0] = -sin(d); 
			(*m)[2][2] = cos(d); 
			break; 
		case OP_ZAXIS: 
			(*m)[0][0] = cos(d); 
			(*m)[0][1] = -sin(d); 
			(*m)[1][0] = sin(d); 
			(*m)[1][1] = cos(d); 
			break; 
		default: 
			break; 
	}
	return m;
}
