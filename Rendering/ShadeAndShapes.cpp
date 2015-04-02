#include "Common/Common.h" 
#include "Rendering/ShadeAndShapes.h" 


void Sphere::translate(const Vec3& trans){
	_center+=trans;
	updateTransform(); 
}

// rotate a sphere has no effect
void Sphere::rotate(double r, int axis){}

void Box::translate(const Vec3& trans){
	_corner+=trans;
	updateTransform(); 
}

void Box::rotate(double d, int axis){ 
	// the rotation code for the other objects should look very similar.
	// first construct a rotation about x/y/z axis

	Mat4 m; 
	switch(axis){
		case OP_XAXIS: 
			m[1][1] = cos(d); 
			m[1][2] = -sin(d); 
			m[2][1] = sin(d); 
			m[2][2] = cos(d); 
			break; 
		case OP_YAXIS: 
			m[0][0] = cos(d); 
			m[0][2] = sin(d); 
			m[2][0] = -sin(d); 
			m[2][2] = cos(d); 
			break; 
		case OP_ZAXIS: 
			m[0][0] = cos(d); 
			m[0][1] = -sin(d); 
			m[1][0] = sin(d); 
			m[1][1] = cos(d); 
			break; 
		default: 
			break; 
	}

	Vec3 cv = _corner - _center; 

	// rotate the three linearly independent vectors that determine the shape
	_lengthv = _lengthv*m;
	_widthv = _widthv*m;
	_heightv = _heightv*m;

	// rotation is about the center, so points that are not the center will also be rotated; 
	_corner = _center + (cv*m); 

	updateTransform(); 
}
// TODO: fill in these functions
void Cylinder::translate(const Vec3& trans) {
	_baseCenter+=trans;
	updateTransform();
}
void Cylinder::rotate(double d, int axis) {

}
void Cone::translate(const Vec3& trans) {
	_baseCenter+=trans;
	updateTransform();
}
void Cone::rotate(double d, int axis) {}
void Ellipsoid::translate(const Vec3& trans) {
	_center+=trans;
	updateTransform();
}
void Ellipsoid::rotate(double d, int axis) {}

// definition of the sphere can be pretty sparse.  
//You don't need to define the transform associated with a sphere.
void Sphere::updateTransform() {

}

/**
 * @brief calculate the first interction between a ray and a sphere
 * @details implement the algorithm in the book
 * 
 * @param sphere a sphere
 * @param ret the result struct of intersection
 */
void Intersector::visit(Sphere* sphere, void* ret){
	// TODO: rewrite this as described in the textbook
	IsectData* iret = (IsectData*) ret; 

	Pt3 center = sphere->getCenter(); 
	double radius = sphere->getRadius(); 

	double closest = GeometryUtils::pointRayClosest(center,_ray); 
    Vec3 C2P = _ray.at(closest) - center; 
    double r2 = radius*radius; 
	double d2 = C2P*C2P; 

	if(d2 > r2 + EPS){
        iret->hit = false; 
		iret->t0 = 0; 
	}
	else{
		double h = sqrt(r2 - d2); 
		if (closest+h<0){
			iret->hit = false;
			return;
		} else if (closest-h<0.001){
			iret->t0 = closest+h; 
			iret->hit = true;
		} else {
			iret->t0 = closest-h; 
			iret->hit = true; 
		}
		iret->normal = (_ray.at(iret->t0) - center); 
		iret->normal.normalize(); 
	}
}

// The updateTransform functions should properly set up the transformation of this geometry.
// The transformation takes a unit shape into the shape described by the parameters.
// This function also computes the inverse of the transformation.
void Box::updateTransform(){
	Vec3 ncenter = _corner; 
	ncenter += _length/2 * _lengthv; 
	ncenter += _width/2 * _widthv; 
	ncenter += _height/2 * _heightv; 
	_center = ncenter; 
	_mat = compose(_lengthv * _length, _widthv * _width, _heightv * _height, _center);
	_imat = !_mat; 

	Geometry::updateTransform(); // must call this at the end
}

// a box has six faces, which are basically six planes with rectangular boundaries.
// TODO: still need to return the normal at the intersection// still need to return the normal at the intersection
void Intersector::visit(Box* op, void* ret){

	IsectData* iret = (IsectData*) ret; 

	//if(true) {iret->hit=false; return;}

	Pt3 np = _ray.p*op->getInverseMat(); 
	Pt3 ppp = np*(1/np[3]); 
	Vec3 vvv = _ray.dir*op->getInverseMat(); 

	Ray ray(ppp,vvv); 

	Plane p(Pt3(.5,.5,0),Vec3(0,0,1,0)); 
	double d = GeometryUtils::planeRay(p,ray); 
	Pt3 pp = ray.at(d); 

	iret->hit = false; 
	iret->t0 = FINF32; 

	if(pp[0]>=0&&pp[0]<=1&&pp[1]>=0&&pp[1]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(0,0,-1,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}

	p = Plane(Pt3(.5,.5,1),Vec3(0,0,-1,0)); 
	d = GeometryUtils::planeRay(p,ray); 
	pp = ray.at(d); 

	if(pp[0]>=0&&pp[0]<=1&&pp[1]>=0&&pp[1]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(0,0,1,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}

	p = Plane(Pt3(.5,0,.5),Vec3(0,1,0,0)); 
	d = GeometryUtils::planeRay(p,ray); 
	pp = ray.at(d); 

	if(pp[0]>=0&&pp[0]<=1&&pp[2]>=0&&pp[2]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(0,-1,0,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}

	p = Plane(Pt3(.5,1,.5),Vec3(0,-1,0,0)); 
	d = GeometryUtils::planeRay(p,ray); 
	pp = ray.at(d); 

	if(pp[0]>=0&&pp[0]<=1&&pp[2]>=0&&pp[2]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(0,1,0,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}

	p = Plane(Pt3(0,.5,.5),Vec3(1,0,0,0)); 
	d = GeometryUtils::planeRay(p,ray); 
	pp = ray.at(d); 

	if(pp[1]>=0&&pp[1]<=1&&pp[2]>=0&&pp[2]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(-1,0,0,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}

	p = Plane(Pt3(1,.5,.5),Vec3(-1,0,0,0)); 
	d = GeometryUtils::planeRay(p,ray); 
	pp = ray.at(d); 

	if(pp[1]>=0&&pp[1]<=1&&pp[2]>=0&&pp[2]<=1){
		if (d>0.001 && iret->t0 > d){
			iret->t0 = d;
			iret->hit = true; 
			iret->normal = Vec3(1,0,0,0) * transpose(op->getInverseMat());
			iret->normal.normalize();
		}
	}
}

/**
 * @brief update the transform matrix of ellipsoid
 * @details [long description]
 */
void Ellipsoid::updateTransform() {
	_mat = compose(_axis1 * _len1, _axis2 * _len2, _axis3 * _len3, _center);
	_imat = !_mat;
	Geometry::updateTransform();
}
void Intersector::visit(Ellipsoid* op, void* ret){
	// TODO: rewrite this as described in the textbook
	IsectData* iret = (IsectData*) ret; 

	// Intersect transformed line with canonical sphere
	// Note that the vector of the trasformed line is normalized
	Pt3 np = _ray.p*op->getInverseMat(); 
	Pt3 ppp = np*(1/np[3]); 
	Vec3 vvv = _ray.dir*op->getInverseMat(); 
	double sizev = sqrt(vvv*vvv);
	vvv.normalize();
	Ray ray(ppp,vvv); 

	Pt3 center(0,0,0); 
	double closest = GeometryUtils::pointRayClosest(center,ray); 
    Vec3 C2P = ray.at(closest) - center; 
	double d2 = C2P*C2P; 

	if(d2 > 1 + EPS){
        iret->hit = false; 
		iret->t0 = 0; 
		return;
	}
	else{
		double h = sqrt(1 - d2); 
		if (closest+h<0){
			iret->hit = false;
			return;
		} else if (closest-h<0.001){
			iret->t0 = closest+h; 
			iret->hit = true;
		} else {
			iret->t0 = closest-h; 
			iret->hit = true; 
		}
		iret->normal = (ray.at(iret->t0) - center); 
		iret->normal.normalize(); 
	}

	// In reality, the hit point t0 is adjusted for the size of vvv
	iret->t0 /= sizev;
	// In reality, the normal is adjusted to be in the right frame
	iret->normal = iret->normal * transpose(op->getInverseMat());
	iret->normal.normalize();
}

void Cylinder::updateTransform() {
	_mat = compose(_baseAxis1 * _lenAxis1, _baseAxis2 * _lenAxis2, _centerAxis * _height, _baseCenter);
	_imat = !_mat;
	Geometry::updateTransform();
}
void Intersector::visit(Cylinder* op, void* ret){}

void Cone::updateTransform() {
	_mat = compose(_baseAxis1 * _lenAxis1, _baseAxis2 * _lenAxis2, _centerAxis * _height, _baseCenter);
	_imat = !_mat;
	Geometry::updateTransform();
}
void Intersector::visit(Cone* op, void* ret){

}

// The operator is the widget that allows you to translate and rotate a geometric object
// It is colored as red/green/blue.  When one of the axis is highlighted, it is yellow.
void Intersector::visit(Operator* op, void* ret){
	IsectAxisData* iret = (IsectAxisData*) ret; 
	Pt3 center = op->getPrimaryOp()->getCenter(); 

	iret->hit = false; 

	if(op->getState()==OP_TRANSLATE){
		Ray rx(center,op->getDirX()); 
		Ray ry(center,op->getDirY()); 
		Ray rz(center,op->getDirZ()); 

		double dx = GeometryUtils::pointRayDist(rx.p+rx.dir*.5,_ray); 
		double dy = GeometryUtils::pointRayDist(ry.p+ry.dir*.5,_ray); 
		double dz = GeometryUtils::pointRayDist(rz.p+rz.dir*.5,_ray); 

		double tol = .12f; 
		if(dx>tol && dy>tol && dz>tol) 
			iret->hit = false; 
		else{
			iret->hit = true; 
			if(dx<dy){
				if(dx<dz) iret->axis = OP_XAXIS; 
				else      iret->axis = OP_ZAXIS; 
			}
			else{
				if(dy<dz) iret->axis = OP_YAXIS; 
				else      iret->axis = OP_ZAXIS; 
			}
		}
	}
	else if(op->getState()==OP_ROTATE){
		Plane xy(center,Vec3(0,0,1,0)); 
		Plane xz(center,Vec3(0,1,0,0)); 
		Plane yz(center,Vec3(1,0,0,0)); 

		double dxy = GeometryUtils::planeRay(xy,_ray); 
		double dxz = GeometryUtils::planeRay(xz,_ray); 
		double dyz = GeometryUtils::planeRay(yz,_ray); 

		double txy = abs(mag(_ray.at(dxy)-center)-OP_STEP); 
		double txz = abs(mag(_ray.at(dxz)-center)-OP_STEP); 
		double tyz = abs(mag(_ray.at(dyz)-center)-OP_STEP); 

		double tol = .05f; 
		if(txy>tol && txz>tol && tyz>tol) 
			iret->hit = false; 
		else{
			iret->hit = true; 
			if(txy<txz){
				if(txy<tyz) iret->axis = OP_ZAXIS; 
				else      iret->axis = OP_XAXIS; 
			}
			else{
				if(txz<tyz) iret->axis = OP_YAXIS; 
				else      iret->axis = OP_XAXIS; 
			}
		}
	}
}
