#include "Rendering/Raytracer.h" 
#include "Rendering/Shading.h" 
#include <FL/glu.h> 
#include "Common/Common.h" 
#include "Common/Matrix.h"

Raytracer::Raytracer(){
	_pixels = NULL; 
}

void Raytracer::drawInit(GLdouble modelview[16], GLdouble proj[16], GLint view[4]){
	_width = view[2]; 
	_height = view[3]; 

	if(_pixels)
		delete [] _pixels; 

	_pixels = new float[_width*_height*4]; 

	for(int j=0;j<16;j++){
		_modelview[j] = modelview[j]; 
		_proj[j] = proj[j]; 
	}

	for(int j=0;j<4;j++)
		_view[j] = view[j]; 

	Mat4 mv,pr;
	for(int j=0;j<16;j++){
		mv[j/4][j%4] = modelview[j]; 
		pr[j/4][j%4] = proj[j];
	}

	_final = mv*pr; 
	_invFinal = !_final; 

	_last = 0; 
}

Pt3 Raytracer::unproject(const Pt3& p){
	Pt3	np = p;
	np[0] = (p[0]-_view[0])/_view[2];
	np[1] = (p[1]-_view[1])/_view[3];

	np[0]=np[0]*2-1;
	np[1]=np[1]*2-1;
	np[2]=np[2]*2-1;

	Pt3 ret = np*_invFinal; 

	ret[0]/=ret[3]; 
	ret[1]/=ret[3];
	ret[2]/=ret[3];
	ret[3]=1;

	return ret;
}

bool Raytracer::draw(int step){
	int size = _width*_height; 

	int j=0;
	for(j=_last;j<size && j<_last+step;j++){
		int x = j%_width; 
		int y = j/_width; 
		
		Pt3 rst = unproject(Pt3(x,y,0)); 
		Pt3 red = unproject(Pt3(x,y,1));

		Ray r(rst,red-rst);
		r.dir.normalize(); 

		TraceResult res; 
		if(_scene)
			res = trace(r,0); 

		_pixels[(x + y*_width)*4] = res.color[0]; 
		_pixels[(x + y*_width)*4+1] = res.color[1]; 
		_pixels[(x + y*_width)*4+2] = res.color[2]; 
		_pixels[(x + y*_width)*4+3] = 1.f; 
	}

	_last = j; 
	return (_last>=size); 
}



TraceResult Raytracer::trace(const Ray& ray, int depth){
	// TODO: perform recursive tracing here
	// TODO : perform proper illumination, shadowing, etc... 
	_intersector.setRay(ray); 

	double bestt = FINF32;
	Material* bestMat = NULL; 
	Pt3 hitPoint;
	Vec3 normal;
	for(int j=0;j<_scene->getNumObjects();j++){
		IsectData data;
		Geometry* geom = _scene->getObject(j); 
		Material* mat = _scene->getMaterial(geom); 
		geom->accept(&_intersector,&data); 

		if(data.hit){
			if(data.t0<bestt){
				bestt = data.t0;
				bestMat = mat;
				hitPoint = ray.at(data.t0);
				normal = data.normal;
			}
		}
	}

	//cout<<"number of lights is "<<_scene->getNumLights()<<endl;
	TraceResult res; 
	res.color = Color(0.4f,0.4f,0.4f); 
	if(bestt < FINF32) {
		Vec3 R = -2*(ray.dir*normal)*normal + ray.dir;
		res.color = Color(0.0f,0.0f,0.0f);
		res.hit = true;
		if (!ray.inside){
			// ambient
			res.color += multiply(_scene->getLight(0)->getAmbient(),bestMat->getAmbient());
			// diffuse and specular
			for (int i=0; i<_scene->getNumLights(); i++){
				Vec3 L = _scene->getLight(i)->getPos()-hitPoint;
				Ray shadowRay(hitPoint, L);
				float scale = 1.0;
				for(int j=0;j<_scene->getNumObjects();j++){
					IsectData data;
					Geometry* geom = _scene->getObject(j); 
					Material* mat = _scene->getMaterial(geom); 
					geom->accept(&_intersector,&data); 
					if(data.hit){
						if(data.t0<1.0){
							scale *= mat->getTransparency();
						}
					}
				}
				L.normalize();
				if (scale>0.001){
					res.color += (L*normal)*scale*multiply(_scene->getLight(i)->getColor(),bestMat->getDiffuse());
					res.color += pow((L*R),bestMat->getSpecExponent())*scale*multiply(_scene->getLight(i)->getColor(),bestMat->getSpecular());
				}
			}
		}
		if (depth<2){
			// reflect
			R.normalize();
			Ray reflectRay(hitPoint, R);
			reflectRay.inside = ray.inside;
			TraceResult reflectResult = trace(reflectRay,depth+1);
			if (reflectResult.hit){
				res.color += bestMat->getReflective()*reflectResult.color;
			}
			// refract
			float c2c1 = (ray.inside ? bestMat->getRefractIndex() : 1.0/bestMat->getRefractIndex());
			float cos2 = 1 - c2c1*c2c1*(1-pow((normal*ray.dir),2));
			if (cos2>0){
				Vec3 W = (-c2c1*(normal*ray.dir) - sqrt(cos2))*normal + c2c1 * ray.dir;
				Ray refractRay(hitPoint, W);
				refractRay.inside = !ray.inside;
				TraceResult refractResult = trace(refractRay,depth+1);
				if (refractResult.hit){
					res.color += bestMat->getTransparency()*refractResult.color;
				}
			}
		}
	}
	return res; 
}