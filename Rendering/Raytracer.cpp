#include "Rendering/Raytracer.h" 
#include "Rendering/Shading.h" 
#include <FL/glu.h> 
#include "Common/Common.h" 
#include "Common/Matrix.h"

#define MAX_DEPTH 4

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
/**
 * @brief Perform Raytracing on every pixel on the screen
 * @details Coveret every pixel on the screen to the world coordinate 
 * and perform raytracing on it. To provent long tracing waiting time 
 * we seperate the total pixels into array with step length.
 * 	
 * @param step the lengh pixels that one function call need to perform.
 * @return wheather it finishs all the pixels on the screen
 */
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
/**
 * @brief Recursive trace one ray.
 * @details Ray trace scenes containing any number of light sources
 * and spheres. Implement the ray tracing illumination model and shadow 
 * effects, reflection, refaction, transparency for spheres.
 * 
 * @param ray the ray need to trace
 * @param depth the depth of funtion calls, start with 0
 * 
 * @return ray tracing results
 */
TraceResult Raytracer::trace(const Ray& ray, int depth){
	// TODO: perform recursive tracing here
	// TODO : perform proper illumination, shadowing, etc... 
	_intersector.setRay(ray); 

	double bestt = FINF32;
	Material* bestMat = NULL; 
	Pt3 hitPoint;
	Vec3 normal;
	//iterate all objects in the scene
	for(int j = 0; j<_scene->getNumObjects(); j++){
		IsectData data;
		Geometry* geom = _scene->getObject(j); 
		Material* mat = _scene->getMaterial(geom); 
		geom->accept(&_intersector, &data); 

		//find the first hit object
		if(data.hit){
			if(data.t0 < bestt  && data.t0 > 0.001){
				bestt = data.t0;
				bestMat = mat;
				hitPoint = ray.at(data.t0);
				normal = data.normal;
			}
		}
	}

	TraceResult res; 
	if (depth==0){
		res.color = Color(0.4f,0.4f,0.4f); 
	} else {
		res.color = Color(0.0f, 0.0f, 0.0f);
	}
	if(bestt < FINF32) {
		Vec3 reflectVec = -2*(ray.dir*normal)*normal + ray.dir;
		res.color = Color(0.0f,0.0f,0.0f);
		res.hit = true;
		if (!ray.inside){
			// ambient
			res.color += multiply(_scene->getLight(0)->getAmbient(),bestMat->getAmbient());
			// diffuse and specular
			// shadows casting algorithm
			for (int i=0; i<_scene->getNumLights(); i++){
				Vec3 lightVec = _scene->getLight(i)->getPos() - hitPoint;
				double dist2Source = sqrt(lightVec * lightVec);
				lightVec.normalize();
				Ray shadowRay(hitPoint, lightVec);
				float shadowFactor = 1.0f;
				_intersector.setRay(shadowRay);
				for(int j=0;j<_scene->getNumObjects();j++){
					IsectData data;
					Geometry* geom = _scene->getObject(j); 
					Material* mat = _scene->getMaterial(geom); 
					geom->accept(&_intersector,&data); 
					if(data.hit){
						// Test if the intersector is in the range of hitpoint and light source
						if(data.t0>0.001 && data.t0 < dist2Source){
							shadowFactor *= mat->getTransparency();
						}
					} 
				}
				lightVec.normalize();
				if (shadowFactor >= 0.001f){
					res.color += (lightVec * normal) * shadowFactor * multiply(_scene->getLight(i)->getColor(), bestMat->getDiffuse());
					if (lightVec * reflectVec >0){
						res.color += pow((lightVec * reflectVec), bestMat->getSpecExponent()) * 
								shadowFactor * multiply(_scene->getLight(i)->getColor(),bestMat->getSpecular());
					}
				}
			}
		} 
		if (depth < MAX_DEPTH){
			if ( ray.inside) {
				normal = -normal;
			}
			//reflect
			reflectVec.normalize();
			Ray reflectRay(hitPoint, reflectVec);
			reflectRay.inside = ray.inside;
			TraceResult reflectResult = trace(reflectRay, depth + 1);
			if (reflectResult.hit){
				res.color += bestMat->getReflective() * reflectResult.color;
			}
			// refract
			float c2c1 = (ray.inside ? bestMat->getRefractIndex() : 1.0 / bestMat->getRefractIndex());
			float cos2 = 1 - c2c1 * c2c1 * (1 - pow((normal * ray.dir), 2));
			//Check for total reflection
			if (cos2 > 0){
				Vec3 W = (-c2c1 * (normal * ray.dir) - sqrt(cos2)) * normal + c2c1 * ray.dir;
				Ray refractRay(hitPoint, W);
				refractRay.inside = !ray.inside;
				TraceResult refractResult = trace(refractRay, depth + 1);
				if (refractResult.hit){
					res.color += bestMat->getTransparency() * refractResult.color;
				}
			}
		}
	}
	return res; 
}