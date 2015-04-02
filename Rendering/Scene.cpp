#include "Rendering/Scene.h" 
#include "Rendering/ShadeAndShapes.h" 
#include "Rendering/Shading.h" 
#include "Rendering/Geometry.h" 
#include <fstream> 
#include <sstream> 
#include <iostream> 
#include <algorithm>
#include <cstdlib>
#include <ctime>

#include <FL/gl.h>

using namespace std; 

Vec3 readVec3(istream* stream){
	Vec3 ret(0,0,0,0); 
	(*stream)>>ret[0]>>ret[1]>>ret[2]; 
	return ret; 
}

Pt3 readPt3(istream* stream){
	Pt3 ret(0,0,0); 
	(*stream)>>ret[0]>>ret[1]>>ret[2]; 
	return ret; 
}

float readFloat(istream* stream){
	float ret; 
	(*stream)>>ret; 
	return ret; 
}

int readInt(istream* stream){
	int ret; 
	(*stream)>>ret; 
	return ret; 
}

string readString(istream* stream){
	string ret; 
	(*stream)>>ret; 
	return ret; 
}

void writeVec3(ostream* stream, const Vec3& v){
	(*stream)<<v[0]<<" "<<v[1]<<" "<<v[2]; 
}

void writePt3(ostream* stream, const Pt3& v){
	(*stream)<<v[0]<<" "<<v[1]<<" "<<v[2]; 
}

void writeFloat(ostream* stream, float d){
	(*stream)<<d; 
}

void writeInt(ostream* stream, int d){
	(*stream)<<d;
}

void writeString(ostream* stream, const string& s){
	(*stream)<<s;
}

/**
 * @brief delect geometry object
 * @details delect the gemoetry object from the object vector and its material  
 * 
 * @param obj the gemotry object need to be deleted
 */
void Scene::deleteObject(Geometry* obj){
	int pos = find(_objs.begin(), _objs.end(), obj) - _objs.begin();
	if (pos < _objs.size())
	{
		cout << "find " << pos << endl;
		deleteMaterial(obj);
		_objs.erase(_objs.begin() + pos);
	}
}

/**
 * @brief add material to the object
 * @details attach material to the object
 * 
 * @param obj the object
 * @param mat the material
 */
void Scene::attachMaterial(Geometry* obj, Material* mat){
	_mats[obj] = mat; 
}

/**
 * @brief attach the default material to the object
 * @details attach the default material to the object
 * 
 * @param obj the object need to attach material
 */
void Scene::attachMaterial(Geometry* obj) { 
	
	Material* mat = new Material(); 
	std::srand(std::time(0));
	Pt3 aPoint(std::rand() / (float)RAND_MAX, std::rand() / (float)RAND_MAX, std::rand() / (float)RAND_MAX);
	mat->setAmbient(aPoint); 
	mat->setDiffuse(aPoint); 
	mat->setSpecular(Pt3(0.5, 0.5, 0.5)); 
	mat->setSpecExponent(2.0f); 
	mat->setReflective(0.5f); 
	mat->setTransparency(0.1f); 
	mat->setRefractIndex(1.6f); 
	_mats[obj] = mat; 
}

/**
 * @brief delete material
 * @details delete the material attached to the object
 * 
 * @param obj the object need to remove material
 */
void Scene::deleteMaterial(Geometry* obj){
	_mats.erase(obj);
}

bool SceneUtils::writeScene(const std::string& fname, Scene* scene){
	std::ofstream fout(fname.c_str()); 

	if(scene && fout.good()){
		int nlights = scene->getNumLights(); 
		if(nlights<1) return false; 

		WriteSceneObjectVisitor writer; 
		writer.setStream(&fout); 

		Color amb = scene->getLight(0)->getAmbient(); 
		writePt3(&fout,amb); 
		fout<<endl; // this is for reading purpose
		writeInt(&fout,nlights); 
		fout<<endl;

		for(int j=0;j<nlights;j++)
			writer.visit(scene->getLight(j),NULL); 
		fout<<endl;
		writeInt(&fout,scene->getNumObjects()); 
		fout<<endl<<endl;

		for(int j=0;j<scene->getNumObjects();j++){
			Geometry* geom = scene->getObject(j); 
			Material* material = scene->getMaterial(geom); 
			geom->accept(&writer,NULL); 
			material->accept(&writer,NULL); 
			fout<<endl;
		}
	}
	fout<<endl;

	Mat4* rot = scene->getRotate(); 
	Mat4* trans = scene->getTranslate(); 

	for(int j=0;j<4;j++){
		for(int k=0;k<4;k++){
			fout<<(*rot)[j][k]<<" "; 
		}
	}
	fout<<(*trans)[3][0]<<" "<<(*trans)[3][1]<<" "<<(*trans)[3][2]<<"  // camera description"<<endl;

	return false; 
}

// no error handling at all.  pretty unsafe.  please make sure your file is well-formed.
Scene* SceneUtils::readScene(const std::string& fname){
	Scene* ret = NULL; 

	std::fstream fin(fname.c_str()); 
	std::stringstream ss; 
	std::string line; 

	if(fin.good()){
		ret = new Scene(); 

		while(!fin.eof()){
			getline(fin,line);
			size_t pos = line.find("//");
			if(pos==string::npos)
				ss<<line;
			else
				ss<<line.substr(0,pos);
			ss<<endl;
		}

		ReadSceneObjectVisitor reader; 
		reader.setStream(&ss); 

		Color amb = readPt3(&ss); 

		int numLights = readInt(&ss); 
		for(int j=0;j<numLights;j++){
			Light* l = new Light(); 
			l->setId(GL_LIGHT0+j); 
			reader.visit(l,NULL); 
			l->setAmbient(amb); 
			ret->addLight(l); 
		}

		int numObjs = readInt(&ss); 
		for(int j=0;j<numObjs;j++){
			string type = readString(&ss); 
			Geometry* geom = NULL; 
			if(type=="sphere")
				geom = new Sphere(); 
			else if(type=="ellipsoid")
				geom = new Ellipsoid(); 
			else if(type=="box")
				geom = new Box(); 
			else if(type=="cylinder")
				geom = new Cylinder(); 
			else if(type=="cone")
				geom = new Cone(); 

			geom->accept(&reader,NULL); 

			Material* mat = new Material(); 
			mat->accept(&reader,NULL); 

			ret->addObject(geom); 
			ret->attachMaterial(geom,mat); 
		}

		Mat4* rot = ret->getRotate(); 
		Mat4* trans = ret->getTranslate(); 

		for(int j=0;j<4;j++)
			for(int k=0;k<4;k++)
				(*rot)[j][k] = readFloat(&ss); 				

		for(int j=0;j<3;j++)
			(*trans)[3][j] = readFloat(&ss); 
	}

	return ret; 
}

void ReadSceneObjectVisitor::visit(Material* mat, void* ret){
	mat->setAmbient(readPt3(_stream)); 
	mat->setDiffuse(readPt3(_stream)); 
	mat->setSpecular(readPt3(_stream)); 
	mat->setSpecExponent(readFloat(_stream)); 
	mat->setReflective(readFloat(_stream)); 
	mat->setTransparency(readFloat(_stream)); 
	mat->setRefractIndex(readFloat(_stream)); 
}

void ReadSceneObjectVisitor::visit(Light* light, void* ret){
	Pt3 pos = readPt3(_stream); 
	Color color = readPt3(_stream); 
	light->setPos(pos); 
	light->setColor(color); 
}

void ReadSceneObjectVisitor::visit(Sphere* sphere, void* ret){
	sphere->setCenter(readPt3(_stream)); 
	sphere->setRadius(readFloat(_stream)); 
}

// TODO: need to properly fill this out
void ReadSceneObjectVisitor::visit(Ellipsoid* op, void* ret){
	op->setCenter(readPt3(_stream)); //corner
	op->setAxis1(readVec3(_stream)); //v1
	op->setAxis2(readVec3(_stream)); //v2
	op->setAxis3(readVec3(_stream)); //v3
	op->setLen1(readFloat(_stream)); //l1
	op->setLen2(readFloat(_stream)); //l2
	op->setLen3(readFloat(_stream)); //l3
	op->updateTransform(); 
}

// TODO: need to properly fill this out (if you want)
void ReadSceneObjectVisitor::visit(Box* op, void* ret){
	op->setCorner(readPt3(_stream)); //corner
	op->setLengthVec(readVec3(_stream)); //v1
	op->setWidthVec(readVec3(_stream)); //v2
	op->setHeightVec(readVec3(_stream)); //v3
	op->setLength(readFloat(_stream)); //l1
	op->setWidth(readFloat(_stream)); //l2
	op->setHeight(readFloat(_stream)); //l3
	op->updateTransform(); 
}

// TODO: need to properly fill this out
void ReadSceneObjectVisitor::visit(Cylinder* op, void* ret){
	op->setCenter(readPt3(_stream)); //corner
	op->setAxis1(readVec3(_stream)); //v1
	op->setAxis2(readVec3(_stream)); //v2
	op->setAxis3(readVec3(_stream)); //v3
	op->setLen1(readFloat(_stream)); //l1
	op->setLen2(readFloat(_stream)); //l2
	op->setLen3(readFloat(_stream)); //l3
	op->updateTransform(); 
}

// TODO: need to properly fill this out
void ReadSceneObjectVisitor::visit(Cone* op, void* ret){
	op->setCenter(readPt3(_stream)); //corner
	op->setAxis1(readVec3(_stream)); //v1
	op->setAxis2(readVec3(_stream)); //v2
	op->setAxis3(readVec3(_stream)); //v3
	op->setLen1(readFloat(_stream)); //l1
	op->setLen2(readFloat(_stream)); //l2
	op->setLen3(readFloat(_stream)); //l3
	op->updateTransform(); 
}

void WriteSceneObjectVisitor::visit(Material* mat, void* ret){
	writePt3(_stream,mat->getAmbient()); 
	(*_stream)<<endl;
	writePt3(_stream,mat->getDiffuse()); 
	(*_stream)<<endl;
	writePt3(_stream,mat->getSpecular()); 
	(*_stream)<<endl;
	writeFloat(_stream,mat->getSpecExponent()); 
	(*_stream)<<endl;
	writeFloat(_stream,mat->getReflective()); 
	(*_stream)<<endl;
	writeFloat(_stream,mat->getTransparency()); 
	(*_stream)<<endl;
	writeFloat(_stream,mat->getRefractIndex()); 
	(*_stream)<<endl;
}

void WriteSceneObjectVisitor::visit(Light* light, void* ret){
	writePt3(_stream,light->getPos()); 
	(*_stream)<<endl;
	writePt3(_stream,light->getColor()); 
	(*_stream)<<endl;
}

void WriteSceneObjectVisitor::visit(Sphere* sphere, void* ret){
	writeString(_stream,"sphere"); (*_stream)<<endl;
	writePt3(_stream,sphere->getCenter()); 
	(*_stream)<<endl;
	writeFloat(_stream,sphere->getRadius()); 
	(*_stream)<<endl;
}

void WriteSceneObjectVisitor::visit(Box* op, void* ret){
	writeString(_stream,"box"); (*_stream)<<endl;
	writePt3(_stream,op->getCorner()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getLengthVec()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getWidthVec()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getHeightVec()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLength()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getWidth()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getHeight()); 
	(*_stream)<<endl;
}

// TODO: fill in all the functions below
void WriteSceneObjectVisitor::visit(Ellipsoid* op, void* ret){
	writeString(_stream,"ellipsoid"); (*_stream)<<endl;
	writePt3(_stream,op->getCenter()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis1()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis2()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis3()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen1()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen2()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen3()); 
	(*_stream)<<endl;
}
void WriteSceneObjectVisitor::visit(Cylinder* op, void* ret){
	writeString(_stream,"cylinder"); (*_stream)<<endl;
	writePt3(_stream,op->getCenter()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis1()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis2()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis3()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen1()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen2()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen3()); 
	(*_stream)<<endl;

}
void WriteSceneObjectVisitor::visit(Cone* op, void* ret){
	writeString(_stream,"cone"); (*_stream)<<endl;
	writePt3(_stream,op->getCenter()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis1()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis2()); 
	(*_stream)<<endl;
	writeVec3(_stream,op->getAxis3()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen1()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen2()); 
	(*_stream)<<endl;
	writeFloat(_stream,op->getLen3()); 
	(*_stream)<<endl;
}

