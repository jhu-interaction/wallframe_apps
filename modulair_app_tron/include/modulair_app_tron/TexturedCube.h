#ifndef _TexturedCube_h
#define _TexturedCube_h
// Includes //
// #include "OSGObjectBase.h"
// using namespace osg;
// using namespace osg;		
// using namespace modulair;

#include <modulair_osg_tools/osg_object_base.h>


namespace modulair{

	class TexturedCube;
	typedef QList<TexturedCube*> TexCubeList;
	typedef std::map<QString,TexturedCube*> TexCubeMap;

	class TexturedCube : public OSGObjectBase
	{
	public:
		TexturedCube(	double edgelength, TextureList* tex, 
        		     	int texIndex, osg::Vec3 center);
    TexturedCube( double edgelength, TextureList* tex, 
                  int texIndex, osg::Vec3 center, QString wall);
    TexturedCube( double edgelength, TextureList* tex, int top, int bottom, 
    							int left, int back, int right, int front, osg::Vec3 center);
    TexturedCube( double edgelength, double height, TextureList* tex, int top, int bottom, 
                  int left, int back, int right, int front, osg::Vec3 center);
		~TexturedCube(){};
		void applyNewTexture(int tindex);
		osg::Node* plane(int index, 
                          osg::Vec3 tl, osg::Vec3 tr,
                          osg::Vec3 bl, osg::Vec3 br);
		void setTexture(int index);
		void setTransparency(double val);
		

		int _texIndex;

	protected:
	private:
		TextureList* _textures;

	};
}	
#endif