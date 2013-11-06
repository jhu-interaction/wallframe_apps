#ifndef _KBSkyBox_h
#define _KBSkyBox_h
// Includes //
#include "TexturedCube.h"

namespace modulair{

	class KBSkyBox : public TexturedCube
	{
	public:
		KBSkyBox(	double edgelength,
    		     	QList<osg::ref_ptr< osg::TextureRectangle > >* tex, 
    		     	int top, int bottom, int left, int back, int right, int front, osg::Vec3 center) 
    		     	: TexturedCube()
   	{
   		double d = edgelength/2;
	    savedScale = this->getScale();
	    _textures = tex;
	    //top
	    this->addChild(plane(top, Vec3(-d,d,-d),Vec3(-d,d,d),Vec3(d,d,-d),Vec3(d,d,d)));
	    //bottom
	    this->addChild(plane(bottom, Vec3(-d,-d,-d),Vec3(-d,-d,d),Vec3(d,-d,-d),Vec3(d,-d,d)));
	    //front
	    this->addChild(plane(front, Vec3(d,d,-d),Vec3(d,d,d),Vec3(d,-d,-d),Vec3(d,-d,d)));
	    //back
	    this->addChild(plane(back, Vec3(-d,d,d),Vec3(-d,d,-d),Vec3(-d,-d,d),Vec3(-d,-d,-d)));
	    //left
	    this->addChild(plane(left, Vec3(-d,d,-d),Vec3(d,d,-d),Vec3(-d,-d,-d),Vec3(d,-d,-d)));
	    //right
	    this->addChild(plane(right, Vec3(d,d,d),Vec3(-d,d,d),Vec3(d,-d,d),Vec3(-d,-d,d)));

	    this->setPos3DAbs(center);	
   	}
  	
		~KBSkyBox(){};

	protected:
	private:
		TextureList* _textures;

	};
}
#endif