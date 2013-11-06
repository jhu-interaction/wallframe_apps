#ifndef _TiledPlane_h
#define _TiledPlane_h
// // Includes //
// #include "OSGObjectBase.h"
#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>
#include <osgText/Text>

namespace modulair{

	class TiledPlane;
	typedef QList<TiledPlane*> TiledPlaneList;
	typedef std::map<QString,TiledPlane*> TiledPlaneMap;

	class TiledPlane : public OSGObjectBase
	{
	public:
    TiledPlane(  osg::Vec3 ul, osg::Vec3 ur, 
                    osg::Vec3 ll, osg::Vec3 lr,
                    TexImageList* texImages, int texIndex, 
                    osg::Vec3 center, int x, int y);
		~TiledPlane(){};
		void initGeometry(int index);
		void applyNewTexture(int tindex);
		osg::Node* generatePlane(int index);
		void setTexture(int index);
		bool triggerBehavior(QString type);
		void setTransparency(double val);
		const osg::Vec3 ul(){return ul_;};
		const osg::Vec3 ur(){return ur_;};
		const osg::Vec3 ll(){return ll_;};
		const osg::Vec3 lr(){return lr_;};

		QString _id;
		QString _collection;
		int texIndex_;
		bool tiled;
		osg::Texture2D* texture_;
		int xdim, ydim;

	protected:
	private:
		osg::Vec3 ul_,ur_,ll_,lr_;
		TextureList* textures_;
		TexImageList* textureImages_;

	};
}
#endif