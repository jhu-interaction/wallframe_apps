#ifndef _TexturedPlane_h
#define _TexturedPlane_h
// Includes //
#include <modulair_osg_tools/osg_object_base.h>

#include <osgText/Text>

namespace modulair{

	class TexturedPlane;
	typedef QList<TexturedPlane*> TexPlaneList;
	typedef std::map<QString,TexturedPlane*> TexPlaneMap;

	class TexturedPlane : public OSGObjectBase
	{
	public:
		TexturedPlane(osg::Vec3 ul, osg::Vec3 ur, osg::Vec3 ll, osg::Vec3 lr,
                	TextureList* tex, int texIndex);
		TexturedPlane(osg::Vec3 ul, osg::Vec3 ur, osg::Vec3 ll, osg::Vec3 lr);
		~TexturedPlane();
		void initGeometry(int index);
		void createWithTexture(TextureList* tex, int texIndex);
		void applyNewTexture(int tindex);
		osg::Node* generatePlane(int index);
		void setTexture(int index);
		bool triggerBehavior(QString type);
		void setTransparency(double val);
    const osg::Vec3 ul(){return ul_;}
    const osg::Vec3 ur(){return ur_;}
    const osg::Vec3 ll(){return ll_;}
    const osg::Vec3 lr(){return lr_;}
    const osg::Plane getPlane(){return plane_;}
		

		QString _id;
		QString _collection;
		int _texIndex;

	protected:
	private:
		osg::Vec3 ul_,ur_,ll_,lr_,center_;
		TextureList* _textures;
		osg::Plane plane_;

	};
}
#endif
