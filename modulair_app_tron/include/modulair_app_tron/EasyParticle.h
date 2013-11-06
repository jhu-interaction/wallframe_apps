#ifndef _easyParticle_h
#define _easyParticle_h
// Includes //
// #include "OSGObjectBase.h"
#include <modulair_osg_tools/osg_object_base.h>


#include <osgParticle/ExplosionEffect>
#include <osgParticle/ExplosionDebrisEffect>
#include <osgParticle/SmokeEffect>
#include <osgParticle/SmokeTrailEffect>
#include <osgParticle/FireEffect>

#define P_EXPLODE 0
#define P_TAIL 1

// using nam		espace modulair;

namespace modulair{
	class EasyParticle;
	typedef QList<EasyParticle*> ParticleList;
	class EasyParticle : public OSGObjectBase
	{
	public:
		EasyParticle(osg::Vec3 center, QString path, osg::Vec3 colour, int type);
		~EasyParticle(){};
		osg::Vec3 wind;
		int duration;
		int tick();
		osgParticle::ParticleSystem* system;
	protected:
		void initializeParticles(osg::Vec3 center,QString path,osg::Vec3 colour, int type);
	private:
		osg::Geode* _parGeode;
		

		
	};
}
#endif