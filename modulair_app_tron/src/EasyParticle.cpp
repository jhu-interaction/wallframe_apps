// #include "EasyParticle.h"
#include <modulair_app_tron/EasyParticle.h>
using namespace osg;
using namespace modulair;



EasyParticle::EasyParticle(osg::Vec3 center,QString path,osg::Vec3 colour, int type) : OSGObjectBase()
{
	
	
	initializeParticles(center,path,colour, type);
	wind = osg::Vec3(1.0f,0.0f,0.0f);
	


}

int EasyParticle::tick()
{
	duration--;
	if(duration==0)
	{
		for (int i = 0; i < system->numParticles(); i++)
			{
				system->destroyParticle(i);

			}	
			
			system->setFrozen(true);
	}
	

}
void EasyParticle::initializeParticles(osg::Vec3 center,QString path,osg::Vec3 colour, int type)
{

	_parGeode = new osg::Geode();
	if(type == P_EXPLODE)
	{
		duration = 5000;
    osgParticle::ExplosionEffect* explosion = new osgParticle::ExplosionEffect(center,10000.0f);
		//osgParticle::ExplosionDebrisEffect* explosionDebri = new osgParticle::ExplosionDebrisEffect(center,1000.0f);
		
		//osgParticle::FireEffect* fire = new osgParticle::FireEffect(center,1000.0f);

		explosion->setWind(wind);
		//explosionDebri->setWind(wind);
		
		// fire->setWind(wind);

		this->addChild(explosion);
		//this->addChild(explosionDebri);
		
		// this->addChild(fire);

		explosion->getParticleSystem()->setDefaultAttributes(path.toStdString(),true,false,0);
		explosion->getParticleSystem()->setDoublePassRendering(true);
		system = explosion->getParticleSystem();
		osgParticle::Particle ihatemylife = explosion->getParticleSystem()->getDefaultParticleTemplate();
		osgParticle::rangev4 param = osgParticle::rangev4(osg::Vec4(colour[0],colour[1],colour[2],0.0),osg::Vec4(colour[0],colour[1],colour[2],1.0));
		ihatemylife.setColorRange(param);
		explosion->getParticleSystem()->setDefaultParticleTemplate(ihatemylife);
		

		
		_parGeode->addDrawable(explosion->getParticleSystem());
		//_parGeode->addDrawable(explosionDebri->getParticleSystem());
		
		// _parGeode->addDrawable(fire->getParticleSystem());
	}
	else if(type==P_TAIL)
	{
		//std::cerr<<"Created new particle at "<< center[0] <<", "<<center[1]<<", "<<center[2]<<endl;;
		duration = 1000;
		osgParticle::ExplosionEffect* smoke = new osgParticle::ExplosionEffect(center,1.0f);
		smoke->setWind(wind);
		this->addChild(smoke);
		smoke->getParticleSystem()->setDefaultAttributes(path.toStdString(),true,false,0);
		smoke->getParticleSystem()->setDoublePassRendering(false);
		system = smoke->getParticleSystem();
		osgParticle::Particle ihatemylife = smoke->getParticleSystem()->getDefaultParticleTemplate();
		osgParticle::rangev4 param = osgParticle::rangev4(osg::Vec4(colour[0],colour[1],colour[2],0.0),osg::Vec4(colour[0],colour[1],colour[2],1.0));
		osgParticle::rangef param2 = osgParticle::rangef(5.0f,500.0f);
		ihatemylife.setColorRange(param);
		ihatemylife.setSizeRange(param2);
		smoke->getParticleSystem()->setDefaultParticleTemplate(ihatemylife);
		_parGeode->addDrawable(smoke->getParticleSystem());
	}
	this->addChild(_parGeode);
	this->setPos3DAbs(center);
}
