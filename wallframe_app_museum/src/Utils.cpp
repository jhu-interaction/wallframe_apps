# include "Utils.h"

Utils::Utils(Ogre::SceneManager *sceneManager)
{
    mSceneMgr = sceneManager;
}

Ogre::Entity* Utils::createPlane(Ogre::String entName,Ogre::String planeName, Ogre::String matName, double sx,double sy, bool positive)
{
    Ogre::Plane p;
    if(positive)
        p.normal = Ogre::Vector3::UNIT_Z;
    else
        p.normal = Ogre::Vector3::NEGATIVE_UNIT_Z;

    p.d = 0;
    Ogre::MeshManager::getSingleton().createPlane(planeName,
                                                  Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
                                                  p,sx,sy,1,1,true,1,1,1,Ogre::Vector3::UNIT_Y);

    Ogre::Entity *ent = mSceneMgr->createEntity(entName, planeName);
    ent->setMaterialName(matName);

    return ent;
}
