#ifndef __Utils_h_
#define __Utils_h_

#include <OGRE/Ogre.h>

class Utils
{
public:
    Utils(Ogre::SceneManager* sceneManager);

    ~Utils(){};

    Ogre::Entity* createPlane(Ogre::String entName, Ogre::String planeName, Ogre::String matName,double sx,double sy, bool positive = true);
    Ogre::SceneManager* mSceneMgr;

private:
};



#endif // #ifndef __Utils_h_
