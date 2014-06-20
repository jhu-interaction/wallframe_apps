#ifndef __WPainting_h_
#define __WPainting_h_

#include "Utils.h"
#include <OGRE/OgreTechnique.h>

class WPainting
{
public:
    WPainting(Ogre::SceneManager *sceneMgr, Ogre::String paintingName, int texture_path, double w, double h);
    ~WPainting(void);

    void CreateFrame(void);
    void SetMaterial(void);
    bool LoadImage(int ObjectID, Ogre::String texName);
    static void Initialize(std::string objectFileName);
    static std::string GetRandomFile(int folderName);    

    static bool LoadImg(int ObjectID, Ogre::String texName);

    void SetTexturePath(std::string path);
    void UnloadAll(void);
    void Redraw(void);

    inline void SetPosition(double x, double y, double z){ wallNode->setPosition(x, y, z); }

    inline std::string GetPaintingName(){ return textureName; }

    inline Ogre::SceneNode* GetNode(void){ return this->wallNode; }

    inline void SetTextureName(std::string name){ textureName.assign(name.c_str());}

protected:    


private:       
    Ogre::String meshName;
    Ogre::String textureName;    
    Ogre::String BufferedTextureName;
    Ogre::String texturePath;
    Ogre::SceneNode *wallNode;
    Ogre::SceneManager * mSceneMgr;    
    Utils *util;        
    double maxW;
    double maxH;

    Ogre::SceneNode *centerNode;
    Ogre::SceneNode *LSideWall;
    Ogre::SceneNode *RSideWall;
    Ogre::SceneNode *TSideWall;
    Ogre::SceneNode *BSideWall;

public:
    double width;
    double height;
    double thickness;
    static std::vector<int> objFiles;
    static std::vector<Ogre::String> objNames;
    static std::string PATH;
    Ogre::ManualObject* manual;
    Ogre::String materialName;

    bool NewPaintingLoaded;
};

#endif // #ifndef __WPainting_h_
