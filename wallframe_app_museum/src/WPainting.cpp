#include "WPainting.h"
#include <dirent.h>
#include <boost/lexical_cast.hpp>
#include <boost/filesystem.hpp>

std::vector<int> WPainting::objFiles;
std::vector<Ogre::String> WPainting::objNames;
std::string WPainting::PATH = "/home/kel/dev/libMuseum/waltersApi/scripts/Images_Id/";

void WPainting::Initialize(std::string objectFileName)
{
    std::ifstream inFile(objectFileName.c_str(), std::ios::binary);
    if(inFile.is_open())
    {
        int a;
        while (inFile >> a)
        {
            WPainting::objFiles.push_back(a);
            Ogre::String name = WPainting::GetRandomFile(a);
            WPainting::objNames.push_back(name);
        }
    }
    else
    {
        std::cerr << "File cannot be opened" << std::endl;
        exit(1);
    }

}

std::string WPainting::GetRandomFile(int folderName)
{
    std::stringstream ss;
    ss << WPainting::PATH << folderName;
    boost::filesystem::directory_iterator iterator(ss.str());
    std::string fileName;

    fileName = iterator->path().filename();        // For 4.7 on the Wall
//    fileName = iterator->path().filename().string()  // For 4.8 on My PC

    return fileName;
}

WPainting::WPainting(Ogre::SceneManager* sceneMgr, Ogre::String paintingName, int ObjectID, double w, double h)
{    
    mSceneMgr = sceneMgr;
    maxW = w;
    maxH  = h;
    textureName = paintingName;
    thickness = 10;
    util = new Utils(mSceneMgr);

    wallNode = mSceneMgr->createSceneNode(textureName + "_Node");

    if(LoadImage(ObjectID, textureName))
        NewPaintingLoaded = true;
    else
        NewPaintingLoaded = false;

    Redraw();
}

WPainting::~WPainting(void)
{
    centerNode->detachAllObjects();
    centerNode->removeAndDestroyAllChildren();

    LSideWall->detachAllObjects();
    LSideWall->removeAndDestroyAllChildren();

    RSideWall->detachAllObjects();
    RSideWall->removeAndDestroyAllChildren();

    TSideWall->detachAllObjects();
    TSideWall->removeAndDestroyAllChildren();

    BSideWall->detachAllObjects();
    BSideWall->removeAndDestroyAllChildren();

    wallNode->detachAllObjects();
    wallNode->removeAndDestroyAllChildren();

}

void WPainting::SetMaterial(void)
{        
    std::stringstream ss;
    ss  << textureName << "_material";
    materialName = ss.str();

    Ogre::MaterialPtr mMat = Ogre::MaterialManager::getSingleton().create(materialName, "General");
    Ogre::Pass* mPass = mMat->getTechnique(0)->getPass(0);
    mPass->setCullingMode(Ogre::CULL_NONE);

    Ogre::TexturePtr mTex = Ogre::TextureManager::getSingleton().load(textureName, "General");
    mPass->createTextureUnitState()->setTextureName(mTex->getName());
}

void WPainting::CreateFrame(void)
{    
    manual= mSceneMgr->createManualObject();
    manual->begin(materialName,Ogre::RenderOperation::OT_TRIANGLE_LIST );

    manual->position(-width/2, height/2, 0.0);
    manual->textureCoord(0,0);

    manual->position(-width/2, -height/2, 0.0);
    manual->textureCoord(0,1);

    manual->position(width/2, -height/2, 0.0);
    manual->textureCoord(1,1);

    manual->position(width/2, height/2, 0.0);
    manual->textureCoord(1,0);

    manual->index(0);
    manual->index(1);
    manual->index(2);
    manual->index(0);
    manual->index(2);
    manual->index(3);

    manual->end();
    meshName = textureName + "CenterQuad";
    manual->convertToMesh(meshName);
    mSceneMgr->destroyManualObject(manual);

    centerNode = wallNode->createChildSceneNode(textureName + "CenterNode");
    Ogre::Entity * ent = mSceneMgr->createEntity(meshName);
    centerNode->attachObject(ent);
    centerNode->setPosition(0,0,thickness);

    Ogre::String LPlaneName = textureName + "LSideWallPlane";
    Ogre::String LEntName = textureName + "LSideWallEnt";
    LSideWall = wallNode->createChildSceneNode(textureName + "LSideWallNode");
    LSideWall->setPosition(-width/2,0,thickness/2);
    LSideWall->yaw(Ogre::Degree(90));
    LSideWall->attachObject(util->createPlane(LEntName, LPlaneName,"Ogre/Skin", thickness,height));    

    Ogre::String RPlaneName = textureName + "RSideWallPlane";
    Ogre::String REntName = textureName + "RSideWallEnt";
    RSideWall = wallNode->createChildSceneNode(textureName + "RSideWallNode");
    RSideWall->setPosition(width/2,0,thickness/2);
    RSideWall->yaw(Ogre::Degree(90));
    RSideWall->attachObject(util->createPlane(REntName, RPlaneName,"Ogre/Skin", thickness,height, false));

    Ogre::String TPlaneName = textureName + "TSideWallPlane";
    Ogre::String TEntName = textureName + "TSideWallEnt";
    TSideWall = wallNode->createChildSceneNode(textureName + "TSideWallNode");
    TSideWall->setPosition(0,height/2,thickness/2);
    TSideWall->pitch(Ogre::Degree(90));
    TSideWall->attachObject(util->createPlane(TEntName, TPlaneName,"Ogre/Skin", width,thickness, false));

    Ogre::String BPlaneName = textureName + "BSideWallPlane";
    Ogre::String BEntName = textureName + "BSideWallEnt";
    BSideWall = wallNode->createChildSceneNode(textureName + "BSideWallNode");
    BSideWall->setPosition(0,-height/2,thickness/2);
    BSideWall->pitch(Ogre::Degree(90));
    BSideWall->attachObject(util->createPlane(BEntName, BPlaneName,"Ogre/Skin", width,thickness, false));
}

bool WPainting::LoadImage(int ObjectID, Ogre::String texName)
{
    BufferedTextureName = texName;
    std::string fileName = WPainting::GetRandomFile(ObjectID);
    std::stringstream ss;
    ss << WPainting::PATH << ObjectID << "/" << fileName;

    Ogre::String texture_path = ss.str();

    bool image_loaded = false;
    std::ifstream ifs(texture_path.c_str(), std::ios::binary|std::ios::in);
    if (ifs.is_open())
    {
        Ogre::String tex_ext;
        Ogre::String::size_type index_of_extension = texture_path.find_last_of('.');
        Ogre::Image img;
        if (index_of_extension != Ogre::String::npos)
        {
            tex_ext = texture_path.substr(index_of_extension+1);
            Ogre::DataStreamPtr data_stream(new Ogre::FileStreamDataStream(texture_path, &ifs, false));
            img.load(data_stream, tex_ext);

            Ogre::TextureManager::getSingleton().loadImage(texName,
                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, img, Ogre::TEX_TYPE_2D, 0, 1.0f);

            image_loaded = true;
        }
        ifs.close();

        if(img.getHeight() >= img.getWidth())
        {
            height = maxH;
            width  = maxW * (double)img.getWidth()/(double)img.getHeight();
        }
        else
        {
            width  = maxW;
            height = maxH * (double)img.getHeight()/(double)img.getWidth();
        }
    }
    return image_loaded;
}


void WPainting::UnloadAll(void)
{
//    Ogre::TextureManager::unload(textureName);
    Ogre::TextureManager::getSingleton().unload(textureName);
    Ogre::TextureManager::getSingleton().remove(textureName);

    Ogre::MaterialManager::getSingleton().unload(materialName);
    Ogre::MaterialManager::getSingleton().remove(materialName);

    Ogre::MeshManager::getSingleton().unload(meshName);
    Ogre::MeshManager::getSingleton().remove(meshName);

    Ogre::SceneNode::ChildNodeIterator it = wallNode->getChildIterator();
    Ogre::String uniquename = " ";
    while (it.hasMoreElements())
    {
        uniquename = it.getNext()->getName();
        Ogre::Entity *ent = (Ogre::Entity*)mSceneMgr->getSceneNode(uniquename)->getAttachedObject(0);
        mSceneMgr->destroyEntity(ent->getName());
        mSceneMgr->getSceneNode(uniquename)->detachAllObjects();
        mSceneMgr->getSceneNode(uniquename)->removeAndDestroyAllChildren();
        ent = NULL;
    }

    Ogre::MeshManager::getSingleton().removeAll();

    wallNode->removeAndDestroyAllChildren();

    Ogre::MeshManager::getSingleton().removeUnreferencedResources();
    Ogre::TextureManager::getSingleton().removeUnreferencedResources();
    Ogre::MaterialManager::getSingleton().removeUnreferencedResources();
}

void WPainting::SetTexturePath(std::string path)
{
    texturePath = path;
}

void WPainting::Redraw(void)
{        
    while(!NewPaintingLoaded)
    {
//        sleep(0.1);
        std::cout << "Loading " << std::endl;
    }

    textureName = BufferedTextureName;
//    LoadImage(ObjectID, textureName);
    SetMaterial();
    CreateFrame();

    NewPaintingLoaded = false;
}

bool WPainting::LoadImg(int ObjectID, Ogre::String texName)
{
//    BufferedTextureName = texName;
    std::string fileName = WPainting::GetRandomFile(ObjectID);
    std::stringstream ss;
    ss << WPainting::PATH << ObjectID;
    Ogre::String folder = ss.str();

    ss << "/" << fileName;
    Ogre::String texture_path = ss.str();

    bool image_loaded = false;
    std::ifstream ifs(texture_path.c_str(), std::ios::binary|std::ios::in);
    if (ifs.is_open())
    {
        Ogre::String tex_ext;
        Ogre::String::size_type index_of_extension = texture_path.find_last_of('.');
        Ogre::Image img;
        if (index_of_extension != Ogre::String::npos)
        {
            tex_ext = texture_path.substr(index_of_extension+1);
            Ogre::DataStreamPtr data_stream(new Ogre::FileStreamDataStream(texture_path, &ifs, false));
            img.load(data_stream, tex_ext);
            std::cout << "here" << std::endl;


            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(folder, "FileSystem", "Load");
//            Ogre::TextureManager::getSingleton().loadImage(texName,
//                                                           Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME, img, Ogre::TEX_TYPE_2D, 0, 1.0f);

            image_loaded = true;
        }
        ifs.close();
    }
    return image_loaded;
}
