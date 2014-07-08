/*
-----------------------------------------------------------------------------
Filename:    MuseumApp.cpp
-----------------------------------------------------------------------------

This source file is part of the
   ___                 __    __ _ _    _
  /___\__ _ _ __ ___  / / /\ \ (_) | _(_)
 //  // _` | '__/ _ \ \ \/  \/ / | |/ / |
/ \_// (_| | | |  __/  \  /\  /| |   <| |
\___/ \__, |_|  \___|   \/  \/ |_|_|\_\_|
      |___/
      Tutorial Framework
      http://www.ogre3d.org/tikiwiki/
-----------------------------------------------------------------------------
*/
#include "MuseumApp.h"

#include <ros/node_handle.h>
//-------------------------------------------------------------------------------------
MuseumApp::MuseumApp(QWidget *parent) :
    OgreWidget(parent)
{

}

//-------------------------------------------------------------------------------------
MuseumApp::~MuseumApp(void)
{
    delete util;
    delete currRoom;

    mNode->detachAllObjects();
    mNode->removeAndDestroyAllChildren();

    mSceneMgr->destroyAllEntities();
    mSceneMgr->destroyAllManualObjects();

    Ogre::TextureManager::getSingleton().removeAll();
    Ogre::MaterialManager::getSingleton().removeAll();
    Ogre::MeshManager::getSingleton().removeAll();
}

void MuseumApp::InitializeParams(void)
{
    WPainting::Initialize("/home/kel/dev/libMuseum/waltersApi/scripts/id.txt");
    util = new Utils(mSceneMgr);
    currRoom = new Room("Room", util, 2, WIDTH, HEIGHT, DEPTH);
    int f[] = {2,2,2,2};

    // (numOfRooms, numOfpaintings in each rooom)
    currRoom->CreateAdjacentRooms(4, f);

    int TotalRooms = (int)WPainting::objFiles.size()/8;
    int maxRoomIndex = sqrt(TotalRooms);

    for (int i = 0; i < (int)WPainting::objFiles.size()/8; ++i)
    {
        std::vector<int> indices;
        for (int j = 0; j < 8; ++j)
        {
            indices.push_back(WPainting::objFiles[8*i + j]);
        }
        RoomIndex.push_back(indices);
    }



    for (int i = 0; i < maxRoomIndex; ++i)
    {
        std::vector<std::vector<int> > yIndices;
        for (int j = 0; j < maxRoomIndex; ++j)
        {
            std::vector<int> xIndices;
            for (int k = 0; k < 8; ++k)
                xIndices.push_back(WPainting::objFiles[8*j + k]);

            yIndices.push_back(xIndices);
        }
        Room::Map.push_back(yIndices);
    }

    loadPainting = false;
    PaintingIndex=0;
    PaintingNumber=0;
}

//-------------------------------------------------------------------------------------
void MuseumApp::CreateScene()
{
    IsMoving = true;
    InitializeParams();

    // Set the scene's ambient light
    mSceneMgr->setAmbientLight(Ogre::ColourValue(0.9f, 0.9f, 0.9f));

    mNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();

    DrawRoom(currRoom, mNode);
    currRoom->SetPosition(0,0,0);

    DrawRoom(currRoom->rooms[RIGHT], mNode);      // Right
    DrawRoom(currRoom->rooms[LEFT], mNode);     // Left
    DrawRoom(currRoom->rooms[BACK], mNode);     // Back
    DrawRoom(currRoom->rooms[FRONT], mNode);    // Front


    currRoom->rooms[RIGHT]->SetPosition(currRoom->GetWidth()+2,0,0);
    currRoom->rooms[LEFT]->SetPosition(-(currRoom->GetWidth()+2),0,0);
    currRoom->rooms[BACK]->SetPosition(0,0,(currRoom->GetDepth()+2));
    currRoom->rooms[FRONT]->SetPosition(0,0,-(currRoom->GetDepth()+2));

    repaint();
//    update();
    // Create the thread and start work
//    assert(!mThread);
//    mThread = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&MuseumApp::runThread, this)));

//    std::stringstream ss;
//    ss << "/home/kel/dev/libMuseum/waltersApi/scripts/Images_Id/" << WPainting::objFiles[10];

//    Ogre::ResourceGroupManager::getSingleton().addResourceLocation(ss.str(), "FileSystem", "Buffered");
//    Ogre::ResourceGroupManager::getSingleton().initialiseResourceGroup("Buffered");

//    Ogre::TextureManager::getSingleton().load(WPainting::GetRandomFile(WPainting::objFiles[10]),"Buffered");
}

void MuseumApp::DrawRoom(Room *room, Ogre::SceneNode *mNode)
{
    room->Draw();

    mNode->addChild(room->GetNode());
}

void MuseumApp::MoveToRoom(int i)
{        
    Room *temp = currRoom;
    Ogre::Vector3 pose;
    int f[] = {2,2,2,2};

    currRoom = temp->rooms[i];
    currRoom->CreateAdjacentRooms(4, f);

    switch(i)
    {
    case FRONT:
        currRoom->rooms[FRONT] = temp->rooms[BACK];
        currRoom->rooms[LEFT]  = temp->rooms[LEFT];
        currRoom->rooms[RIGHT] = temp->rooms[RIGHT];
        currRoom->rooms[BACK]  = temp;

        temp->DetachAdjacentRooms();
        temp = NULL;

        pose = currRoom->rooms[BACK]->GetPosition();
        currRoom->rooms[BACK]->SetPosition(pose.x, pose.y, pose.z );

        pose = currRoom->rooms[FRONT]->GetPosition();
        currRoom->rooms[FRONT]->SetPosition(pose.x, pose.y, pose.z - 3*(currRoom->GetDepth()+2) );

        pose = currRoom->rooms[RIGHT]->GetPosition();
        currRoom->rooms[RIGHT]->SetPosition(pose.x, pose.y, pose.z - (currRoom->GetDepth()+2) );

        pose = currRoom->rooms[LEFT]->GetPosition();
        currRoom->rooms[LEFT]->SetPosition(pose.x, pose.y, pose.z - (currRoom->GetDepth()+2));

        currRoom->rooms[LEFT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
        currRoom->rooms[RIGHT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);

        currRoom->rooms[LEFT]->SetNewPaintings();
        currRoom->rooms[RIGHT]->SetNewPaintings();

        break;

    case BACK:
        currRoom->rooms[BACK]  = temp->rooms[FRONT];
        currRoom->rooms[LEFT]  = temp->rooms[LEFT];
        currRoom->rooms[RIGHT] = temp->rooms[RIGHT];
        currRoom->rooms[FRONT] = temp;

        temp->DetachAdjacentRooms();
        temp = NULL;

        pose = currRoom->rooms[FRONT]->GetPosition();
        currRoom->rooms[FRONT]->SetPosition(pose.x, pose.y, pose.z );

        pose = currRoom->rooms[BACK]->GetPosition();
        currRoom->rooms[BACK]->SetPosition(pose.x, pose.y, pose.z + 3*(currRoom->GetDepth()+2) );

        pose = currRoom->rooms[RIGHT]->GetPosition();
        currRoom->rooms[RIGHT]->SetPosition(pose.x, pose.y, pose.z + (currRoom->GetDepth()+2) );

        pose = currRoom->rooms[LEFT]->GetPosition();
        currRoom->rooms[LEFT]->SetPosition(pose.x, pose.y, pose.z + (currRoom->GetDepth()+2));

        currRoom->rooms[LEFT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
        currRoom->rooms[RIGHT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);

        currRoom->rooms[LEFT]->SetNewPaintings();
        currRoom->rooms[RIGHT]->SetNewPaintings();

        break;

    case LEFT:
        currRoom->rooms[LEFT]  = temp->rooms[RIGHT];
        currRoom->rooms[FRONT] = temp->rooms[FRONT];
        currRoom->rooms[BACK]  = temp->rooms[BACK];
        currRoom->rooms[RIGHT] = temp;

        temp->DetachAdjacentRooms();
        temp = NULL;

        pose = currRoom->rooms[RIGHT]->GetPosition();
        currRoom->rooms[RIGHT]->SetPosition(pose.x, pose.y, pose.z );

        pose = currRoom->rooms[LEFT]->GetPosition();
        currRoom->rooms[LEFT]->SetPosition(pose.x - 3*(currRoom->GetWidth()+2), pose.y, pose.z );

        pose = currRoom->rooms[FRONT]->GetPosition();
        currRoom->rooms[FRONT]->SetPosition(pose.x - (currRoom->GetWidth()+2), pose.y, pose.z );

        pose = currRoom->rooms[BACK]->GetPosition();
        currRoom->rooms[BACK]->SetPosition(pose.x - (currRoom->GetWidth()+2), pose.y, pose.z);

        currRoom->rooms[FRONT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
        currRoom->rooms[BACK]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);

        currRoom->rooms[FRONT]->SetNewPaintings();
        currRoom->rooms[BACK]->SetNewPaintings();

        break;

    case RIGHT:
        currRoom->rooms[RIGHT]  = temp->rooms[LEFT];
        currRoom->rooms[FRONT]  = temp->rooms[FRONT];
        currRoom->rooms[BACK]   = temp->rooms[BACK];
        currRoom->rooms[LEFT]   = temp;

        temp->DetachAdjacentRooms();
        temp = NULL;

        pose = currRoom->rooms[LEFT]->GetPosition();
        currRoom->rooms[LEFT]->SetPosition(pose.x, pose.y, pose.z );

        pose = currRoom->rooms[RIGHT]->GetPosition();
        currRoom->rooms[RIGHT]->SetPosition(pose.x + 3*(currRoom->GetWidth()+2), pose.y, pose.z );

        pose = currRoom->rooms[FRONT]->GetPosition();
        currRoom->rooms[FRONT]->SetPosition(pose.x + (currRoom->GetWidth()+2), pose.y, pose.z );

        pose = currRoom->rooms[BACK]->GetPosition();
        currRoom->rooms[BACK]->SetPosition(pose.x + (currRoom->GetWidth()+2), pose.y, pose.z);

        currRoom->rooms[FRONT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
        currRoom->rooms[BACK]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);

        currRoom->rooms[FRONT]->SetNewPaintings();
        currRoom->rooms[BACK]->SetNewPaintings();

        break;
    }

    loadPainting = true;

}

void MuseumApp::MoveToNextPainting()
{
    WPainting *paint = currRoom->MoveToPainting(mCamera, PaintingNumber++);
    Ogre::Vector3 location = paint->GetNode()->convertLocalToWorldPosition(Ogre::Vector3(0,0,1500));
    Ogre::Vector3 paintingOrigin = paint->GetNode()->convertLocalToWorldPosition(Ogre::Vector3(0,0,0));

#if 1
    for (double i = 0; i <= 1; i=i+0.005)
    {
        Ogre::Vector3 startPos = mCamera->getPosition();
        Ogre::Vector3 startLookAt = mCamera->getDirection() + startPos;

        Ogre::Vector3 NewPos = (1-i)*startPos + i * location;
        Ogre::Vector3 NewLookAt = (1-i)*startLookAt + i * paintingOrigin;
        mCamera->setPosition(NewPos);
        mCamera->lookAt(NewLookAt);
        repaint();
    }
    ROS_WARN_STREAM("Done");
#else
    mCamera->setPosition(location);
    mCamera->lookAt(paintingOrigin);
    repaint();
#endif
    if(PaintingNumber == 8)
        PaintingNumber = 1;
}

void MuseumApp::MoveToPreviousPainting()
{
    WPainting *paint  = currRoom->MoveToPainting(mCamera, PaintingNumber--);
    Ogre::Vector3 location = paint->GetNode()->convertLocalToWorldPosition(Ogre::Vector3(0,0,1500));
    Ogre::Vector3 paintingOrigin = paint->GetNode()->convertLocalToWorldPosition(Ogre::Vector3(0,0,0));

#if 1
    for (double i = 0; i <= 1; i=i+0.0005)
    {
        Ogre::Vector3 startPos = mCamera->getPosition();
        Ogre::Vector3 startLookAt = mCamera->getDirection() + startPos;

        Ogre::Vector3 NewPos = (1-i)*startPos + i * location;
        Ogre::Vector3 NewLookAt = (1-i)*startLookAt + i * paintingOrigin;
        mCamera->setPosition(NewPos);
        mCamera->lookAt(NewLookAt);
        repaint();
    }
    ROS_WARN_STREAM("Done");
#else
    mCamera->setPosition(location);
    mCamera->lookAt(paintingOrigin);
    repaint();
#endif

    if(PaintingNumber == 1)
        PaintingNumber = 8;
}

bool MuseumApp::frameStarted(const Ogre::FrameEvent &evt)
{

    return true;
}


void MuseumApp::destroyScene(void)
{
    //    // Stops the thread
//    assert(mThread);
//    mThread->join();

    // Destroy scene
//    BaseApplication::destroyScene();
}
/*
void MuseumApp::runThread()
{
    while(!mShutDown)
    {
        return;
        if(loadPainting)
        {
            loadPainting = false;
            // Lock mutex
            boost::unique_lock<boost::mutex>* lock = new boost::unique_lock<boost::mutex>(mMutex);

            //            currRoom->rooms[LEFT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
            //            currRoom->rooms[RIGHT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
            //            currRoom->rooms[FRONT]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
            //            currRoom->rooms[BACK]->LoadPaintingToMemory(RoomIndex[PaintingIndex++]);
            std::stringstream ss;
            ss << rand();
            for (int i = 0; i < 96; ++i)
                WPainting::LoadImg(WPainting::objFiles[i], Ogre::String(ss.str()));

            // Unlock mutex
            delete lock;

            // Wait
            boost::this_thread::sleep(boost::posix_time::milliseconds(25));
        }
    }
}
*/
/*
bool MuseumApp::keyPressed(const OIS::KeyEvent &evt )
{    
    if (evt.key == OIS::KC_ESCAPE)
        mShutDown = true;

    if( mCamera->getPosition().z <= currRoom->GetPosition().z - DEPTH/2)
        MoveToRoom(FRONT);

    if( mCamera->getPosition().z > currRoom->GetPosition().z + DEPTH/2)
        MoveToRoom(BACK);

    if( mCamera->getPosition().x <= currRoom->GetPosition().x - WIDTH/2)
        MoveToRoom(LEFT);

    if( mCamera->getPosition().x > currRoom->GetPosition().x + WIDTH/2)
        MoveToRoom(RIGHT);

    //    std::cout << mCamera->getPosition() << std::endl;

    currRoom->rooms[LEFT]->GetNode()->setVisible(true);
    currRoom->rooms[RIGHT]->GetNode()->setVisible(true);
    currRoom->rooms[FRONT]->GetNode()->setVisible(true);
    currRoom->rooms[BACK]->GetNode()->setVisible(true);


    if(mCamera->getPosition().x <= WIDTH/2)
        currRoom->rooms[LEFT]->GetNode()->setVisible(false);

    if(mCamera->getPosition().z >= -DEPTH/2)
        currRoom->rooms[BACK]->GetNode()->setVisible(false);

    //    if(mCamera->getPosition().y <= (HEIGHT/2 - 20) && mCamera->getPosition().y >= -(HEIGHT/2 - 20))
    mCameraMan->injectKeyDown(evt);

    return true;
}
*/
