#include "Room.h"
#include <string>

std::vector< std::vector<std::vector<int> > > Room::Map;
//std::map<int, std::vector<int,8> > Room::Map;

Room::Room(std::string name, Utils *u, int nP, double w, double h, double d)
{
    Wall = new WALL;
    width = w;
    height = h;
    depth = d;
    roomName =  name+"_RoomNode";
    util = u;
    numPaintings = nP;
    Node = util->mSceneMgr->createSceneNode(roomName);
}

Room::~Room()
{    
    delete Wall;
    Node->removeAndDestroyAllChildren();
    Node->detachAllObjects();
}

void Room::CreateAdjacentRooms(int numR, int numP[])
{
    DetachAdjacentRooms();
    rooms.clear();
    numRooms = numR;

    std::stringstream ss ;

    if(numRooms != 0)
    {
        for (int i = 1; i <= numR; ++i)
        {
            ss.flush();
            ss << roomName << rand() << (i-1);
            Room *r = new Room(ss.str(), util, numP[i-1], width, height, depth);
            rooms.push_back(r);
        }
    }
}

void Room::DetachAdjacentRooms(void)
{
    for (int i = 0; i < rooms.size(); ++i)
        rooms[i] = NULL;
}



void Room::Draw(void)
{       
    //    Ogre::SceneNode *node1 = Node->createChildSceneNode(roomName+"_node1");
    //    Ogre::SceneNode *node2 = Node->createChildSceneNode(roomName+"_node2");
    //    Ogre::SceneNode *node3 = Node->createChildSceneNode(roomName+"_node3");
    //    Ogre::SceneNode *node4 = Node->createChildSceneNode(roomName+"_node4");
    //    Ogre::SceneNode *node5 = Node->createChildSceneNode(roomName+"_node5");
    //    Ogre::SceneNode *node6 = Node->createChildSceneNode(roomName+"_node6");

    // Front
    Wall->Node[0] = Node->createChildSceneNode(roomName+"_node0");
    Wall->Node[0]->setPosition(0.0, 0.0, -depth/2);

    // Back
    Wall->Node[1] = Node->createChildSceneNode(roomName+"_node1");
    Wall->Node[1]->setPosition(0.0, 0.0, depth/2);
    Wall->Node[1]->yaw(Ogre::Degree(180));

    // Left
    Wall->Node[2] = Node->createChildSceneNode(roomName+"_node2");
    Wall->Node[2]->yaw(Ogre::Degree(90));
    Wall->Node[2]->setPosition(-width/2.0, 0.0, 0.0);

    // Right
    Wall->Node[3] = Node->createChildSceneNode(roomName+"_node3");
    Wall->Node[3]->yaw(Ogre::Degree(270));
    Wall->Node[3]->setPosition(width/2.0, 0.0, 0.0);

    // Bottom
    Wall->Node[4] = Node->createChildSceneNode(roomName+"_node4");
    Wall->Node[4]->pitch(Ogre::Degree(90));
    Wall->Node[4]->setPosition(0.0, -height/2.0, 0.0);

    // Top
    Wall->Node[5] = Node->createChildSceneNode(roomName+"_node5");
    Wall->Node[5]->pitch(Ogre::Degree(90));
    Wall->Node[5]->setPosition(0.0, height/2.0, 0.0);

    //    node1->attachObject(util->createPlane(roomName+"_Wall_front"        ,roomName+"_plane_1"  , "Ogre/Wall"     , width, height, true ));
    Wall->Node[0]->addChild(CreateWall("DoorFront", width, height));
    Wall->Node[1]->addChild(CreateWall("DoorBack", width, height));
    Wall->Node[2]->addChild(CreateWall("DoorLeft", depth, height));
    Wall->Node[3]->addChild(CreateWall("DoorRight", depth, height));
    Wall->Node[4]->attachObject(util->createPlane(roomName+"_Wall_bottom"       ,roomName+"_plane_4"  , "Ogre/Floor"    , width, depth, false));
    Wall->Node[5]->attachObject(util->createPlane(roomName+"_Wall_top"          ,roomName+"_plane_5"  , "Ogre/Floor"    , width, depth, true ));
    //    node4->attachObject(util->createPlane(roomName+"_Wall_back"         ,roomName+"_plane_4"  , "Ogre/Wall"     , width, height, false));
    //    node5->attachObject(util->createPlane(roomName+"_Wall_left"         ,roomName+"_plane_5"  , "Ogre/Wall"     , depth, height, true ));
    //    node6->attachObject(util->createPlane(roomName+"_Wall_right"        ,roomName+"_plane_6"  , "Ogre/Wall"     , depth, height, false));

    DrawPaintings();
}

Ogre::SceneNode* Room::CreateWall(std::string randomName, double w, double h)
{
    Ogre::SceneNode *node1 = util->mSceneMgr->createSceneNode(roomName + randomName + "_node");

    Ogre::SceneNode *node2 = node1->createChildSceneNode(roomName + randomName + "_child_1");
    node2->attachObject(util->createPlane(roomName + randomName + "_Wall_left"  ,roomName + randomName + "_plane_1"  , "Ogre/Wall"     , w/2-175, h, true ));
    node2->setPosition(-w/4-87.5,0,0);

    Ogre::SceneNode *node3 = node1->createChildSceneNode(roomName + randomName + "_child2");
    node3->attachObject(util->createPlane(roomName + randomName + "_Wall_right" ,roomName + randomName + "_plane_2"  , "Ogre/Wall"     , w/2-175, h, true ));
    node3->setPosition(w/4+87.5,0,0);

    Ogre::SceneNode *node4 = node1->createChildSceneNode(roomName + randomName + "_child3");
    node4->attachObject(util->createPlane(roomName + randomName + "_Wall_top"   ,roomName + randomName + "_plane_3"  , "Ogre/Wall"     , 350, 50, true ));
    node4->setPosition(0,h/2-25,0);

    //    Ogre::SceneNode *node5 = node1->createChildSceneNode(roomName + randomName + "_child4");
    //    node5->attachObject(util->createPlane(roomName + randomName + "_Wall_Line_bottom"   ,roomName + randomName + "_plane_4"  , "Ogre/LineTex"     , width, 25, true ));
    //    node5->setPosition(0, -height/2 + 50, 0);

    return node1;
}

void Room::DrawPaintings(void)
{
    double rW = width;
    double border = 40.0;
    double inBwSpace = 50.0;
    double doorWidth = 350.0;

    double whiteSpace = (rW - 4*border - doorWidth - inBwSpace*(numPaintings-1))/numPaintings;
    double maxW = 2.5*whiteSpace/5;
    double maxH = maxW;

    // Paintings on all four walls
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < numPaintings; ++i)
        {
            std::stringstream ss;
            ss << "Image" << rand() << i;
            paint  = NULL;
            paint = new WPainting(util->mSceneMgr, ss.str(), WPainting::objFiles[rand() % WPainting::objFiles.size()], maxW, maxH);
            //            paint->CreateFrame();
            double loc;

            if(i >= std::floor(numPaintings/2))
                loc = -rW/2.0 + border + 2*inBwSpace + doorWidth + 3*maxW/4 + (whiteSpace + inBwSpace) * i;
            else
                loc = -rW/2.0 + border + 3*maxW/4 + (whiteSpace + inBwSpace) * i;

            paint->SetPosition(loc, 0, 0);
            Wall->art[j].push_back(paint);

            paint = NULL;
        }
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < numPaintings; ++j)
            Wall->Node[i]->addChild(Wall->art[i][j]->GetNode());
}

void Room::RedrawPaintings(void)
{
    // Remove Previous Nodes
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < numPaintings; ++j)
            Wall->Node[i]->removeChild(Wall->art[i][j]->GetNode());

    //    int k=0;
    //    for (int j = 0; j < 4; ++j)
    //    {
    //        for (int i = 0; i < numPaintings; ++i)
    //        {
    //            std::stringstream ss;
    //            ss << "Image" << rand() << i;
    //            paint  = NULL;
    //            paint = new WPainting(util->mSceneMgr, ss.str(), WPainting::objFiles[index[k]], maxW, maxH);

    //            Wall->art[j].push_back(paint);

    //            paint = NULL;
    //            k++;
    //        }
    //    }

    for (int i = 0; i < 4; ++i)
    {
        Wall->art[i].clear();
        Wall->art[i] = paintings[i];
    }

    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < numPaintings; ++j)
            Wall->Node[i]->addChild(Wall->art[i][j]->GetNode());
}

void Room::LoadPaintingToMemory(std::vector<int> index)
{
    BufferdNames.clear();
    std::stringstream ss;
    int k=0;
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < numPaintings; ++i)
        {
            ss << "Image" << i << rand();
            BufferdNames.push_back(Ogre::String(ss.str()));

            if(Wall->art[j][i]->LoadImage(index[k], Ogre::String(ss.str())))
                Wall->art[j][i]->NewPaintingLoaded = true;
            else
                Wall->art[j][i]->NewPaintingLoaded = false;
            k++;
        }
    }
}

void Room::SetNewPaintings(void)
{   
    int k=0;
    for (int j = 0; j < 4; ++j)
    {
        for (int i = 0; i < numPaintings; ++i)
        {
            std::stringstream ss;
            ss << "Image" << rand() << i;

            Wall->art[j][i]->UnloadAll();
            Wall->art[j][i]->Redraw();

            k++;
        }
    }
}
