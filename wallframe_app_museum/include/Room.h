#ifndef __Room_h_
#define __Room_h_

#include "WPainting.h"

#define DOOR_WDITH 800

class Room
{
    struct WALL{
        Ogre::SceneNode* Node[4];
        int* numPaintings[4];
        std::vector<WPainting*> art[4];
    };
public:
    Room(std::string name, Utils *u, int nP, double w, double h, double d);    
    Room(){};

    ~Room(void);

    void CreateAdjacentRooms(int numR, int numP[]);
    void DetachAdjacentRooms(void);    
    Ogre::SceneNode *CreateWall(std::string randomName, double w, double h);            
    void Draw(void);    
    void DrawPaintings(void);    
    void RedrawPaintings(void);
    void SetNewPaintings(void);
    void LoadPaintingToMemory(std::vector<int> index);

    // Inline Functions
    inline void SetWorldPosition(double x, double y, double z)
    {
        Ogre::Vector3 v = Node->convertWorldToLocalPosition(Ogre::Vector3(x,y,z));
        SetPosition(v.x, v.y, v.z);
    };

    inline Ogre::Vector3 GetWorldPosition(){return Node->convertLocalToWorldPosition(GetPosition());}
    inline double GetIndex(){return index;}
    inline double GetWidth(){return width;}
    inline double GetHeight(){return height;}
    inline double GetDepth(){return depth;}
    inline Ogre::SceneNode* GetNode(){return Node;}
    inline Ogre::Vector3 GetPosition(){ return Node->getPosition();}
    inline void SetPosition(double x, double y, double z){Node->setPosition(x,y,z);}    

private:
    double index;
    double width;
    double height;
    double depth;    
    Utils *util;
    Ogre::SceneNode *Node;
    WPainting *paint;        
    std::vector <WPainting*> paintings[4];

    std::vector<Ogre::String> BufferdNames;
    std::vector<Ogre::String> adjacentPainting;

public:
    Ogre::String roomName;
    WALL *Wall;
    double numPaintings;
    double numRooms;
    std::vector<Room*> rooms;
    std::vector<WPainting*> art;

//    static std::map<int, std::vector<int,8> > Map;
    static std::vector< std::vector<std::vector<int> > > Map;

};

#endif // #ifndef __Room_h_
