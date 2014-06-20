/*
-----------------------------------------------------------------------------
Filename:    MuseumApp.h
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
#ifndef __MuseumApp_h_
#define __MuseumApp_h_

//#include "BaseApplication.h"
#include "ogrewidget.h"
#include "Room.h"
#include <boost/thread.hpp>

#define WIDTH 9500
#define HEIGHT 800
#define DEPTH 9500

class MuseumApp : public OgreWidget
{    

public:    
    enum{RIGHT =0, LEFT=1, BACK=2, FRONT=3};
    MuseumApp(QWidget*parent);
    ~MuseumApp(void);
    void DrawRoom(Room *room, Ogre::SceneNode *mNode);

protected:
    void CreateScene();
    void CreateRoom(Ogre::SceneNode* node);
    void InitializeParams();
//    virtual bool keyPressed( const OIS::KeyEvent &evt );
    void MoveToRoom(int i);
    bool frameStarted(const Ogre::FrameEvent& evt);
    void runThread();
    void destroyScene(void);

    void Update(void);
private:    
    Utils *util;
    Room *currRoom;
    Ogre::SceneNode* mNode;
    bool IsMoving;
    bool loadPainting;
    std::vector< std::vector<int> > RoomIndex;
    int P;
};



#endif // #ifndef __MuseumApp_h_
