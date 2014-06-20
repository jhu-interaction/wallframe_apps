#ifndef __OGREWIDGET_H__
#define __OGREWIDGET_H__

#include <OGRE/Ogre.h>
#include <QGLWidget>
#include <QX11Info>
#include <QKeyEvent>
#include "CameraMan.h"

//#include <OIS/OISEvents.h>
//#include <OIS/OISInputManager.h>
//#include <OIS/OISKeyboard.h>
//#include <OIS/OISMouse.h>

//#include <SdkTrays.h>
//#include <SdkCameraMan.h>

class OgreWidget : public QGLWidget,public Ogre::FrameListener
{
    //Q_OBJECT;

public:
    OgreWidget( QWidget *parent=0 );

    ~OgreWidget()
    {
        mRoot->shutdown();
        delete mRoot;
        delete mCameraMan;
        destroy();
    }
    virtual void CreateScene() = 0;

protected:
    // OpenGL functions
    virtual void initializeGL();
    virtual void resizeGL( int, int );
    virtual void paintGL();

    void init(void);
    void setupResources(void);
    void loadResources(void);
    void CreateCamera(void);
    void CreateViewports(void);
    bool Configure(void);

    // Ogre::FrameListener
    virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

    // OgreBites
//    OgreBites::SdkCameraMan* mCameraMan;       // basic camera controller

    Ogre::Root *mRoot;
    Ogre::RenderWindow *mWindow;
    Ogre::Camera *mCamera;
    Ogre::Viewport *mViewport;
    Ogre::SceneManager *mSceneMgr;
    Ogre::String mResourcesCfg;
    Ogre::String mPluginsCfg;

    QOgre::CameraMan*   mCameraMan;

    bool mShutDown;

private:
    //    void paintEvent(QPaintEvent *e);
    //    void resizeEvent(QResizeEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
    void wheelEvent(QWheelEvent *e);
    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    bool updateExternalSystems();
    Ogre::Vector2		    mAbsolute;
    Ogre::Vector2		    mRelative;

public:
    bool mSystemInitialized;
};

#endif
