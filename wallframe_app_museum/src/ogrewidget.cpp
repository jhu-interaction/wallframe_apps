#include "ogrewidget.h"


OgreWidget::OgreWidget(QWidget *parent) :
    QGLWidget( parent ),
    mShutDown(false),
    mWindow(NULL)
{
//    setMinimumSize(5760,3240);      // For Wall
//    setMinimumSize(800,600);        // PC
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
    init();
}

void OgreWidget::init(void )
{

#ifdef _DEBUG
    mResourcesCfg = "resources_d.cfg";
    mPluginsCfg = "plugins_d.cfg";
#else
    mResourcesCfg = "/home/kel/jhu-interaction/wallframe_apps/wallframe_app_museum/dist/bin/resources.cfg";
    mPluginsCfg = "/home/kel/jhu-interaction/wallframe_apps/wallframe_app_museum/dist/bin/plugins.cfg";
#endif

    // create the main ogre object
    mRoot = new Ogre::Root( mPluginsCfg);

    setupResources();

    Ogre::RenderSystem *renderSystem = mRoot->getRenderSystemByName("OpenGL Rendering Subsystem");

    assert( renderSystem ); // user might pass back a null renderer, which would be bad!

    mRoot->setRenderSystem( renderSystem );
    QString dimensions = QString( "%1x%2" )
            .arg(this->width())
            .arg(this->height());

    renderSystem->setConfigOption( "Video Mode", dimensions.toStdString() );

    // initialize without creating window
    mRoot->getRenderSystem()->setConfigOption( "Full Screen", "No" );
    mRoot->saveConfig();
    mRoot->initialise(false); // don't create a window    
}

bool OgreWidget::Configure()
{    

    //== Creating and Acquiring Ogre Window ==//

    // Get the parameters of the window QT created
    Ogre::String winHandle;
#ifdef WIN32
    // Windows code
    winHandle += Ogre::StringConverter::toString((unsigned long)(this->parentWidget()->winId()));
#elif MACOS
    // Mac code, tested on Mac OSX 10.6 using Qt 4.7.4 and Ogre 1.7.3
    Ogre::String winHandle  = Ogre::StringConverter::toString(winId());
#else
    // Unix code

    QX11Info info = x11Info();
    winHandle  = Ogre::StringConverter::toString((unsigned long)(info.display()));    
    winHandle += ":";    
    winHandle += Ogre::StringConverter::toString((unsigned int)(info.screen()));    
    winHandle += ":";    
    winHandle += Ogre::StringConverter::toString((unsigned long)(this->parentWidget()->winId()));    
#endif

    Ogre::NameValuePairList params;
#ifndef MACOS
    // code for Windows and Linux
    params["parentWindowHandle"] = winHandle;
    params["externalGLControl"] = "true";
    params["currentGLContext"] = "true";
//    params["left"] = "0" ;
//    params["top"] = "0";

    mWindow = mRoot->createRenderWindow( "QOgreWidget_RenderWindow",
                                         this->width(),
                                         this->height(),
                                         false,
                                         &params );

    mWindow->setActive(true);
    mWindow->setVisible(true);
    WId ogreWinId = 0x0;
    mWindow->getCustomAttribute( "WINDOW", &ogreWinId );

//    assert( ogreWinId );

    // bug fix, extract geometry
    QRect geo = this->frameGeometry ( );

    // create new window
    this->create( ogreWinId );

    // set geometrie infos to new window
    this->setGeometry (geo);

#else
    // code for Mac
    params["externalWindowHandle"] = winHandle;
    params["macAPI"] = "cocoa";
    params["macAPICocoaUseNSView"] = "true";
    mWindow = mRoot->createRenderWindow("QOgreWidget_RenderWindow",
                                        width(), height(), false, &params);
    mWindow->setActive(true);
    makeCurrent();
#endif

    setAttribute( Qt::WA_PaintOnScreen, true );
    setAttribute( Qt::WA_NoBackground );
}

void OgreWidget::initializeGL()
{            
    Configure();
    mSceneMgr = mRoot->createSceneManager( Ogre::ST_GENERIC );

    CreateCamera();
    CreateViewports();

    // Set default mipmap level (NB some APIs ignore this)
    Ogre::TextureManager::getSingleton().setDefaultNumMipmaps(5);

    // Load resources
    loadResources();

    mSystemInitialized = true;

    CreateScene();
}

void OgreWidget::CreateCamera(void)
{
    mCamera = mSceneMgr->createCamera("PlayerCam");

    mCamera->setPosition(Ogre::Vector3(0,0,0));
    mCamera->lookAt(Ogre::Vector3(0,0,-1));
    mCamera->setNearClipDistance(5);

    mCameraMan = new QOgre::CameraMan(mCamera);
//    mCameraMan = new OgreBites::SdkCameraMan(mCamera);   // create a default camera controller
}

void OgreWidget::CreateViewports()
{
    Ogre::Viewport *mViewport = mWindow->addViewport( mCamera );
    mViewport->setBackgroundColour(Ogre::ColourValue(0,0,0));
}

void OgreWidget::paintGL()
{
    // Be sure to call "OgreWidget->repaint();" to call paintGL
    assert( mWindow );
    mRoot->renderOneFrame();
    mWindow->update(false);
}

void OgreWidget::resizeGL( int width, int height )
{
    //    assert( mWindow );
    //    mWindow->windowMovedOrResized();

    assert( mWindow );
    mWindow->resize(width, height);
    mCamera->setAspectRatio(((qreal)width)/((qreal)height));
    mWindow->windowMovedOrResized();
}

void OgreWidget::setupResources(void)
{
    // Load resource paths from config file
    Ogre::ConfigFile cf;
    cf.load(mResourcesCfg);

    // Go through all sections & settings in the file
    Ogre::ConfigFile::SectionIterator seci = cf.getSectionIterator();

    Ogre::String secName, typeName, archName;
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        Ogre::ConfigFile::SettingsMultiMap *settings = seci.getNext();
        Ogre::ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            Ogre::ResourceGroupManager::getSingleton().addResourceLocation(
                        archName, typeName, secName);
        }
    }
}

void OgreWidget::loadResources(void)
{
    Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}


bool OgreWidget::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
    std::cout << "Fra" << std::endl;
    if(mWindow->isClosed())
        return false;

    if(mShutDown)
        return false;

    mCameraMan->frameRenderingQueued(evt);   // if dialog isn't up, then update the camera

    return true;
}

void OgreWidget::mouseMoveEvent(QMouseEvent *e)
{
    if (mSystemInitialized)
    {
        Ogre::Vector2 oldPos = mAbsolute;
        mAbsolute = Ogre::Vector2(e->pos().x(), e->pos().y());
        mRelative = mAbsolute - oldPos;
        mCameraMan->injectMouseMove(mRelative);
        repaint();
    }
}

//-------------------------------------------------------------------------------------
void OgreWidget::mousePressEvent(QMouseEvent *e)
{
    std::cout << "Press" << std::endl;
    if (mSystemInitialized)
    {
        mCameraMan->injectMouseDown(e);
        repaint();
    }
}

//-------------------------------------------------------------------------------------
void OgreWidget::mouseReleaseEvent(QMouseEvent *e)
{
    std::cout << "Release" << std::endl;
    if (mSystemInitialized)
    {
        mCameraMan->injectMouseUp(e);
        repaint();
    }
}
//-------------------------------------------------------------------------------------
void OgreWidget::wheelEvent(QWheelEvent *e)
{
    if (mSystemInitialized)
    {
//        mCameraMan->injectMouseWheel(e);
        repaint();
    }
}

//-------------------------------------------------------------------------------------
void OgreWidget::keyPressEvent(QKeyEvent *e)
{
    if (mSystemInitialized)
    {
        mCameraMan->injectKeyDown(e);
        repaint();
    }
}

//-------------------------------------------------------------------------------------
void OgreWidget::keyReleaseEvent(QKeyEvent *e)
{
    if (mSystemInitialized)
    {
        mCameraMan->injectKeyUp(e);
        repaint();
    }
}

//-------------------------------------------------------------------------------------
bool OgreWidget::updateExternalSystems()
{
    return true;
}



