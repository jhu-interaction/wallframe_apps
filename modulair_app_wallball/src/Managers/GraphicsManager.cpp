
#include <Managers/GraphicsManager.h>

#include <iostream>

using namespace irr;
using namespace core;
using namespace scene;
using namespace video;
using namespace io;
using namespace gui;

QWidget* GraphicsManager::s_QWidget = 0;
void GraphicsManager::setQWidget(QWidget* qWidget) {
	s_QWidget = qWidget;
}

GraphicsManager* GraphicsManager::s_Singleton = 0;
GraphicsManager* GraphicsManager::singleton() {
	if (GraphicsManager::s_Singleton == 0)
		GraphicsManager::s_Singleton = new GraphicsManager();
	return GraphicsManager::s_Singleton;
}

void GraphicsManager::shutdown() {
	if (GraphicsManager::s_Singleton != 0)
		delete GraphicsManager::s_Singleton;
	GraphicsManager::s_Singleton = 0;
}

IGUIFont *font = 0;
dimension2d<u32> size;

GraphicsManager::GraphicsManager() {
  if (GraphicsManager::s_QWidget == 0) {
    // TODO In this case the QWidget has not been initialized and we should fail.

    if (m_Device == 0) {
      m_Device = createDevice( video::EDT_OPENGL, dimension2d<u32>(5738,3218), 16,
			       false, false, false);
    }
  } else {
    SIrrlichtCreationParameters parameters;
    
    parameters.WindowId = (void*)s_QWidget->winId();
        
    parameters.AntiAlias = 0;
    parameters.Bits = 32;
    parameters.DeviceType = EIDT_X11;
    parameters.Doublebuffer = true;
    parameters.DriverType = EDT_OPENGL;
    parameters.EventReceiver = 0;
    parameters.Fullscreen = false;
    parameters.HighPrecisionFPU = false;
    parameters.IgnoreInput = false;
    parameters.Stencilbuffer = true;
    parameters.Stereobuffer = false;
    parameters.Vsync = false;
    parameters.WindowSize.Width = s_QWidget->width();
    parameters.WindowSize.Height = s_QWidget->height();
    parameters.WithAlphaChannel = false;
    parameters.ZBufferBits = 16;
    
    m_Device = createDeviceEx( parameters );
    m_Device->setResizable(true);
  
    std::cout << "CREATED device with QtWidget: " << m_Device << "\n" << std::flush;
  }
			
  m_VideoDriver = m_Device->getVideoDriver();
  m_SceneManager = m_Device->getSceneManager();
  m_GUIEnvironment = m_Device->getGUIEnvironment();

  // CAMERA
  vector3df cameraPosition(0,22.5f,35);
  vector3df lookAtPosition(0,22.5f,0);
  ICameraSceneNode* cameraNode = m_SceneManager->addCameraSceneNode(0, cameraPosition, lookAtPosition);
  cameraNode->setUpVector(vector3df(0.0f, 1.0f, 0.0f));
  
  // LIGHTS
  m_SceneManager->setAmbientLight(SColorf(0.5,0.5,0.5,1));
  m_SceneManager->addLightSceneNode( 0, core::vector3df(60,20,100), video::SColorf(0.0f,0.0f,0.0f), 50.0f, 1 );
  //m_SceneManager->addLightSceneNode( 0, core::vector3df(-60,20,100), video::SColorf(0.0f,0.0f,5.0f), 50.0f, 1 );
  //m_SceneManager->addLightSceneNode( 0, core::vector3df(-50,30,40), video::SColorf(0.0f,0.0f,0.0f), 75.0f, 1 );
}

GraphicsManager::~GraphicsManager() {

//	m_Device->drop(); // WHY DID THIS SEG FAULT ???
}

bool GraphicsManager::tick() {
	if (GraphicsManager::s_QWidget == 0 && !m_Device->run())
		return false;
		
    //if (font == 0) {
        //font = m_GUIEnvironment->getBuiltInFont();
        //size = font->getDimension(L"Test Text");
    //}

	m_VideoDriver->beginScene(true, true, SColor(0,0,0,0));

	m_SceneManager->drawAll();

    //font->draw(L"Test Text",rect<s32>(20,668, (20 + size.Width),(668 + size.Height)), SColor(255,255,255,255));

	m_GUIEnvironment->drawAll();

	bool success = m_VideoDriver->endScene();
    //std::cout << "Irrlicht Render Success: " << success << "\n" << std::flush;
}

void GraphicsManager::resize(int w, int h) {
    dimension2d<u32> windowSize(w,h);
    m_VideoDriver->OnResize(windowSize);

    ICameraSceneNode *camera = m_SceneManager->getActiveCamera();
    if (camera != 0) {
        camera->setAspectRatio((f32)w/(f32)h);
    }
}

IMeshSceneNode* GraphicsManager::getMeshSceneNode(path meshFilepath, path textureFilepath) {
    
    //std::cout << "About to load mesh: " << meshFilepath.c_str() << "\n" << std::flush;
	IMesh* mesh = m_SceneManager->getMesh(meshFilepath);
	if (!mesh)
	{
		m_Device->drop();
		return 0;
	}
	
	IMeshSceneNode* node = m_SceneManager->addMeshSceneNode( mesh );
	node->setMaterialFlag(EMF_LIGHTING, true);
    //std::cout << "About to load texture: " << textureFilepath.c_str() << "\n" << std::flush;

    ITexture *texture = m_VideoDriver->getTexture(textureFilepath);

    // TODO Why does this fail sometimes seemingly being passed the mesh file path?
    if (texture) {
    	node->setMaterialTexture(0,  texture);
    }

	return node;
}
