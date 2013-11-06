#include "KBLevelEditor.h"
#include "OSGObjectBase.h"
#include <iostream>

using namespace osg;
using namespace lair;
using namespace std;

KBLevelEditor::KBLevelEditor( QWidget *par, QWidget *appManager, QString appID, bool useKin) : AppBase(par, appID)
{
    this->myParent = par;
    this->myID = appID;
    this->myManager = appManager;
    this->suspended = false;
    paused = false;

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(updateEnvironment()));
    connect(this,SIGNAL(destroyed()),myManager,SLOT(confirmDestroyed()));
}

void KBLevelEditor::increment(){ runtime++;}

void KBLevelEditor::config()
{
    double _pi = 3.14159265;
    this->resize(W_WIDTH,W_HEIGHT-INFOBAR_HEIGHT);
    this->move(0,0);
    this->show();

    root = new osg::Group;

    env_wrapper_ = new OSGObjectBase();
    cube_wrapper_ = new OSGObjectBase();
    wall_wrapper_ = new OSGObjectBase();
    button_wrapper_ = new OSGObjectBase();
                        
                                           


    // TRANSPARENCY ///////////////////////////////////////////////////////////
    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    // ENVIRONMENT ////////////////////////////////////////////////////////////
    std::cerr<<"<<< KBLevelEditor >>> Setting Up Environment... "<<std::endl;
    env_wrapper_ = new OSGObjectBase();
    env_wrapper_->addChild(cursor_wrapper_);
    env_wrapper_->addChild(cube_wrapper_);
    env_wrapper_->addChild(sphere_wrapper_);

    root->addChild(env_wrapper_);
    std::cerr<<"Done."<< std::endl;
    
    // CAMERA ATTACHED OBJECTS ////////////////////////////////////////////////
    std::cerr<<"<<< KBLevelEditor >>> Setting Up Camera Attached Objects... ";
    _cameraAttachedObjects = new OSGObjectBase();
    root->addChild(_cameraAttachedObjects);
    std::cerr<<"Done."<< std::endl;
    
    // GL QT WIDGET ///////////////////////////////////////////////////////////
    glWidget = new osgQt::GLWidget(this);
    glWidget = addViewWidget( createCamera(0,0,60,40,
                              glWidget, "mainCamera", false), root);
    QGridLayout* grid = new QGridLayout;
    grid->addWidget( glWidget);
    setLayout( grid );
    std::cout<<"<<< KBLevelEditor >>> Created Widget"<< std::endl;

    // STARTUP ////////////////////////////////////////////////////////////////
    _timer.start( 10 );
    _dataTimer.start(10);
    suspended = false;
}


void KBLevelEditor::updateEnvironment()
{

    UserPtrMap::iterator it;
    for(it = users.begin();it!=users.end();++it){
        LairUser* user = it->second;

        double xAng, yAng;
        Eigen::Vector3d Y(0,1,0);
        Eigen::Vector3d Z(0,0,-1);
        Eigen::Vector3d head = user->joint_pos_body_3d[HEA];
        Eigen::Vector3d neck = user->joint_pos_body_3d[NEK];
        Eigen::Vector3d rsho = user->joint_pos_body_3d[RSH];
        Eigen::Vector3d body_up, body_out, body_up_yz, body_out_xz;

        body_up = neck;
        body_out = neck.cross(rsho);

        body_up_yz = body_up.cwiseProduct(Eigen::Vector3d(0,1,1));
        body_out_xz = body_out.cwiseProduct(Eigen::Vector3d(1,0,1));

        xAng = acos(body_up_yz.dot(Y)/(body_up_yz.norm()*Y.norm()));
        yAng = acos(body_out_xz.dot(Z)/(body_out_xz.norm()*Z.norm()));

        // cout<<head<<" -- "<<xAng<<" -- "<<yAng<<endl;
        double zvel = 400;
        double xvel = .02;
        double maxx = 75;
        double maxz = 150;
        double zoff = 20, xoff = 20;
        double dx = 0,dy = 0,dz = 0;

        cout<<rsho[2]<<endl;

        if(head[0] < 0-xoff){
            theta += fabs(xvel*head[2]/maxx);
            spheres_[0]->rotateAbs(osg::Vec3(0,theta,0));
            _camServo.rotateCameraRel(_camera,cam_eye,
                                        cam_look,cam_up,cam_start, cam_off, theta);
        }else if(head[0] > 0+xoff){
            theta -= fabs(xvel*head[2]/maxx);
            spheres_[0]->rotateAbs(osg::Vec3(0,theta,0));
            _camServo.rotateCameraRel(_camera,cam_eye,
                                        cam_look,cam_up,cam_start, cam_off, theta);
        }
        

        osg::Quat R; R.makeRotate(theta,osg::Vec3(0,1,0),
                                      0,osg::Vec3(1,0,0),
                                      0,osg::Vec3(0,0,1));

        if(head[2] < -40-zoff){
            dz = fabs(zvel*head[2]/maxz);
            spheres_[0]->setPos3DRel(R * osg::Vec3(0,0,-dz));
            // spheres_[0]->rotateRel(osg::Vec3(0,-dz/400,0));
            _camServo.translateCameraRel( _camera,R*osg::Vec3(0,0,-dz),
                                                cam_eye,cam_look,cam_up);
        }else if(head[2] > -40+zoff){
            dz = fabs(zvel*head[2]/maxz);
            spheres_[0]->setPos3DRel(R * osg::Vec3(0,0,dz));
            // spheres_[0]->rotateRel(osg::Vec3(0,dz/400,0));
            _camServo.translateCameraRel( _camera,R*osg::Vec3(0,0,dz),
                                                cam_eye,cam_look,cam_up);
        }

        // if(head[0] < 0-xoff){
        //     dx-=fabs(xvel*head[0]/maxx);
        //     spheres_[0]->rotateRel(osg::Vec3(0,-dx/500/3.141,0));
        //     _camServo.rotateCameraRel(_camera,osg::Vec3(0,-dx/500/3.141,0));
        // }else if(head[0] > 0+xoff){
        //     dx+=fabs(xvel*head[0]/maxx);
        //     spheres_[0]->rotateRel(osg::Vec3(0,dx/500/3.141,0));
        //     _camServo.rotateCameraRel(_camera,osg::Vec3(0,dx/500/3.141,0));
        // }

        // if(rsho[0] < -10-xoff){
        //     theta += fabs(xvel*rsho[2]/maxx);
        //     spheres_[0]->rotateRel(osg::Vec3(theta,0,0));
        //     _camServo.rotateCameraRel(_camera,cam_eye,
        //                                 cam_look,cam_up,cam_start, cam_off, theta);
        // }else if(rsho[0] > -10+xoff){
        //     theta -= fabs(xvel*rsho[2]/maxx);
        //     spheres_[0]->rotateRel(osg::Vec3(theta,0,0));
        //     _camServo.rotateCameraRel(_camera,cam_eye,
        //                                 cam_look,cam_up,cam_start, cam_off, theta);
        // }
        // spheres_[0]->setPos3DRel(osg::Vec3(0,0,dz));
        // _camServo.translateCameraRel(_camera,osg::Vec3(dx,dy,dz));
        // spheres_[0]->setPos3DAbs(torso);

    }
}

void KBLevelEditor::pause()
{
    this->hide();
    _timer.stop();
    _dataTimer.stop();
    this->myParent->update();
    paused = true;
    std::cerr<<"<< KBLevelEditor >> Pausing"<<std::endl;
}

void KBLevelEditor::unpause()
{
    this->show();
    _timer.start(10);
    _dataTimer.start(30);
    this->update();
    paused = false;
    std::cerr<<"<< KBLevelEditor >> Un-pausing"<<std::endl;
}

void KBLevelEditor::resume()
{
    _timer.start(10);
    _dataTimer.start(30);
    this->show();
    this->glWidget->show();
    this->update();
    this->suspended = false;
    std::cerr<<"<< KBLevelEditor >> Resumed"<<std::endl;
}

void KBLevelEditor::suspend()
{
    _timer.stop();
    _dataTimer.stop();
    this->hide();
    this->glWidget->hide();
    this->myParent->update();
    this->suspended = true;
    std::cerr<<"<< KBLevelEditor >> Suspended"<<std::endl;
}

osgQt::GLWidget* KBLevelEditor::addViewWidget( osg::Camera* camera, osg::Node* scene )
{
    setCamera( camera );
    
    setSceneData( scene );
    addEventHandler( new osgViewer::StatsHandler );
    //setCameraManipulator( new osgGA::TrackballManipulator );
    
    KBLevelEditorKeyboardHandler* myFirstEventHandler =
        new KBLevelEditorKeyboardHandler();
    myFirstEventHandler->setup(this);
    addEventHandler(myFirstEventHandler); 
    setThreadingModel(SingleThreaded);
    osgQt::GraphicsWindowQt* gw = 
        dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
    
    return gw ? gw->getGLWidget() : NULL;
}

osg::Camera* KBLevelEditor::createCamera( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const std::string& name, bool windowDecoration)
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->windowName = name;
    traits->windowDecoration = windowDecoration;
    traits->x = x;
    traits->y = y;
    traits->width = w;
    traits->height = h;
    traits->doubleBuffer = true;
    traits->alpha = ds->getMinimumNumAlphaBits();
    traits->stencil = ds->getMinimumNumStencilBits();
    traits->sampleBuffers = ds->getMultiSamples();
    traits->samples = ds->getNumMultiSamples();
    
    traits->inheritedWindowData = new osgQt::GraphicsWindowQt::WindowData(NULL, QTObject);
    
    _camera = new osg::Camera;
    _camera->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera->setViewMatrixAsLookAt( osg::Vec3d( 0,1000,5000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera;
}

// Keyboard Handler ////////////////////////////
bool KBLevelEditorKeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            switch(ea.getKey())
            {
                case 'q':
                    appPtr->sendSelfTerminate();
                    break;
            } 
        }
        default:
            return false;
    }
}

void KBLevelEditorKeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v){v.visit(*this);}
void KBLevelEditorKeyboardHandler::setup(KBLevelEditor* appPt){this->appPtr = appPt;}