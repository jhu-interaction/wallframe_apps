#include "KineBallRunSandbox.h"
#include "OSGObjectBase.h"
#include <iostream>

using namespace osg;
using namespace lair;
using namespace std;

osg::Vec3 cam1_start(0,1000,5000);
osg::Vec3 cam1_look_start(0,200,0);
osg::Vec3 cam1_up_start(0,1,0);
osg::Vec3 cam1_eye(0,1000,5000);
osg::Vec3 cam1_look(0,200,0);
osg::Vec3 cam1_up(0,1,0);
osg::Vec3 cam1_off(0,1000,5000);

osg::Vec3 cam2_start(0,1000,-5000);
osg::Vec3 cam2_look_start(0,200,0);
osg::Vec3 cam2_up_start(0,1,0);
osg::Vec3 cam2_eye(0,1000,-5000);
osg::Vec3 cam2_look(0,200,0);
osg::Vec3 cam2_up(0,1,0);
osg::Vec3 cam2_off(0,1000,-5000);
double theta1 = 0;
double theta2 = 0;
double phi = 0;
double cam_theta1 = 0;
double s_yaw = 0;
bool top_level = false;
double cam_mult = 1;
double dz1 = 0;
double dz2 = 0;

#define pi 3.14159265
#define elif else if


KineBallRunSandbox::KineBallRunSandbox( QWidget *par, QWidget *appManager, 
                                        QString appID, bool useKin) : AppBase(par, appID)
{
    this->myParent = par;
    this->myID = appID;
    this->myManager = appManager;

    readConfigFile();

    this->suspended = false;
    this->useKinect = useKin;
    runtime = 0;
    paused = false;
    runs = 0;
    trail_count1 = 0;
    trail_pop1 = 0;
    trail_count2 = 0;
    trail_pop2 = 0;

    setThreadingModel(SingleThreaded);
}

void KineBallRunSandbox::increment(){ runtime++;}

void KineBallRunSandbox::readConfigFile()
{   
    QString configFileName = QString("../apps/") + this->myID + QString("/")+ this->myID + QString(".txt");
    QFile config_file(configFileName);
    if (!config_file.open(QIODevice::ReadOnly)){
        cerr<<"*** KineBallRunSandbox Config File *** >> Error with config file"<<endl;
        cerr<<"*** KineBallRunSandbox Config File *** >> File is "<<configFileName.toStdString()<<endl;
        exit(0);
    }

    cerr<<"*** KineBallRunSandbox Config File *** >> Parsing Config File ... "<<endl;
      QTextStream stream ( &config_file );
      while( !stream.atEnd() ) {
        QString line;
        QStringList lineElem;
        line = stream.readLine();
        lineElem = line.split(" ");

        if(lineElem[0] == "ASSET_DIR"){
          this->assetDir = lineElem[1];
          cerr<<this->assetDir.toStdString()<<endl;
        }
        if(lineElem[0] == "IMAGE_DIR"){
          this->imageDir = lineElem[1];
          cerr<<this->imageDir.toStdString()<<endl;
        }
        if(lineElem[0] == "TEX_IMG_DIR"){
          this->texImageDir = lineElem[1];
          cerr<<this->texImageDir.toStdString()<<endl;
        }

    }
}

osg::Vec3 KineBallRunSandbox::loadMap(double sz, double ballH)
{   
    int cell_sz = sz;
    int h = cell_sz/2;
    osg::Vec3 rval;
    int maxcol = 0,maxrow = 0;

    cerr<<"*** KineBallRunSandbox Map File *** >> Parsing Map File ... "<<endl;

    QString configFileName = QString("../apps/") + this->myID + QString("/")+ this->myID + QString("Map1.txt");
    QFile pre_config_file(configFileName);
    if (!pre_config_file.open(QIODevice::ReadOnly)){
        cerr<<"*** KineBallRunSandbox Map File *** >> Error with map file"<<endl;
        cerr<<"*** KineBallRunSandbox Map File *** >> File is "<<configFileName.toStdString()<<endl;
        exit(0);
    }

    // Preparse to get floor size
    QTextStream pre_stream ( &pre_config_file );
    int max_rows = -1, max_cols = 0;
    while( !pre_stream.atEnd() ) {
        QString line;
        charVec t_line;
        line = pre_stream.readLine();
        int len = line.size();
        if(len>max_cols) max_cols = len;
        max_rows++;
    }

    cout<<"Map Size is "<<max_rows<<" x "<<max_cols<<endl;

    double fx = max_cols*cell_sz;
    double fz = max_rows*cell_sz;

    grid_maxx = fx;
    grid_maxz = fz;

    TiledPlane* fl = new TiledPlane(osg::Vec3(0,0,0),osg::Vec3(fx,0,0),
                                    osg::Vec3(0,0,fz),osg::Vec3(fx,0,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_cols,max_rows);
    fl->setTransparency(.9);
    // TiledPlane* nwall = new TiledPlane(osg::Vec3(0,cell_sz,0),osg::Vec3(fx,cell_sz,0),
    //                                 osg::Vec3(0,0,0),osg::Vec3(fx,0,0),
    TiledPlane* nwall = new TiledPlane(osg::Vec3(-fx/2,cell_sz/2,0),osg::Vec3(fx/2,cell_sz/2,0),
                                    osg::Vec3(-fx/2,-cell_sz/2,0),osg::Vec3(fx/2,-cell_sz/2,0),
                                    &textureImages_,1,
                                    osg::Vec3(fx/2,cell_sz/2,0),max_cols,1);
    TiledPlane* swall = new TiledPlane(osg::Vec3(0,cell_sz,fz),osg::Vec3(fx,cell_sz,fz),
                                    osg::Vec3(0,0,fz),osg::Vec3(fx,0,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_cols,1);
    TiledPlane* ewall = new TiledPlane(osg::Vec3(fx,cell_sz,0),osg::Vec3(fx,cell_sz,fz),
                                    osg::Vec3(fx,0,0),osg::Vec3(fx,0,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_rows,1);
    TiledPlane* wwall = new TiledPlane(osg::Vec3(0,cell_sz,0),osg::Vec3(0,cell_sz,fz),
                                    osg::Vec3(0,0,0),osg::Vec3(0,0,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_rows,1);
    map_wrapper_->addChild(fl);
    // map_wrapper_->addChild(nwall);
    // map_wrapper_->addChild(swall);
    // map_wrapper_->addChild(ewall);
    // map_wrapper_->addChild(wwall);
    wall_wrapper_->addChild(nwall);
    wall_wrapper_->addChild(swall);
    wall_wrapper_->addChild(ewall);
    wall_wrapper_->addChild(wwall);
    
    // colliders_.push_back(nwall);
    // colliders_.push_back(swall);
    // colliders_.push_back(ewall);
    // colliders_.push_back(wwall);

    map_wrapper_->addChild(wall_wrapper_);

    QFile config_file(configFileName);
    if (!config_file.open(QIODevice::ReadOnly)){
        cerr<<"*** KineBallRunSandbox Map File *** >> Error with map file"<<endl;
        cerr<<"*** KineBallRunSandbox Map File *** >> File is "<<configFileName.toStdString()<<endl;
        exit(0);
    }

    QTextStream stream ( &config_file );
    int row = 0,col = 0;
    int level_off = 0;

    while( !stream.atEnd() ) {
        QString line;
        charVec t_line;
        line = stream.readLine();
        int len = line.size();
        col = 0;

        if(line[0] == QChar('#')){
            // row = -1;
            // col = 0;
            // level_off = -cell_sz; 
            // top_level = false;
        }

        for(int i = 0;i<len;i++){
            t_line.push_back(line[i].toLatin1());

            if(line[i] == QChar('R')){
                TexturedCube* b = new TexturedCube( cell_sz,&_assetTextures,0,
                                                    osg::Vec3(col*cell_sz,h+level_off,row*cell_sz));
                map_wrapper_->addChild(b);
                // wall_wrapper_->addChild(b);
                cout<<line[i].toLatin1();
            }else if(line[i] == QChar('B')){
                TexturedCube* b = new TexturedCube( cell_sz,&_assetTextures,0,
                                                    osg::Vec3(col*cell_sz,h+level_off,row*cell_sz));
                // map_wrapper_->addChild(b);
                wall_wrapper_->addChild(b);
                colliders_.push_back(b);

                cout<<line[i].toLatin1();
            }else if(line[i] == QChar('-')){
                cout<<line[i].toLatin1();    
            }else if(line[i] == QChar('\\')){
                cout<<line[i].toLatin1();    
            }else if(line[i] == QChar('x')){
                cout<<line[i].toLatin1();  
                p1_start_pos = osg::Vec3(col*cell_sz,ballsize,row*cell_sz);
            }else if(line[i] == QChar('y')){
                cout<<line[i].toLatin1();  
                p2_start_pos = osg::Vec3(col*cell_sz,ballsize,row*cell_sz);
            }
            col++;
        }
        minimap.push_back(t_line);
        cout<<minimap.size()<<endl;
        cout<<endl;
        row++;
    }
    int max = max_cols;
    if(max_rows>max_cols) max = max_rows;

    skybox_ = new TexturedCube( 6*max*cell_sz,max*cell_sz,&_assetTextures,
                                2,3,4,5,6,7,osg::Vec3(0,max*cell_sz/2,0));
    skybox_wrapper_->addChild(skybox_);

    cout<<endl;
    cout<<minimap.size()<<endl;
    cout<<minimap[0].size()<<endl;
    cout<<"MINIMAP"<<endl;
    int N=minimap.size();
    int M=minimap[0].size();
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < M; j++)
        {
            cout<<minimap[i][j];
        }
        cout<<endl;
    }
    return rval;
}

void KineBallRunSandbox::config()
{
    double _pi = 3.14159265;
    this->resize(W_WIDTH,W_HEIGHT);
    this->move(0,0);
    this->show();

    root = new osg::Group;
    if(useKinect) std::cerr<<"<<< KineBallRunSandbox >>> Application using kinect input."<<std::endl;
                                
    setImageDirectories();
    LoadTextures();
    setTexImageDirs();
    LoadTextureImages();                         
                                           
    // Cursors
    std::cerr<<"Adding Cursors... ";
    cursor_wrapper_ = new OSGObjectBase();
    std::cerr<<"Done."<< std::endl;

    // Ball
    ballPosLast = osg::Vec3(0,0,0);

    // // Cubes
    cube_wrapper_ = new OSGObjectBase();    
    map_wrapper_ = new OSGObjectBase();    
    wall_wrapper_ = new OSGObjectBase();                          
    trail_wrapper_1 = new OSGObjectBase();                        
    trail_wrapper_2 = new OSGObjectBase();                       
    skybox_wrapper_ = new OSGObjectBase();

    ballsize=400;
    gridsize = 16000;

    loadMap(gridsize,ballsize);

    // Players /////////////////////////////////////////////////////////
    player1_wrapper_ = new OSGObjectBase();
    player1_wrapper_->box.set(  osg::Vec3(-ballsize,-ballsize,-ballsize),
                                osg::Vec3(ballsize,ballsize,ballsize));

    //EasyParticle* particle = new EasyParticle(player1_wrapper_->getPosition());
    // player1_wrapper_->addChild(particle);

    player2_wrapper_ = new OSGObjectBase();
    player2_wrapper_->box.set(  osg::Vec3(-ballsize,-ballsize,-ballsize),
                                osg::Vec3(ballsize,ballsize,ballsize));

    SphereObject* s1_in = new SphereObject(300,Vec3(0,0,0),osg::Vec4(255.0/255.0,191.0/255.0,89.0/255.0,1));
    player1_wrapper_->addChild(s1_in);
    spheres_.push_back(s1_in);

    SphereObject* s1_out = new SphereObject(400,Vec3(0,0,0),osg::Vec4(255.0/255.0,191.0/255.0,89.0/255.0,.5));
    s1_out->triggerBehavior("ghost");
    player1_wrapper_->addChild(s1_out);
    spheres_.push_back(s1_out);

    SphereObject* s2_in = new SphereObject(300,Vec3(0,0,0),osg::Vec4(0,157.0/255.0,255.0/255.0,1));
    player2_wrapper_->addChild(s2_in);
    spheres_.push_back(s2_in);

    SphereObject* s2_out = new SphereObject(400,Vec3(0,0,0),osg::Vec4(0,157.0/255.0,255.0/255.0,.5));
    s2_out->triggerBehavior("ghost");
    player2_wrapper_->addChild(s2_out);
    spheres_.push_back(s2_out);

    // TRANSPARENCY ///////////////////////////////////////////////////////////
    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    ss->setRenderBinDetails(1, "DepthSortedBin");
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );
    // ss->setMode( GL_CULL_FACE, osg::StateAttribute::ON | osg::StateAttribute::OVERRIDE);

    // FOG ////////////////////////////////////////////////////////////////////
    osg::Fog* fog(new osg::Fog());
    fog->setMode(osg::Fog::LINEAR);
    fog->setDensity(1);
    fog->setStart(-1000);
    fog->setEnd(-5000);
    fog->setColor(osg::Vec4(.2,.2,.2,.5));
    ss->setAttributeAndModes(fog,osg::StateAttribute::ON);
    // ENVIRONMENT ////////////////////////////////////////////////////////////
    std::cerr<<"<<< KineBallRunSandbox >>> Setting Up Environment... "<<std::endl;
    _envWrapper = new OSGObjectBase();
    _envWrapper->addChild(cursor_wrapper_);
    _envWrapper->addChild(cube_wrapper_);
    _envWrapper->addChild(map_wrapper_);
    _envWrapper->addChild(wall_wrapper_);
    _envWrapper->addChild(trail_wrapper_1);
    _envWrapper->addChild(trail_wrapper_2);
    _envWrapper->addChild(skybox_wrapper_);
    _envWrapper->addChild(player1_wrapper_);
    _envWrapper->addChild(player2_wrapper_);

    root->addChild(_envWrapper);
    std::cerr<<"Done."<< std::endl;
    
    // CAMERA ATTACHED OBJECTS ////////////////////////////////////////////////
    std::cerr<<"<<< KineBallRunSandbox >>> Setting Up Camera Attached Objects... ";
    _cameraAttachedObjects = new OSGObjectBase();
    root->addChild(_cameraAttachedObjects);
    std::cerr<<"Done."<< std::endl;


    //osgViewer
    
    // GL QT WIDGET ///////////////////////////////////////////////////////////
    // glWidget1 = new osgQt::GLWidget(this);
    // glWidget1 = addViewWidget( createCamera1(0,0,60,40,
    //                           glWidget1, "mainCamera1", false), root);
    // glWidget2 = new osgQt::GLWidget(this);
    // glWidget2 = addViewWidget( createCamera2(0,0,60,40,
                               // glWidget2, "mainCamera2", false), root);

    glWidget1=addQWidget( createQCamera1(0,0,60,40,
                               "mainCamera1", false), root);

    glWidget2 = addQWidget( createQCamera2(0,0,60,40,
                                "mainCamera2", false), root);

    QGridLayout* grid = new QGridLayout;
    // grid->addWidget( glWidget1);
    // grid->addWidget( glWidget2);

    grid->addWidget( glWidget1,0,0);
    grid->addWidget( glWidget2,0,1);
    this->setLayout( grid );

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_timer, SIGNAL(timeout()), this, SLOT(increment()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(pullData()));
    connect(this,SIGNAL(destroyed()),myManager,SLOT(confirmDestroyed()));

    std::cout<<"<<< KineBallRunSandbox >>> Created Widget"<< std::endl;

    // Start Players at initial positions
    player1_wrapper_->setPos3DRel(p1_start_pos);
    player2_wrapper_->setPos3DRel(p2_start_pos);
    _camServo.translateCameraRel( _camera1,osg::Vec3(p1_start_pos),
                                                    cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel( _camera2,osg::Vec3(p2_start_pos),
                                                    cam2_eye,cam2_look,cam2_up);

    // STARTUP ////////////////////////////////////////////////////////////////
    _timer.start( 10 );
    _dataTimer.start(15);
    std::cout<<"<<< KineBallRunSandbox >>> Timers Started"<< std::endl;
    suspended = false;
    std::cout<<"<<< KineBallRunSandbox >>> Configured Successfully"<< std::endl;
}

void KineBallRunSandbox::pullData()
{
  if(this->useKinect){
    requestUserData();
    updateEnvironment();
  }
    keyboardUpdateEnv();
}

void KineBallRunSandbox::resetPlayer1()
{
    cout<<"Player 1 Reset"<<endl;
    player1_wrapper_->setPos3DAbs(Vec3(0,0,0));
    player1_wrapper_->setPos3DRel(p1_start_pos);
    _camServo.translateCameraAbs( _camera1,cam1_start,cam1_look_start,cam1_up_start,
                                    cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel( _camera1,osg::Vec3(p1_start_pos),
                                                    cam1_eye,cam1_look,cam1_up);
}

void KineBallRunSandbox::resetPlayer2()
{
    cout<<"Player 2 Reset"<<endl;
    player2_wrapper_->setPos3DAbs(Vec3(0,0,0));
    player2_wrapper_->setPos3DRel(p2_start_pos);
    _camServo.translateCameraAbs( _camera2,cam2_start,cam2_look_start,cam2_up_start,
                                    cam2_eye,cam2_look,cam2_up);
    _camServo.translateCameraRel( _camera2,osg::Vec3(p2_start_pos),
                                                    cam2_eye,cam2_look,cam2_up);
}

void KineBallRunSandbox::keyboardUpdateEnv()
{
        int twp1 = theta1/(2*pi);
        theta1 = theta1-((double)twp1)*2*pi;

        vct3 pos1 = player1_wrapper_->getPos3D();

        // Rotate P1
        player1_wrapper_->rotateAbs(osg::Vec3(0,theta1,0));
        _camServo.rotateCameraRel(_camera1,cam1_eye,
                                    cam1_look,cam1_up,cam1_start, cam1_off, theta1);
        // Translate P1                                  
        osg::Quat R1; R1.makeRotate(theta1,osg::Vec3(0,1,0),
                              0,osg::Vec3(1,0,0),
                              0,osg::Vec3(0,0,1));
        player1_wrapper_->setPos3DRel(R1*osg::Vec3(0,0,-dz1));
        _camServo.translateCameraRel( _camera1,R1*osg::Vec3(0,0,-dz1),
                                                cam1_eye,cam1_look,cam1_up);

        int twp2 = theta2/(2*pi);
        theta2 = theta2-((double)twp2)*2*pi;

        vct3 pos2 = player2_wrapper_->getPos3D();

        // Rotate P1
        player2_wrapper_->rotateAbs(osg::Vec3(0,theta2,0));
        _camServo.rotateCameraRel(_camera2,cam2_eye,
                                    cam2_look,cam2_up,cam2_start, cam2_off, theta2);
        // Translate P1                                  
        osg::Quat R2; R2.makeRotate(theta2,osg::Vec3(0,1,0),
                              0,osg::Vec3(1,0,0),
                              0,osg::Vec3(0,0,1));
        player2_wrapper_->setPos3DRel(R2*osg::Vec3(0,0,dz2));
        _camServo.translateCameraRel( _camera2,R2*osg::Vec3(0,0,dz2),
                                                cam2_eye,cam2_look,cam2_up);
        
        testCollisionP1();
        testCollisionP2();

        updateTrail(player1_wrapper_->getPosition(),trail_wrapper_1,&lightTrail_1,
                    trail_count1,trail_pop1,dz1);

        updateTrail(player2_wrapper_->getPosition(),trail_wrapper_2,&lightTrail_2,
                    trail_count2,trail_pop2,dz2);

}

void KineBallRunSandbox::testCollisionP1()
{
    // Test self trail collision
    for(int i = 0;i<lightTrail_1.size()-20;i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_1[i]->getBound();
        if(sb.intersects(lb)){
            cerr<<"P1 Collided"<<endl;
            resetPlayer1();
            theta1 = 0;
            lightTrail_1.clear();
            trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
            trail_count1 = 0;
            trail_pop1 = 0;
            break;
        }
    }
    
    // Test p2 trail collision
    for(int i = 0;i<lightTrail_2.size();i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_2[i]->getBound();
        if(sb.intersects(lb)){
            cerr<<"P1 Collided"<<endl;
            resetPlayer1();
            theta1 = 0;
            lightTrail_1.clear();
            trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
            trail_count1 = 0;
            trail_pop1 = 0;
            break;
        }
    } 
    for(int i=0;i<colliders_.size();i++)
    {
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = player1_wrapper_->calcBB();
        // const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        // std::cerr<<"P1 BB: "<<sb.radius()<<"  COLIDER BB: "<<thebox.radius()<<" "<<i<<" of " <<colliders_.size()<<endl;
        // osg::Vec3 d = thebox.center()-sb.center();
        // std::cerr<<"Distance "<<d.length()<<endl;
        // std::cerr<<"Center"<<","<<thebox.center()[0]/8000<<","<<thebox.center()[1]/8000<<","<<thebox.center()[2]/8000<<endl;
        if(sb.intersects(thebox))
        {
            // cerr<<"P1 Collided"<<endl;
            resetPlayer1();
            theta1 = 0;
            lightTrail_1.clear();
            trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
            trail_count1 = 0;
            trail_pop1 = 0;
            break;
        }
    }

    osg::Vec3 pos = player1_wrapper_->getPosition();
    if(pos[0] > grid_maxx || pos[0] < 0 || pos[2] > grid_maxz || pos[2] < 0){
        resetPlayer1();
        theta1 = 0;
        lightTrail_1.clear();
        trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
        trail_count1 = 0;
        trail_pop1 = 0;
    }
}

void KineBallRunSandbox::testCollisionP2()
{
    // Test self trail collision
    for(int i = 0;i<lightTrail_2.size()-20;i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_2[i]->getBound();
        if(sb.intersects(lb)){
            cerr<<"P2 Collided"<<endl;
            resetPlayer2();
            theta2 = 0;
            lightTrail_2.clear();
            trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
            trail_count2 = 0;
            trail_pop2 = 0;
            break;
        }
    }
    
    // Test p1 trail collision
    for(int i = 0;i<lightTrail_1.size();i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_1[i]->getBound();
        if(sb.intersects(lb)){
            cerr<<"P2 Collided"<<endl;
            resetPlayer2();
            theta2 = 0;
            lightTrail_2.clear();
            trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
            trail_count2 = 0;
            trail_pop2 = 0;
            break;
        }
    }
     
    for(int i=0;i<colliders_.size();i++)
    {
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = player2_wrapper_->calcBB();
        // const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        if(sb.intersects(thebox))
        {
            // cerr<<"P2 Collided"<<endl;
            resetPlayer2();
            theta2 = 0;
            lightTrail_2.clear();
            trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
            trail_count2 = 0;
            trail_pop2 = 0;
            break;
        }
    }

    osg::Vec3 pos = player2_wrapper_->getPosition();
    if(pos[0] > grid_maxx || pos[0] < 0 || pos[2] > grid_maxz || pos[2] < 0){
        resetPlayer2();
        theta2 = 0;
        lightTrail_2.clear();
        trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
        trail_count2 = 0;
        trail_pop2 = 0;
    }
}

void KineBallRunSandbox::updateTrail(osg::Vec3 pos,OSGObjectBase* wrapper, TexPlaneList* trail, 
                                    int& count, int& pop, double dz)
{
    // Update Trail
    count+=dz;
    // cout<<trail_count1<<endl;
    if(count>400){
        // cout<<"Added Trail"<<endl;
        osg::Vec3 sp = pos;
        
        if(pop == 0){
            TexturedPlane* t = new TexturedPlane(sp+Vec3(0,400,0),sp+Vec3(0,400,400),
                                                 sp+Vec3(0,-400,0),sp+Vec3(0,-400,400),
                                                 &_assetTextures,1);
            trail->push_back(t);
            wrapper->addChild(t);
        }else{
            TexturedPlane* t = new TexturedPlane(sp+Vec3(0,400,0),trail->last()->ul(),
                                                 sp+Vec3(0,-400,0),trail->last()->ll(),
                                                 &_assetTextures,1);
            trail->push_back(t);
            wrapper->addChild(t);
        }
        // Reset counter
        count = 0;
        // Pop from trail when too long
        pop++;
        // cout<<trail_pop1<<endl;
        if(pop > 600){
            if(!trail->isEmpty());{
                wrapper->removeChild(trail->at(0));
                trail->pop_front();
                // cout<<"Removed Trail"<<endl;
            }
            
        }
    }
}


void KineBallRunSandbox::pause()
{
    this->hide();
    _timer.stop();
    _dataTimer.stop();
    this->myParent->update();
    paused = true;
    std::cerr<<"<< KineBallRunSandbox >> Pausing"<<std::endl;
}

void KineBallRunSandbox::unpause()
{
    this->show();
    _timer.start(10);
    _dataTimer.start(30);
    this->update();
    paused = false;
    std::cerr<<"<< KineBallRunSandbox >> Un-pausing"<<std::endl;
}

void KineBallRunSandbox::resume()
{
    _timer.start(10);
    _dataTimer.start(30);
    this->show();
    // this->glWidget->show();
    this->update();
    this->suspended = false;
    std::cerr<<"<< KineBallRunSandbox >> Resumed"<<std::endl;
}

void KineBallRunSandbox::suspend()
{
    _timer.stop();
    _dataTimer.stop();
    this->hide();
    // this->glWidget->hide();
    this->myParent->update();
    this->suspended = true;
    std::cerr<<"<< KineBallRunSandbox >> Suspended"<<std::endl;
}

osgQt::GLWidget* KineBallRunSandbox::addQWidget( osg::Camera* camera, osg::Node* scene )
{
    osgViewer::View* view=new osgViewer::View;
    view->setCamera( camera );
    addView(view);
    
    view->setSceneData( scene );
    view->addEventHandler( new osgViewer::StatsHandler );
    //setCameraManipulator( new osgGA::TrackballManipulator );
    
    KineBallRunSandboxKeyboardHandler* myFirstEventHandler =
        new KineBallRunSandboxKeyboardHandler();
    myFirstEventHandler->setup(this);
    view->addEventHandler(myFirstEventHandler); 
    //view->setThreadingModel(SingleThreaded);
    osgQt::GraphicsWindowQt* gw = 
        dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
    
    return gw ? gw->getGLWidget() : NULL;
}

osg::Camera* KineBallRunSandbox::createCamera1( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const std::string& name, bool windowDecoration)
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
    
    _camera1 = new osg::Camera;
    _camera1->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    _camera1->setSmallFeatureCullingPixelSize(30.f);
    _camera1->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera1->setViewMatrixAsLookAt( osg::Vec3d( 0,1000,5000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera1->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera1->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera1->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera1;
}

osg::Camera* KineBallRunSandbox::createQCamera1( int x, int y, int w, int h, const std::string& name, bool windowDecoration)
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
    
    //traits->inheritedWindowData = new osgQt::GraphicsWindowQt::WindowData(NULL, QTObject);
    
    _camera1 = new osg::Camera;
    _camera1->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    _camera1->setSmallFeatureCullingPixelSize(30.f);
    _camera1->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera1->setViewMatrixAsLookAt( osg::Vec3d( 0,1000,5000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera1->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera1->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera1->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera1;
}

osg::Camera* KineBallRunSandbox::createQCamera2( int x, int y, int w, int h,  const std::string& name, bool windowDecoration)
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
    
    //traits->inheritedWindowData = new osgQt::GraphicsWindowQt::WindowData(NULL, QTObject);
    
    _camera2 = new osg::Camera;
    _camera2->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    _camera2->setSmallFeatureCullingPixelSize(30.f);
    _camera2->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera2->setViewMatrixAsLookAt( osg::Vec3d( 0,500,2000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera2->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera2->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera2->setProjectionMatrixAsPerspective(45.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera2;
}

// Keyboard Handler ////////////////////////////
bool KineBallRunSandboxKeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
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
                // Camera Zoom
                case 'p':
                    cam_mult = 100;
                    break;
                case 'o':
                    cam_mult = 1;
                    break;

                case 'z':
                    dz1 = 0;
                    break;
                case 'x':
                    dz1 = 25;
                case 'c':
                    dz1 = 600;
                    break;
                case 'a':
                    theta1 += .025;
                    break;
                case 'd':
                    theta1 -= .025;
                    break;

                case 'b':
                    dz2 = 0;
                    break;
                case 'n':
                    dz2 = 25;
                case 'm':
                    dz2 = 600;
                    break;
                case 'h':
                    theta2 += .025;
                    break;
                case 'k':
                    theta2 -= .025;
                    break;
                case '4':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(-1000*cam_mult,0,0),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '6':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(1000*cam_mult,0,0),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '8':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,0,-1000*cam_mult),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '2':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,0,1000*cam_mult),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '1':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,1000*cam_mult,0),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '3':
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,-1000*cam_mult,0),
                                                                    cam1_eye,cam1_look,cam1_up);
                    break;
                case '7':
                    {   
                        theta1 += .03;
                        appPtr->_camServo.rotateCameraRel(appPtr->_camera1,cam1_eye,
                                                   cam1_look,cam1_up,cam1_start, cam1_off, theta1);
                        break;
                    } break;
                case '9':
                    {   
                        theta1 -= .03;
                        appPtr->_camServo.rotateCameraRel(appPtr->_camera1,cam1_eye,
                                                   cam1_look,cam1_up,cam1_start, cam1_off, theta1);
                        break;
                    } break;
                    break; 
            } 
        }
        default:
            return false;
    }
}

void KineBallRunSandboxKeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v){v.visit(*this);}
void KineBallRunSandboxKeyboardHandler::setup(KineBallRunSandbox* appPt){this->appPtr = appPt;}

void KineBallRunSandbox::setImageDirectories()
{
    cout<<"Getting Image Directories"<<endl;
    // Asset Textures //
    QDir asset_dir(this->assetDir);
    QStringList assetFiles = asset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<assetFiles.count(); i++){
        this->assetPaths << asset_dir.absoluteFilePath(assetFiles[i]);
    }
}

void KineBallRunSandbox::LoadTextures()
{
    std::cout<<">>> Loading Images... "<<endl;
    osg::Image* img;    
    for (int i = 0; i < this->assetPaths.size(); ++i){
        img = osgDB::readImageFile(this->assetPaths.at(i).toStdString());
        cout<<">>> Loading Image: "<<this->assetPaths.at(i).toStdString()<<endl;
        osg::TextureRectangle* t = new osg::TextureRectangle(img);
        // t->setFilter(osg::Texture::MIN_FILTER,osg::Texture::LINEAR_MIPMAP_LINEAR);
        // t->setFilter(osg::Texture::MAG_FILTER,osg::Texture::LINEAR);
        // t->setMaxAnisotropy(8);  
        this->_assetTextures.push_back(t);
    }
    std::cout<<">>> Done"<<std::endl;
}

void KineBallRunSandbox::setTexImageDirs()
{

    cout<<"Getting Texture Directories"<<endl;
    QDir tex_img_dir(this->texImageDir);
    QStringList tex_img_rel_paths = tex_img_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<tex_img_rel_paths.count(); i++){
        this->textureImagePaths << tex_img_dir.absoluteFilePath(tex_img_rel_paths[i]);
    }
}

void KineBallRunSandbox::LoadTextureImages()
{
    std::cout<<"Loading Textures"<<endl;
    osg::Image* img;    
    for (int i = 0; i < this->textureImagePaths.size(); ++i){
        img = osgDB::readImageFile(this->textureImagePaths.at(i).toStdString());
        cout<<">>> Loading Texture: "<<this->textureImagePaths.at(i).toStdString()<<endl; 
        textureImages_.push_back(img);  
        this->_assetTextures.push_back(new osg::TextureRectangle(img));
        std::cout<<". ";
    }
    std::cout<<" Done"<<std::endl;    
}


