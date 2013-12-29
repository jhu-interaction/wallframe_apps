// #include "KineBallRun.h"
// #include "OSGObjectBase.h"
#include <modulair_app_tron/KineBallRun.h>
// #include <modulair_app_tron/KineBallRunSandbox.h>
// #include <modulair_app_tron/KBSkyBox.h>  
#include <modulair_osg_tools/vector_conversions.h>
#include <modulair_osg_tools/osg_object_base.h>
#include <iostream>
#include <ros/ros.h>
#define pi 3.14159265
#define elif else if
#define MAX_SCORE 10
#define TOTAL_NUM_MAPS 3

using namespace osg;
using namespace modulair;
using namespace std;

osg::Vec3 cam1_start(0,2000,7000);
osg::Vec3 cam1_look_start(0,300,0);
osg::Vec3 cam1_up_start(0,1,0);
osg::Vec3 cam1_eye(0,2000,7000);
osg::Vec3 cam1_look(0,0,0);
osg::Vec3 cam1_up(0,1,0);
osg::Vec3 cam1_off(0,2000,7000);

osg::Vec3 cam2_start(0,2000,-7000);
osg::Vec3 cam2_look_start(0,300,0);
osg::Vec3 cam2_up_start(0,1,0);
osg::Vec3 cam2_eye(0,2000,-7000);
osg::Vec3 cam2_look(0,0,0);
osg::Vec3 cam2_up(0,1,0);
osg::Vec3 cam2_off(0,2000,-7000);

double theta1 = 0;
double theta2 = 0;
double phi = 0;
double cam_theta1 = 0;
double s_yaw = 0;
bool top_level = false;
double cam_mult = 1;
double dz1 = 0;
double dz2 = 0;
double hud_sz = 2000;
double hud_off = 800;

osg::Quat R1_old,R2_old;

bool p1pullback = false;
bool p2pullback = false;


int maptimer = 0;



// ASK 
namespace modulair{
// ASK build config 
// ASK pause resume updateApp
KineBallRun::KineBallRun(std::string app_name, ros::NodeHandle nh, int event_deque_size) :wallframe::WallframeAppBaseQt(app_name, nh, event_deque_size){

    runtime = 0;
    paused = false;
    runs = 0;
    trail_count1 = 0;
    trail_pop1 = 0;
    trail_count2 = 0;
    trail_pop2 = 0;

    astate = ASTATE_START_INSTRUCTIONS;
    p1_state = PSTATE_RUN;
    p2_state = PSTATE_RUN;

    p1id = -1;
    p2id = -1;
    p1id_prev = -1;
    p2id_prev = -1;

    p1_reset_timer.setSingleShot(true);
    p2_reset_timer.setSingleShot(true);

    _mapChangeTimer.setSingleShot(true);
    _instructionTimer.setSingleShot(true);
    _mandatoryTimer.setSingleShot(true);
    QObject::connect(&_mandatoryTimer, SIGNAL(timeout()),this,SLOT(dismissability()));

    QObject::connect(&_mapChangeTimer, SIGNAL(timeout()),this,SLOT(mapChangeResume()));

    QObject::connect(&_countdownTimer, SIGNAL(timeout()),this,SLOT(updateCountdown()));

    QObject::connect(&_instructionTimer, SIGNAL(timeout()),this,SLOT(hideHelp()));

    QObject::connect(&p1_reset_timer, SIGNAL(timeout()),this,SLOT(resetP1Pos()));
    QObject::connect(&p2_reset_timer, SIGNAL(timeout()),this,SLOT(resetP2Pos()));

    setThreadingModel(SingleThreaded);
            }


void KineBallRun::increment(){ runtime++;}

bool KineBallRun::build()
{   
    std::string asset_path;

    if (!node_.getParam("/modulair/apps/modulair_app_tron/paths/assets", asset_path)) {
      ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
        name_.c_str(), node_.getNamespace().c_str());
      return false;
    } else{
      asset_path_ = QString(asset_path.c_str());

      if (!QDir(asset_path_).exists()) {
	ROS_ERROR("Modulair%s: Asset path does not exists: %s",
		name_.c_str(), asset_path.c_str());
	return false;
      }
    }

    ROS_WARN_STREAM("TronApp:  Asset path is [" << this->asset_path_.toStdString() << "]");

    this->assetDir = this->asset_path_;
    this->tagAssetDir = this->asset_path_ + QString("/tags");
    this->imageDir = this->asset_path_ + QString("/images");
    this->texImageDir = this->asset_path_ + QString("/tex");
    this->mapDir = this->asset_path_ + QString("/maps");

    config();
    return true;
}

osg::Vec3 KineBallRun::loadMap(double sz, double ballH, int mapnum)
{   

    map_wrapper_ = new OSGObjectBase();    
    wall_wrapper_ = new OSGObjectBase();   
    minimap.clear();
    colliders_.clear();
    int cell_sz = sz;
    int h = cell_sz/2;
    osg::Vec3 rval;
    int maxcol = 0,maxrow = 0;

    
    QString configFileName = this->mapDir + QString("/")+ QString("Map") +  QString("%1").arg(mapnum)  +QString(".txt");
    QFile pre_config_file(configFileName);
    if (!pre_config_file.open(QIODevice::ReadOnly)){
        ROS_WARN_STREAM("<<< KineBallRun >>>  Error with map file");
        ROS_WARN_STREAM("<<< KineBallRun >>>  File is "<<configFileName.toStdString());
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

    ROS_INFO_STREAM("<<< KineBallRun >>> Map Size is "<<max_rows<<" x "<<max_cols);

    double fx = max_cols*cell_sz;
    double fz = max_rows*cell_sz;

    grid_maxx = fx;
    grid_maxz = fz;

    TiledPlane* fll = new TiledPlane(osg::Vec3(0,0,0),osg::Vec3(fx/2,0,0),
                                    osg::Vec3(0,0,fz),osg::Vec3(fx/2,0,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_cols*0.6,max_cols*0.6);
    fll->setTransparency(.9);
    TiledPlane* flr = new TiledPlane(osg::Vec3(fx/2,0,0),osg::Vec3(fx,0,0),
                                    osg::Vec3(fx/2,0,fz),osg::Vec3(fx,0,fz),
                                    &textureImages_,2,
                                    osg::Vec3(0,0,0),max_cols*0.6,max_cols*0.6);
    flr->setTransparency(.9);
    TiledPlane* fl2l = new TiledPlane(osg::Vec3(0,-cell_sz,0),osg::Vec3(fx/2,-cell_sz,0),
                                    osg::Vec3(0,-cell_sz,fz),osg::Vec3(fx/2,-cell_sz,fz),
                                    &textureImages_,1,
                                    osg::Vec3(0,0,0),max_cols*0.6,max_cols*0.6);
    TiledPlane* fl2r = new TiledPlane(osg::Vec3(fx/2,-cell_sz,0),osg::Vec3(fx,-cell_sz,0),
                                    osg::Vec3(fx/2,-cell_sz,fz),osg::Vec3(fx,-cell_sz,fz),
                                    &textureImages_,2,
                                    osg::Vec3(0,0,0),max_cols*0.6,max_cols*0.6);
    // TiledPlane* nwall = new TiledPlane(osg::Vec3(0,cell_sz,0),osg::Vec3(fx,cell_sz,0),
    //                                 osg::Vec3(0,0,0),osg::Vec3(fx,0,0),
    TiledPlane* nwall = new TiledPlane(osg::Vec3(-fx/2,cell_sz/2,0),osg::Vec3(fx/2,cell_sz/2,0),
                                    osg::Vec3(-fx/2,-cell_sz/2,0),osg::Vec3(fx/2,-cell_sz/2,0),
                                    &textureImages_,3,
                                    osg::Vec3(fx/2,cell_sz/2,0),max_cols*0.6,1);
    TiledPlane* swall = new TiledPlane(osg::Vec3(0,cell_sz,fz),osg::Vec3(fx,cell_sz,fz),
                                    osg::Vec3(0,0,fz),osg::Vec3(fx,0,fz),
                                    &textureImages_,3,
                                    osg::Vec3(0,0,0),max_cols*0.6,1);
    TiledPlane* ewall = new TiledPlane(osg::Vec3(fx,cell_sz,0),osg::Vec3(fx,cell_sz,fz),
                                    osg::Vec3(fx,0,0),osg::Vec3(fx,0,fz),
                                    &textureImages_,3,
                                    osg::Vec3(0,0,0),max_cols*0.6,1);
    TiledPlane* wwall = new TiledPlane(osg::Vec3(0,cell_sz,0),osg::Vec3(0,cell_sz,fz),
                                    osg::Vec3(0,0,0),osg::Vec3(0,0,fz),
                                    &textureImages_,3,
                                    osg::Vec3(0,0,0),max_cols*0.6,1);
    map_wrapper_->addChild(fll);
    map_wrapper_->addChild(flr);
    map_wrapper_->addChild(fl2l);
    map_wrapper_->addChild(fl2r);
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

    
    root->addChild(map_wrapper_);
    root->addChild(wall_wrapper_);

    QFile config_file(configFileName);
    if (!config_file.open(QIODevice::ReadOnly)){
        ROS_WARN_STREAM("<<< KineBallRun >>>  Error with map file");
        ROS_WARN_STREAM("<<< KineBallRun >>>  File is "<<configFileName.toStdString());
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
                TexturedCube* b;
                if(i<len/2)
                {
                    b = new TexturedCube( cell_sz,&_assetTextures,0,
                                                    osg::Vec3(col*cell_sz,h+level_off,row*cell_sz));
                }
                else
                {
                    b = new TexturedCube( cell_sz,&_assetTextures,15,
                                                    osg::Vec3(col*cell_sz,h+level_off,row*cell_sz));
                }
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
                                3,4,5,6,7,8,osg::Vec3(0,max*cell_sz/2,0));
    // skybox_wrapper_->addChild(skybox_);

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
// ASK 
void KineBallRun::config()
{
    double _pi = 3.14159265;
    // this->resize(this->width_,this->height_-INFOBAR_HEIGHT);
    // this->move(0,0);
    this->show();

    dismissable=false;
    root = new osg::Group;

    actual_root = new osg::Group;

    actual_root->addChild(root);
    if(useKinect) ROS_WARN_STREAM("<<< KineBallRun >>> Application using kinect input.");
                                
    setImageDirectories();
    LoadTextures();
    setTexImageDirs();
    LoadTextureImages();                         
                                           
    // Cursors
    ROS_INFO_STREAM("<<< KineBallRun >>> Adding Cursors.");
    cursor_wrapper_ = new OSGObjectBase();

    // Ball
    ballPosLast = osg::Vec3(0,0,0);

    // // Cubes
    cube_wrapper_ = new OSGObjectBase();    
    map_wrapper_ = new OSGObjectBase();    
    wall_wrapper_ = new OSGObjectBase();                          
    trail_wrapper_1 = new OSGObjectBase();                        
    trail_wrapper_2 = new OSGObjectBase();                       
    skybox_wrapper_ = new OSGObjectBase();


    p1_hud_wrapper = new OSGObjectBase();
    // ROS_INFO_STREAM("<<< KineBallRun >>> 427.");
 
    p1arrow = new TexturedPlane( osg::Vec3(hud_sz,0,-hud_sz),osg::Vec3(-hud_sz,0,-hud_sz),
                                osg::Vec3(hud_sz,0,hud_sz),osg::Vec3(-hud_sz,0,hud_sz),
                                &_assetTextures,12);

    ROS_INFO_STREAM("<<< KineBallRun >>> 431.");
 
    p1winlose = new TexturedPlane( osg::Vec3(-hud_sz,0,-hud_sz),osg::Vec3(hud_sz,0,-hud_sz),
                                osg::Vec3(-hud_sz,0,hud_sz),osg::Vec3(hud_sz,0,hud_sz),
                                &_assetTextures,17);
    ROS_INFO_STREAM("<<< KineBallRun >>> 436.");

    p1winlose->setHidden();
    // p1arrow->setPosition(osg::Vec3(0,ballsize*2,0));
    ROS_INFO_STREAM("<<< KineBallRun >>> 440.");

    p1_hud_wrapper->addChild(p1arrow);
    ROS_INFO_STREAM("<<< KineBallRun >>> 443.");
    p1_hud_wrapper->addChild(p1winlose);
    ROS_INFO_STREAM("<<< KineBallRun >>> 445.");

    p2_hud_wrapper = new OSGObjectBase();
    p2arrow = new TexturedPlane( osg::Vec3(-hud_sz,0,hud_sz),osg::Vec3(hud_sz,0,hud_sz),
                                osg::Vec3(-hud_sz,0,-hud_sz),osg::Vec3(hud_sz,0,-hud_sz),
                                &_assetTextures,13);

    p2winlose = new TexturedPlane( osg::Vec3(hud_sz,0,hud_sz),osg::Vec3(-hud_sz,0,hud_sz),
                                osg::Vec3(hud_sz,0,-hud_sz),osg::Vec3(-hud_sz,0,-hud_sz),
                                &_assetTextures,18);
    p2winlose->setHidden();
    // p2arrow->setPosition(osg::Vec3(0,ballsize*2,0));

    p2_hud_wrapper->addChild(p2arrow);
    p2_hud_wrapper->addChild(p2winlose);
    // SphereObject* sss = new SphereObject(25,Vec3(0,0,0),osg::Vec4(255.0/255.0,191.0/255.0,89.0/255.0,1));
    // p1_hud_wrapper->addChild(sss);
    p1hud = new TexturedPlane(osg::Vec3(300,500,300),osg::Vec3(600,500,300),
                                            osg::Vec3(300,100,300),osg::Vec3(600,100,300),
                                            &_tagAssetTextures,26);
    // p1hud->setPosition(osg::Vec3(0,2230,7740));
    osg::Vec3 c1s = cam1_start, c1l = cam1_look_start;
    double vang = (c1s*c1l);
    double vnorm = (c1s.length()*c1l.length());

    double p1hudang = acos(vang/vnorm);
    printv(cam1_start);
    printv(cam1_look_start);

    p1hud->rotateAbs(osg::Vec3(-((_pi/2)-p1hudang),0,0));

    p1_hud_wrapper->addChild(p1hud);


    p2hud = new TexturedPlane(osg::Vec3(600,500,300),osg::Vec3(300,500,300),
                                            osg::Vec3(600,100,300),osg::Vec3(300,100,300),
                                            &_tagAssetTextures,25);
    // p1hud->setPosition(osg::Vec3(0,2230,7740));
    osg::Vec3 c2s = cam2_start, c2l = cam2_look_start;
    double vang2 = (c2s*c2l);
    double vnorm2 = (c2s.length()*c2l.length());

    double p2hudang = acos(vang2/vnorm2);
    printv(cam2_start);
    printv(cam2_look_start);

    p2hud->rotateAbs(osg::Vec3(-((_pi/2)-p2hudang),0,0));

    p2_hud_wrapper->addChild(p2hud);

    // p2_hud_wrapper = new OSGObjectBase();

    ballsize=400;
    gridsize = 25000;

    mapnum =1;
    totalmaps=TOTAL_NUM_MAPS;
    maxscore = MAX_SCORE;

    loadMap(gridsize,ballsize,mapnum);

    _partWrapper = new OSGObjectBase();
    root->addChild(_partWrapper);

    // Players /////////////////////////////////////////////////////////
    player1_wrapper_ = new OSGObjectBase();
    player1_wrapper_->box.set(  osg::Vec3(-ballsize,-ballsize,-ballsize),
                                osg::Vec3(ballsize,ballsize,ballsize));
    player1_wrapper_->addChild(p1_hud_wrapper);

    player2_wrapper_ = new OSGObjectBase();
    player2_wrapper_->box.set(  osg::Vec3(-ballsize,-ballsize,-ballsize),
                                osg::Vec3(ballsize,ballsize,ballsize));
    player2_wrapper_->addChild(p2_hud_wrapper);

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
    ROS_INFO_STREAM("<<< KineBallRun >>> Setting Up Environment.");
    _envWrapper = new OSGObjectBase();
    _envWrapper->addChild(cursor_wrapper_);
    _envWrapper->addChild(cube_wrapper_);
    // _envWrapper->addChild(map_wrapper_);
    // _envWrapper->addChild(wall_wrapper_);
    _envWrapper->addChild(trail_wrapper_1);
    _envWrapper->addChild(trail_wrapper_2);
    _envWrapper->addChild(skybox_wrapper_);
    _envWrapper->addChild(player1_wrapper_);
    _envWrapper->addChild(player2_wrapper_);


    coll_sphere = new SphereObject(2000,Vec3(-9000,0,-9000),osg::Vec4(0,157.0/255.0,255.0/255.0,.5));
    _envWrapper->addChild(coll_sphere);


    // _envWrapper->addChild(p1_hud_wrapper);
    // _envWrapper->addChild(p2_hud_wrapper);

    root->addChild(_envWrapper);

    
    // CAMERA ATTACHED OBJECTS ////////////////////////////////////////////////
    ROS_INFO_STREAM("<<< KineBallRun >>> Setting Up Camera Attached Objects.");
    _cameraAttachedObjects = new OSGObjectBase();
    root->addChild(_cameraAttachedObjects);

    glWidget1=addQWidget( createQCamera1(0,0,60,40,
                               "mainCamera1", false), root);

    glWidget2 = addQWidget( createQCamera2(0,0,60,40,
                                "mainCamera2", false), root);

    QGridLayout* grid = new QGridLayout;
    

    grid->addWidget( glWidget1,0,0);
    grid->addWidget( glWidget2,0,1);
    this->setLayout( grid );

    center = new QLabel(this);
    center->raise();
    center->show();
    int infobar_height = this->height_*.05;
    center->resize(infobar_height/4,this->height_-infobar_height);
    center->setStyleSheet("background-color:#00D5FF");
    center->move(this->width_/2-infobar_height/8,0);

    score = new QLabel(this);
    score->raise();
    score->show();
    score->resize(this->width_/8,this->height_/8);
    score->setStyleSheet("border-style:solid;border-width:15px;border-color:#00D5FF;background-color:#222222");
    score->move(this->width_/2-this->width_/16,0);

    int mapw = this->width_/2;
    int maph = this->height_/3 - infobar_height;

    QFont textFont;
      textFont.setPixelSize(this->width_/64);
      textFont.setBold(true);

    p1scoreboard = new QLabel(this);
    p1scoreboard->move(this->width_/2-this->width_/16,0);
    p1scoreboard->resize(this->width_/16,this->height_/8);
    p1scoreboard->raise();
    p1scoreboard->show();
    p1scoreboard->setFont(textFont);
    p1scoreboard->setText("Score:\n0/10");
    p1scoreboard->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    p1scoreboard->setStyleSheet("color:#00D5FF");

    p2scoreboard = new QLabel(this);
    p2scoreboard->move(this->width_/2,0);
    p2scoreboard->resize(this->width_/16,this->height_/8);
    p2scoreboard->raise();
    p2scoreboard->show();
    p2scoreboard->setFont(textFont);
    p2scoreboard->setText("Score:\n0/10");
    p2scoreboard->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    p2scoreboard->setStyleSheet("color:#00D5FF");

    mapChangeImg = QPixmap(assetPaths[16]);

    mapchange = new QLabel(this);
    mapchange->setPixmap(mapChangeImg);
    mapchange->setScaledContents(true);
    mapchange->move(0,-400+this->height_/3);
    mapchange->resize(this->width_,400);
    mapchange->hide();

    countdown = new CountDownWidget(this);
    countdown->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    countdown->move(0,-400+this->height_/3);
    countdown->resize(this->width_,400);
    countdown->hide();

        // Help Screen
    QFont helpFont;
    helpFont.setPixelSize(this->width_/85);
    helpFont.setBold(true);
    QFont labelFont;
    labelFont.setPixelSize(this->width_/55);
    labelFont.setBold(true);

    help_container = new QWidget(this);
    help_container->move(0,2*this->height_/3);
    help_container->resize(this->width_,this->height_/3);
    help_container->show();
    help_container->raise();
    
    h1 = new QLabel(this);
    h1->setFont(helpFont);
    h1->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    h1->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    h1->setText("- Make the other player crash!\n- Avoid crashing into anything!\n- Use your tail to cut off\nyour opponent!\n- First to 10 points WINS!");
    
    h2 = new QLabel(this);
    h2->setFont(helpFont);
    h2->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    h2->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    h2->setText("- STAND to the left or right of center\n- Lean FORWARD to accelerate\n- Lean BACK to slow down\n- Lean LEFT or RIGHT to turn\n- Put your hands together\nto skip instructions");
    
    h3 = new QLabel(this);
    h3->setFont(helpFont);
    h3->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    h3->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    h3->setText("- STAND to the left or right of center\n- Lean FORWARD to accelerate\n- Lean BACK to slow down\n- Lean LEFT or RIGHT to turn\n- Put your hands together\nto skip instructions");

    h4 = new QLabel(this);
    h4->setFont(helpFont);
    h4->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    h4->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    h4->setText("- Make the other player crash!\n- Avoid crashing into anything!\n- Use your tail to cut off\nyour opponent!\n- First to 10 points WINS!");

    hl1 = new QLabel(this);
    hl1->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    hl1->setFont(labelFont);
    hl1->setText("How to Play");
    hl1->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    hl2 = new QLabel(this);
    hl2->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    hl2->setFont(labelFont);
    hl2->setText("Controls");
    hl2->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    hl3 = new QLabel(this);
    hl3->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    hl3->setFont(labelFont);
    hl3->setText("Controls");
    hl3->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
    hl4 = new QLabel(this);
    hl4->setStyleSheet("color:#ffffff;background-color:#00D5FF");
    hl4->setFont(labelFont);
    hl4->setText("How to Play");
    hl4->setAlignment(Qt::AlignCenter | Qt::AlignCenter);

    help_layout = new QGridLayout;
    help_layout->addWidget(hl1,0,0);
    help_layout->addWidget(hl2,0,1);
    help_layout->addWidget(hl3,0,2);
    help_layout->addWidget(hl4,0,3);
    help_layout->addWidget(h1,1,0);
    help_layout->addWidget(h2,1,1);
    help_layout->addWidget(h3,1,2);
    help_layout->addWidget(h4,1,3);
    help_layout->setRowMinimumHeight(1,this->height_/4);
    help_container->setLayout(help_layout);

    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_timer, SIGNAL(timeout()), this, SLOT(increment()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(keyboardUpdateEnv()));
    // connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(pullData()));
    // connect(this,SIGNAL(destroyed()),myManager,SLOT(confirmDestroyed()));
    connect( &_mandatoryTimer, SIGNAL(timeout()), this, SLOT(dismissability()));

    ROS_INFO_STREAM("<<< KineBallRun >>> Created Widget");

    // Start Players at initial positions
    player1_wrapper_->setPos3DRel(p1_start_pos);
    player2_wrapper_->setPos3DRel(p2_start_pos);
    _camServo.translateCameraRel(   _camera1,osg::Vec3(p1_start_pos),
                                    cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel(   _camera2,osg::Vec3(p2_start_pos),
                                    cam2_eye,cam2_look,cam2_up);
    _camServo.translateCameraRel( _camera1,osg::Vec3(0,0,600),cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel( _camera2,osg::Vec3(0,0,600),cam2_eye,cam2_look,cam2_up);

    // STARTUP ////////////////////////////////////////////////////////////////
    _timer.start( 50 );
    _dataTimer.start(60);
    ROS_WARN_STREAM("<<< KineBallRun >>> Timers Started");
    // suspended = false;
    ROS_WARN_STREAM("<<< KineBallRun >>> Configured Successfully");

    resetP1Pos();
    resetP2Pos();
    p1score=0;
    p2score=0;

    // showMenuTip();
}

void KineBallRun::showHelp()
{
    help_container->show();
    help_container->raise();
    help_container->repaint();
}

void KineBallRun::hideHelp()
{
    help_container->hide();
    help_container->repaint();
    astate = ASTATE_START_COUNTDOWN_FIRST;
}

bool ccw(osg::Vec3 a,osg::Vec3 b,osg::Vec3 c)
{
    return (c[1]-a[1])*(b[0]-a[0])>(b[1]-a[1])*(c[0]-a[0]);
}

bool line_seg_intersect(osg::Vec3 s1,osg::Vec3 e1,osg::Vec3 s2,osg::Vec3 e2)
{
    return (ccw(s1,s2,e2) != ccw(e1,s2,e2) )&&( ccw(s1,e1,s2) != ccw(s1,e1,e2) );
}

void KineBallRun::updateP1Score(int s)
{
    p1scoreboard->setText(QString("Score:\n%1/10").arg(s));
}

void KineBallRun::updateP2Score(int s)
{
    p2scoreboard->setText(QString("Score:\n%1/10").arg(s));
}

// void KineBallRun::pullData()
// {
//     if(this->useKinect){
//         requestUserData();
//     }
//     keyboardUpdateEnv();
// }

void KineBallRun::resetPlayer1()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Player 1 Reset");
    
    EasyParticle* partsys = new EasyParticle(p1prev,this->assetPaths[10],osg::Vec3(255.0/255.0,191.0/255.0,89.0/255.0),P_EXPLODE);
    _partWrapper->addChild(partsys);
    particles_.push_back(partsys);
    player1_wrapper_->setHidden();
    if(p1pullback){
        _camServo.translateCameraRel(   _camera1,R1_old*osg::Vec3(0,0,dz1*5),
                                        cam1_eye,cam1_look,cam1_up); 
    }
    p1_reset_timer.start(2500);
    p1_state = PSTATE_WAIT;
}

void KineBallRun::resetPlayer2()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Player 2 Reset");
    
    EasyParticle* partsys = new EasyParticle(p2prev,this->assetPaths[10],osg::Vec3(0,157.0/255.0,255.0/255.0),P_EXPLODE);
    _partWrapper->addChild(partsys);
    particles_.push_back(partsys);
    player2_wrapper_->setHidden();
    if(p2pullback){
        _camServo.translateCameraRel(   _camera2,R2_old*osg::Vec3(0,0,-dz2*5),
                                        cam2_eye,cam2_look,cam2_up); 
    }
    p2_reset_timer.start(2500);
    p2_state = PSTATE_WAIT;
}

void KineBallRun::resetP1Pos()
{
    player1_wrapper_->setPos3DAbs(Vec3(0,0,0));
    player1_wrapper_->setPos3DRel(p1_start_pos);
    _camServo.translateCameraAbs( _camera1,cam1_start,cam1_look_start,cam1_up_start,
                                    cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel( _camera1,osg::Vec3(p1_start_pos),
                                                    cam1_eye,cam1_look,cam1_up);
    _camServo.translateCameraRel( _camera1,osg::Vec3(0,0,1000),cam1_eye,cam1_look,cam1_up);                                                   
    p1_state = PSTATE_RUN;
    theta1=0;
    player1_wrapper_->setVisible();                                                    
    p2score++;
}

void KineBallRun::resetP2Pos()
{
    player2_wrapper_->setPos3DAbs(Vec3(0,0,0));
    player2_wrapper_->setPos3DRel(p2_start_pos);
    _camServo.translateCameraAbs( _camera2,cam2_start,cam2_look_start,cam2_up_start,
                                    cam2_eye,cam2_look,cam2_up);
    _camServo.translateCameraRel( _camera2,osg::Vec3(p2_start_pos),
                                                    cam2_eye,cam2_look,cam2_up);
    _camServo.translateCameraRel( _camera2,osg::Vec3(0,0,1000),cam2_eye,cam2_look,cam2_up);                                                      
    p2_state = PSTATE_RUN;
    theta2=0;
    player2_wrapper_->setVisible();
    p1score++;
}

void KineBallRun::dismissability()
{
    dismissable=true;
}




void KineBallRun::initiateUsers()
{

    // printf("Initiate users \n");
    p1id = -1;
    p2id = -1;
    double close = 10000000;
    // Get closest

    wallframe::AppUserMap::iterator it;

    // UserPtrMap::iterator it;
    Eigen::Vector3d u1pos,u2pos;
    int closest=-1;
    Eigen::Vector3d torclosest;
    for(it = users_.begin();it!=users_.end();it++){
        wallframe::AppUser user = it->second;
        // AppUser* user = it->second;

        // ASK
        Eigen::Vector3d torso = user.jtPosByName("torso");
        // Eigen::Vector3d torso = user->joint_pos_3d[TOR];
        if(torso[2]< close){
 
            // ASK
            closest = it->first;
        
            // ASK
            torclosest = user.jtPosByName("torso");
            // torclosest = user->joint_pos_3d[TOR];
 
            close = torso[2];
        }
    }

    if(closest !=-1)
    {
        if(torclosest[0]<0)
        {
            p1id=closest;
        }
        else
        {
            p2id=closest;
        }
    }
    // if(users_.size()>0){printf("there is a user\n");}

    if(users_.size()>1){
        close = 10000000;
        

        // Get second closest
        for(it = users_.begin();it!=users_.end();it++){
            // AppUser* user = it->second;
 
          wallframe::AppUser user = it->second;
            // AppUser* user = it->second;
 
            // ASK

            // Eigen::Vector3d torso = user->joint_pos_3d[TOR];
            
            Eigen::Vector3d torso = user.jtPosByName("torso");

            if(it->first != p1id && it->first != p2id){
                if(torso[2]< close){
                    if(p1id == -1){
                        p1id = it->first;
                    }else{
                        if(torso[0]>torclosest[0])
                        {
                            p2id = it->first;
                        }
                        else
                        {
                            p2id=p1id;
                            p1id=it->first;
                        }
                    }
                    close = torso[2];
                }
            }
        }
    }


    if(p1id == -1){
        ai_mode1 = true;
    }else{
        ai_mode1 = false;
    }

    if(p2id == -1){
        ai_mode2 = true;
    }else{
        ai_mode2 = false;
    }

    if(p1id != p1id_prev){
        if(p1id <= 12 && p1id != -1){
            p1hud->setTexture(p1id+12);
        }else{
            p1hud->setTexture(26);
        }
            
    }
    if(p2id != p2id_prev){
        if(p2id <= 12 && p2id != -1){
            p2hud->setTexture(p2id);
        }else{
            p2hud->setTexture(25);
        }      
    }

    p1id_prev = p1id;
    p2id_prev = p2id;
}

void KineBallRun::updateFromUsers()
{
    initiateUsers();

    Eigen::Vector3d head;

    double zvel = 700;
    double xvel = .015;
    double maxx = 120;
    double maxz = 80;
    double zoff = 10, xoff = 20;
    double dx = 0,dy = 0,dz = 0;

    if(ai_mode1){
        updateP1AI();
    }else if(p1id != -1){
        // head = users_[p1id]->joint_pos_body_3d[HEA];
        head = users_[p1id].jtPosBodyByName("head");
        if(head[0] < 0-xoff){
            theta1 += fabs(xvel*pow(fabs(head[0]/maxx),2));
            
        }else if(head[0] > 0+xoff){
            theta1 -= fabs(xvel*pow(fabs(head[0]/maxx),2));
            
        }
        if(head[2] < -40-zoff){
            dz1 = fabs(200+zvel*1.5*pow(fabs(head[2]/maxz),2));
        }else{
            dz1 = 200;
        }
    }
    
    if(ai_mode2){
        updateP2AI();
    }else if(p2id != -1 ){
        head = users_[p2id].jtPosBodyByName("head");
        if(head[0] < 0-xoff){
            theta2 += fabs(xvel*pow(fabs(head[0]/maxx),2));
            
        }else if(head[0] > 0+xoff){
            theta2 -= fabs(xvel*pow(fabs(head[0]/maxx),2));
            
        }
        if(head[2] < -40-zoff){
            dz2 = fabs(200+zvel*1.5*pow(fabs(head[2]/maxz),2));
        }else{
            dz2 = 200;
        }
    }
       
}

// VK old way to get the user information
// void KineBallRun::initiateUsers()
// {
//     p1id = -1;
//     p2id = -1;
//     double close = 10000000;
//     // Get closest

//     UserPtrMap::iterator it;
//     Eigen::Vector3d u1pos,u2pos;
//     int closest=-1;
//     Eigen::Vector3d torclosest;
//     for(it = users_.begin();it!=users_.end();it++){
//         AppUser* user = it->second;
//         Eigen::Vector3d torso = user->joint_pos_3d[TOR];
//         if(torso[2]< close){
//             closest = it->first;
//             torclosest = user->joint_pos_3d[TOR];
//             close = torso[2];
//         }
//     }

//     if(closest !=-1)
//     {
//         if(torclosest[0]<0)
//         {
//             p1id=closest;
//         }
//         else
//         {
//             p2id=closest;
//         }
//     }

//     if(users_.size()>1){
//         close = 10000000;

//         // Get second closest
//         for(it = users_.begin();it!=users_.end();it++){
//             AppUser* user = it->second;
//             Eigen::Vector3d torso = user->joint_pos_3d[TOR];
//             if(it->first != p1id && it->first != p2id){
//                 if(torso[2]< close){
//                     if(p1id == -1){
//                         p1id = it->first;
//                     }else{
//                         if(torso[0]>torclosest[0])
//                         {
//                             p2id = it->first;
//                         }
//                         else
//                         {
//                             p2id=p1id;
//                             p1id=it->first;
//                         }
//                     }
//                     close = torso[2];
//                 }
//             }
//         }
//     }


//     if(p1id == -1){
//         ai_mode1 = true;
//     }else{
//         ai_mode1 = false;
//     }

//     if(p2id == -1){
//         ai_mode2 = true;
//     }else{
//         ai_mode2 = false;
//     }

//     if(p1id != p1id_prev){
//         if(p1id <= 12 && p1id != -1){
//             p1hud->setTexture(p1id+12);
//         }else{
//             p1hud->setTexture(26);
//         }
            
//     }
//     if(p2id != p2id_prev){
//         if(p2id <= 12 && p2id != -1){
//             p2hud->setTexture(p2id);
//         }else{
//             p2hud->setTexture(25);
//         }      
//     }

//     p1id_prev = p1id;
//     p2id_prev = p2id;
// }

// void KineBallRun::updateFromUsers()
// {
//     initiateUsers();

//     Eigen::Vector3d head;

//     double zvel = 700;
//     double xvel = .015;
//     double maxx = 120;
//     double maxz = 80;
//     double zoff = 10, xoff = 20;
//     double dx = 0,dy = 0,dz = 0;

//     if(ai_mode1){
//         updateP1AI();
//     }else if(p1id != -1){
//         head = users_[p1id]->joint_pos_body_3d[HEA];
//         if(head[0] < 0-xoff){
//             theta1 += fabs(xvel*pow(fabs(head[0]/maxx),2));
            
//         }else if(head[0] > 0+xoff){
//             theta1 -= fabs(xvel*pow(fabs(head[0]/maxx),2));
            
//         }
//         if(head[2] < -40-zoff){
//             dz1 = fabs(200+zvel*1.5*pow(fabs(head[2]/maxz),2));
//         }else{
//             dz1 = 200;
//         }
//     }
    
    
//     if(ai_mode2){
//         updateP2AI();
//     }else if(p2id != -1 ){
//         head = users_[p2id]->joint_pos_body_3d[HEA];
//         if(head[0] < 0-xoff){
//             theta2 += fabs(xvel*pow(fabs(head[0]/maxx),2));
            
//         }else if(head[0] > 0+xoff){
//             theta2 -= fabs(xvel*pow(fabs(head[0]/maxx),2));
            
//         }
//         if(head[2] < -40-zoff){
//             dz2 = fabs(200+zvel*1.5*pow(fabs(head[2]/maxz),2));
//         }else{
//             dz2 = 200;
//         }
//     }
       
// }

void KineBallRun::updateP2AI()
{
    osg::Vec3 pull = osg::Vec3(0.0,0.0,0.0);
    // pull towards other player1
    
    osg::Vec3 p2pos = player2_wrapper_->getPos3D();
    
    dz2=2*max((1050+dz1)/2.0,dz1-100);
    if(p1_state==PSTATE_RUN)
    {
        osg::Vec3 p1pos = player1_wrapper_->getPos3D();
        osg::Quat R1; R1.makeRotate(theta1,osg::Vec3(0,1,0),
                                  0,osg::Vec3(1,0,0),
                                  0,osg::Vec3(0,0,1));
        osg::Vec3 p_pull = p1pos+R1*osg::Vec3(0,0,-15*dz1)-p2pos;
        p_pull.normalize();

        pull+=p_pull;

    
    }
    else
    {
        osg::Vec3 p1pos = osg::Vec3(grid_maxx/2,0,grid_maxz/2);

        osg::Vec3 p_pull = p1pos-p2pos;
        p_pull.normalize();

        pull+=p_pull;
    }



    pull.normalize();
    osg::Vec3 target;

    // check for collisions at target here

    
    bool resolved = false;
    bool problem = false;
    int i=0;
    int iters =0;
    while(!resolved&&iters<5000)
    {
        target = p2pos+pull*dz2;
        coll_sphere->setPos3DAbs(target);
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = coll_sphere->calcBB();
        
        if(sb.intersects(thebox))
        {
            problem = true;
            osg::Vec3 push =target-colliders_[i]->getPos3D();
            push.normalize();
            pull=pull+push*0.7;

        }

        i++;
        if(i==colliders_.size()-1)
        {
            if(!problem)
            {
                resolved=true;
            }
            else
            {
                problem= false;
                i=0;
            }
        }
        iters++;

    }

    double t_x=pull[0];
    double t_y=pull[2];
    double ttheta2=atan2(t_x,t_y)+2*pi;

    double coses = (cos(theta2)+cos(ttheta2))/2;
    double sines = (sin(theta2)+sin(ttheta2))/2;
    theta2 = atan2(sines,coses);
    dz2/=2;
    coll_sphere->setPos3DAbs(osg::Vec3(-9000,0,-9000));
            

}

void KineBallRun::updateP1AI()
{
    osg::Vec3 pull = osg::Vec3(0.0,0.0,0.0);
    // pull towards other player1
    
    osg::Vec3 p1pos = player1_wrapper_->getPos3D();
    
    dz1=2*max((1050+dz2)/2.0,dz2-100);

    if(p2_state==PSTATE_RUN)
    {
        osg::Vec3 p2pos = player2_wrapper_->getPos3D();
        osg::Quat R2; R2.makeRotate(theta2,osg::Vec3(0,1,0),
                                  0,osg::Vec3(1,0,0),
                                  0,osg::Vec3(0,0,1));
        osg::Vec3 p_pull = p2pos+R2*osg::Vec3(0,0,15*dz2) -p1pos;
        p_pull.normalize();

        pull-=p_pull;

    
    }
    else
    {
        osg::Vec3 p2pos = osg::Vec3(grid_maxx/2,0,grid_maxz/2);
       
        osg::Vec3 p_pull = p2pos-p1pos;
        p_pull.normalize();

        pull-=p_pull;
    }



    pull.normalize();
    osg::Vec3 target;

    // check for collisions at target here

    
    bool resolved = false;
    bool problem = false;
    int i=0;
    int iters =0;
    while(!resolved&&iters<5000)
    {
        target = p1pos+pull*dz1;
        coll_sphere->setPos3DAbs(target);
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = coll_sphere->calcBB();
        
        if(sb.intersects(thebox))
        {
            problem = true;
            osg::Vec3 push =target-colliders_[i]->getPos3D();
            push.normalize();
            pull=pull-push*0.7;

        }

        i++;
        if(i==colliders_.size()-1)
        {
            if(!problem)
            {
                resolved=true;
            }
            else
            {
                problem= false;
                i=0;
            }
        }
        iters++;

    }

    double t_x=pull[0];
    double t_y=pull[2];
    double ttheta1=atan2(t_x,t_y)+2*pi;

    double coses = (cos(theta1)+cos(ttheta1))/2;
    double sines = (sin(theta1)+sin(ttheta1))/2;
    theta1 = atan2(sines,coses);
    dz1/=2;
    coll_sphere->setPos3DAbs(osg::Vec3(-9000,0,-9000));
            

}

void KineBallRun::keyboardUpdateEnv()
{
    // printf(" key  \n");

    switch(astate){
        // RUN NORMALLY //
        {
        case ASTATE_RUN:
            if(useKinect){
                updateFromUsers();
            }
            else
            {
                updateP1AI();
                updateP2AI();
            }

            osg::Vec3 p1p,p2p;

            switch(p1_state){
                case PSTATE_WAIT:
                    break;
                case PSTATE_MAPCHANGE:
                    mapchange->show();
                    mapchange->raise();
                    mapchange->repaint();
                    break;
                case PSTATE_RUN:
                    int twp1 = theta1/(2*pi);
                    theta1 = theta1-((double)twp1)*2*pi;

                    osg::Vec3 pos1 = player1_wrapper_->getPos3D();
                    p1prev=player1_wrapper_->getPos3D();

                    // Rotate P1
                    player1_wrapper_->rotateAbs(osg::Vec3(0,theta1,0));
                    player1_wrapper_->rotateAbs(osg::Vec3(0,theta1,0));

                    p1p = player1_wrapper_->getPosition();
                    p2p = player2_wrapper_->getPosition();
                    osg::Vec3 to_p2 = p2p-p1p;
                    osg::Vec3 zorig(0.0,0.0,1.0);

                    double vang = (to_p2*zorig);
                    double vnorm = (to_p2.length()*zorig.length());
                    double p2dir = acos(vang/vnorm);

                    if(p1p[0]<p2p[0])
                        p2dir = -p2dir;

                    p1arrow->rotateAbs(osg::Vec3(0,_pi-p2dir-theta1,0));

                    _camServo.rotateCameraRel(_camera1,cam1_eye,
                                                cam1_look,cam1_up,cam1_start, cam1_off, theta1);
                    // Translate P1                                  
                    osg::Quat R1; R1.makeRotate(theta1,osg::Vec3(0,1,0),
                                          0,osg::Vec3(1,0,0),
                                          0,osg::Vec3(0,0,1));
                    player1_wrapper_->setPos3DRel(R1*osg::Vec3(0,0,-dz1));
                    _camServo.translateCameraRel( _camera1,R1*osg::Vec3(0,0,-dz1),
                                                            cam1_eye,cam1_look,cam1_up);
                    R1_old = R1;
                    testCollisionP1();
                    updateTrail(player1_wrapper_->getPosition(),trail_wrapper_1,&lightTrail_1,
                                trail_count1,trail_pop1,dz1,1);                                                                                    
                    break;
            }
                
            switch(p2_state){
                case PSTATE_WAIT:
                    break;
                case PSTATE_MAPCHANGE:
                    mapchange->show();
                    mapchange->raise();
                    mapchange->repaint();
                    break;
                case PSTATE_RUN:

                    int twp2 = theta2/(2*pi);
                    theta2 = theta2-((double)twp2)*2*pi;

                    osg::Vec3 pos2 = player2_wrapper_->getPos3D();
                    p2prev=player2_wrapper_->getPos3D();

                    // Rotate P1
                    player2_wrapper_->rotateAbs(osg::Vec3(0,theta2,0));

                    p1p = player1_wrapper_->getPosition();
                    p2p = player2_wrapper_->getPosition();
                    osg::Vec3 to_p1 = p1p-p2p;
                    osg::Vec3 zorig(0.0,0.0,-1.0);

                    double vang = (to_p1*zorig);
                    double vnorm = (to_p1.length()*zorig.length());
                    double p1dir = acos(vang/vnorm);

                    if(p1p[0]<p2p[0])
                        p1dir = -p1dir;

                    p2arrow->rotateAbs(osg::Vec3(0,_pi-p1dir-theta2,0));

                    _camServo.rotateCameraRel(_camera2,cam2_eye,
                                                cam2_look,cam2_up,cam2_start, cam2_off, theta2);
                    // Translate P1                                  
                    osg::Quat R2; R2.makeRotate(theta2,osg::Vec3(0,1,0),
                                          0,osg::Vec3(1,0,0),
                                          0,osg::Vec3(0,0,1));
                    player2_wrapper_->setPos3DRel(R2*osg::Vec3(0,0,dz2));
                    _camServo.translateCameraRel( _camera2,R2*osg::Vec3(0,0,dz2),
                                                            cam2_eye,cam2_look,cam2_up); 
                    R2_old = R2;
                    testCollisionP2();
                    updateTrail(player2_wrapper_->getPosition(),trail_wrapper_2,&lightTrail_2,
                                trail_count2,trail_pop2,dz2,2);    
                           
                    break;
            }

            updateParticles();


            // CHANGE THE MAP HERE IF SCORE REACHED MAXSCORE
            if(p1_state != PSTATE_MAPCHANGE && p2_state != PSTATE_MAPCHANGE){
                if(p1score==maxscore||p2score==maxscore)
                {
                    int winner = -1;
                    if(p1score > p2score){
                        winner = 1;
                    }else{
                        winner = 2;
                    }

                    mapnum++;
                    if(mapnum>totalmaps)
                    {
                        mapnum=1;
                    }


                    p1_state = PSTATE_MAPCHANGE;
                    p2_state = PSTATE_MAPCHANGE;
                    mapchange->show();
                    mapchange->raise();
                    mapchange->repaint();
                    p1winlose->setVisible();
                    p2winlose->setVisible();
                    p1arrow->setHidden();
                    p2arrow->setHidden();
                    if(winner == 1){
                        p1winlose->setTexture(17);
                        p2winlose->setTexture(18);
                    }else{
                        p1winlose->setTexture(18);
                        p2winlose->setTexture(17);
                    }
                    _mapChangeTimer.start(1500);
                }
            }

            updateP1Score(p1score);
            updateP2Score(p2score);
            break;
        }
        // COUNTDOWN START //    
        case ASTATE_START_COUNTDOWN:
            countdown_num = 3;
            countdown->setNumber(countdown_num);
            countdown->show();
            countdown->raise();
            _countdownTimer.start(1000);
            astate = ASTATE_COUNTDOWN;
            break;

        case ASTATE_START_COUNTDOWN_FIRST:
            countdown_num = 3;
            countdown->setNumber(countdown_num);
            countdown->show();
            countdown->raise();
            _countdownTimer.start(1000);
            astate = ASTATE_COUNTDOWN;
            break;

        case ASTATE_START_INSTRUCTIONS:
            _instructionTimer.start(15000);
            _mandatoryTimer.start(5000);
            astate = ASTATE_WAIT_INSTRUCTIONS;
            break;

        case ASTATE_WAIT_INSTRUCTIONS:

            initiateUsers();
            if(dismissable)
            {
                if(p1id != -1){
                  wallframe::AppUser user1 = users_[p1id];
                    Eigen::Vector3d right = user1.jtPosByName("right_hand");
                    Eigen::Vector3d left = user1.jtPosByName("left_hand");
                    Eigen::Vector3d v = left-right;

                    double dist = v.norm();
                    if(dist < 160){
                        hideHelp();
                        ROS_INFO_STREAM("<<< KineBallRun >>> User hiding help...");   
                    }
                }

                if(p2id != -1){
                    wallframe::AppUser user2 = users_[p2id];
                    Eigen::Vector3d right = user2.jtPosByName("right_hand");
                    Eigen::Vector3d left = user2.jtPosByName("left_hand");
                    Eigen::Vector3d v = left-right;
                    double dist = v.norm();
                    if(dist < 160){
                        hideHelp();
                        ROS_INFO_STREAM("<<< KineBallRun >>> User hiding help...");   
                    }
                }
            }

            break;

        // COUNTDOWN //    
        case ASTATE_COUNTDOWN:
            break;
    }
}

void KineBallRun::updateCountdown()
{
    countdown_num--;
    if(countdown_num == -1){
        _countdownTimer.stop();
        countdown->hide();
        countdown->repaint();
        astate = ASTATE_RUN;
    }else if(countdown_num == 0){
        countdown->setText("GO!");
        countdown->repaint();
        p1arrow->setVisible();
        p2arrow->setVisible();
    }else{
        countdown->setNumber(countdown_num);
        countdown->repaint();
    }
}

void KineBallRun::mapChangeResume()
{
    root->removeChild( wall_wrapper_ );
    root->removeChild( map_wrapper_ );

    loadMap(gridsize,ballsize,mapnum);

    // indicate new map here
    resetP1Pos();

    lightTrail_1.clear();
    trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
    trail_count1 = 0;
    trail_pop1 = 0;


    resetP2Pos();

    lightTrail_2.clear();
    trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
    trail_count2 = 0;
    trail_pop2 = 0;
    
    p1score=0;
    p2score=0;

    mapchange->hide();
    mapchange->repaint();

    p1winlose->setHidden();
    p2winlose->setHidden();

    p1_state = PSTATE_RUN;
    p2_state = PSTATE_RUN;
    astate = ASTATE_START_COUNTDOWN;
}

void KineBallRun::updateParticles()
{
    int checker;
    for(int i=0;i<particles_.size();i++)
    {
        checker = particles_[i]->tick();
        if(checker==0)
        {

            _partWrapper->removeChild(particles_[i]);
            particles_.removeAt(i);
        }
    }
}

void KineBallRun::testCollisionP1()
{
    p1pullback=false;
    // Test self trail collision
    for(int i = 0;i<lightTrail_1.size()-20;i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_1[i]->getBound();
        if(sb.intersects(lb)){

            double distt = lightTrail_1[i]->getPlane().distance(player1_wrapper_->getPos3D());
            if(abs(distt)< ballsize+dz1){

                ROS_INFO_STREAM("<<< KineBallRun >>> P1 Collided");
                
                theta1 = 0;
                lightTrail_1.clear();
                trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
                trail_count1 = 0;
                trail_pop1 = 0;
                
                resetPlayer1();     

                break;
            }
        }
    }
    
    // Test p2 trail collision
    for(int i = 0;i<lightTrail_2.size();i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_2[i]->getBound();
        if(sb.intersects(lb)){
            double distt = lightTrail_2[i]->getPlane().distance(player1_wrapper_->getPos3D());
            if(abs(distt)< ballsize+dz1){
                ROS_INFO_STREAM("<<< KineBallRun >>> P1 Collided");
                
                theta1 = 0;
                lightTrail_1.clear();
                trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
                trail_count1 = 0;
                trail_pop1 = 0;
                
                // remove trail of other playes until the point we collide
                for(int j=0;j<i;j++)
                {
                    trail_wrapper_2->removeChild(0,1);
                    lightTrail_2.pop_front();   
                    trail_pop2--;
                }
                resetPlayer1();
                
                break;
            }
        }
    } 
    for(int i=0;i<colliders_.size();i++)
    {
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = player1_wrapper_->calcBB();
        // const osg::BoundingSphereImpl<osg::Vec3f> sb = player1_wrapper_->getBound();
        // osg::Vec3 d = thebox.center()-sb.center();
        if(sb.intersects(thebox))
        {

            theta1 = 0;
            lightTrail_1.clear();
            trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
            trail_count1 = 0;
            trail_pop1 = 0;
            
            p1pullback=true;
            resetPlayer1();
            
            break;
        }
    }

    osg::Vec3 pos = player1_wrapper_->getPosition();
    if(pos[0] > grid_maxx || pos[0] < 0 || pos[2] > grid_maxz || pos[2] < 0){
        theta1 = 0;
        lightTrail_1.clear();
        trail_wrapper_1->removeChildren(0,trail_wrapper_1->getNumChildren());
        trail_count1 = 0;
        trail_pop1 = 0;
        p1pullback=true;
        resetPlayer1();
    }
}

void KineBallRun::testCollisionP2()
{
    p2pullback=false;
    // Test self trail collision
    for(int i = 0;i<lightTrail_2.size()-20;i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_2[i]->getBound();
        if(sb.intersects(lb)){
            double distt = lightTrail_2[i]->getPlane().distance(player2_wrapper_->getPos3D());
            if(abs(distt)< ballsize+dz2){
                ROS_INFO_STREAM("<<< KineBallRun >>> P2 Collided");
                theta2 = 0;
                lightTrail_2.clear();
                trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
                trail_count2 = 0;
                trail_pop2 = 0;

                resetPlayer2();
                break;
            }
        }
    }
    
    // Test p1 trail collision
    for(int i = 0;i<lightTrail_1.size();i++){
        const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        const osg::BoundingSphereImpl<osg::Vec3f> lb = lightTrail_1[i]->getBound();
        if(sb.intersects(lb)){
            double distt = lightTrail_1[i]->getPlane().distance(player2_wrapper_->getPos3D());
            if(abs(distt)< ballsize+dz2){
                theta2 = 0;
                lightTrail_2.clear();
                trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
                trail_count2 = 0;
                trail_pop2 = 0;

                // remove trail of other playes until the point we collide
                for(int j=0;j<i;j++)
                {
                    trail_wrapper_1->removeChild(0,1);
                    lightTrail_1.pop_front();   
                    trail_pop1--;
                }
                resetPlayer2();
                break;
            }
        }
    }
     
    for(int i=0;i<colliders_.size();i++)
    {
        osg::BoundingBox thebox = colliders_[i]->calcBB();
        osg::BoundingBox sb = player2_wrapper_->calcBB();
        // const osg::BoundingSphereImpl<osg::Vec3f> sb = player2_wrapper_->getBound();
        if(sb.intersects(thebox))
        {
            theta2 = 0;
            lightTrail_2.clear();
            trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
            trail_count2 = 0;
            trail_pop2 = 0;

            p2pullback=true;
            resetPlayer2();
            break;
        }
    }

    osg::Vec3 pos = player2_wrapper_->getPosition();
    if(pos[0] > grid_maxx || pos[0] < 0 || pos[2] > grid_maxz || pos[2] < 0){
        theta2 = 0;
        lightTrail_2.clear();
        trail_wrapper_2->removeChildren(0,trail_wrapper_2->getNumChildren());
        trail_count2 = 0;
        trail_pop2 = 0;
        p2pullback=true;
        resetPlayer2();
    }
}

void KineBallRun::updateTrail(osg::Vec3 pos,OSGObjectBase* wrapper, TexPlaneList* trail, 
                                    int& count, int& pop, double dz,int player)
{
    // Update Trail
    count+=dz;

    
    if(count>200){
        int num2add = count/200;
        osg::Vec3 sp = pos;
        
        if(pop < 5){
            // do nothing
        }else if(pop == 5){
            TexturedPlane* t = new TexturedPlane(sp+Vec3(0,400,0),sp+Vec3(0,400,200),
                                                 sp+Vec3(0,-400,0),sp+Vec3(0,-400,200),
                                                 &_assetTextures,player);
            

            trail->push_back(t);
            wrapper->addChild(t);
        }else{
            TexturedPlane* t = new TexturedPlane(sp+Vec3(0,400,0),trail->last()->ul(),
                                                 sp+Vec3(0,-400,0),trail->last()->ll(),
                                                 &_assetTextures,player);
            trail->push_back(t);
            wrapper->addChild(t);
        }
        // Reset counter
        count = 0;
        // Pop from trail when too long
        pop++;
        if(pop > 300){
            if(!trail->isEmpty());{
                wrapper->removeChild(trail->at(0));
                trail->pop_front();
            }
            
        }
    }
}

bool KineBallRun::start(){
// Start up app
_timer.start( 10 );
_dataTimer.start(10);
ROS_WARN_STREAM("<<< KineBallRun >>> Timers Started");
return true;
}
bool KineBallRun::stop(){
/*Stop stuff here before destructor is called, if needed*/
return true;
}

bool KineBallRun::pause(){
// this->hide();
// _timer.stop();
// _dataTimer.stop();
// paused = true;
// ROS_WARN_STREAM("<< KineBallRun >> Pausing");
return true;
}

bool KineBallRun::resume(){
// _timer.start(10);
// _dataTimer.start(10);
// this->show();
// this->glWidget->show();
// this->update();
// paused = false;
// ROS_WARN_STREAM("<< KineBallRun >> Resumed");
return true;
}


osgQt::GLWidget* KineBallRun::addQWidget( osg::Camera* camera, osg::Node* scene )
{
    osgViewer::View* view=new osgViewer::View;
    view->setCamera( camera );
    addView(view);
    
    view->setSceneData( scene );
    view->addEventHandler( new osgViewer::StatsHandler );
    //setCameraManipulator( new osgGA::TrackballManipulator );
    
    KineBallRunKeyboardHandler* myFirstEventHandler =
        new KineBallRunKeyboardHandler();
    myFirstEventHandler->setup(this);
    view->addEventHandler(myFirstEventHandler); 
    //view->setThreadingModel(SingleThreaded);
    osgQt::GraphicsWindowQt* gw = 
        dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
    
    return gw ? gw->getGLWidget() : NULL;
}

osg::Camera* KineBallRun::createMapCamera1( int x, int y, int w, int h, const std::string& name, bool windowDecoration)
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
    
    mapcam = new osg::Camera;
    // _camera1->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    // _camera1->setSmallFeatureCullingPixelSize(30.f);
    mapcam->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    mapcam->setViewMatrixAsLookAt( osg::Vec3d( grid_maxx/2,400000,grid_maxz/2 ), // eye
                                    osg::Vec3d( grid_maxx/2,0,grid_maxz/2 ),  // look
                                    osg::Vec3d( 0,0,-1 )); // up

    mapcam->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    mapcam->setViewport( new osg::Viewport(0, 0, w, h) );
    mapcam->setProjectionMatrixAsOrtho(-grid_maxx/2,grid_maxx/2,grid_maxz/2,-grid_maxz/2,1,1000000);
    
    // _cameraOrbit = osg::Vec3(0,0,1);
    // _cameraStart = osg::Vec3(0,1000,5000);

    return mapcam;
}

osg::Camera* KineBallRun::createQCamera1( int x, int y, int w, int h, const std::string& name, bool windowDecoration)
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
    // _camera1->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    // _camera1->setSmallFeatureCullingPixelSize(30.f);
    _camera1->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera1->setViewMatrixAsLookAt( osg::Vec3d( 0,1000,5000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera1->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera1->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera1->setProjectionMatrixAsPerspective(50.0f, static_cast<double>(w)/static_cast<double>(h), 1.0f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera1;
}

osg::Camera* KineBallRun::createQCamera2( int x, int y, int w, int h,  const std::string& name, bool windowDecoration)
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
    // _camera2->setCullingMode(osg::CullSettings::ENABLE_ALL_CULLING);
    // _camera2->setSmallFeatureCullingPixelSize(30.f);
    _camera2->setGraphicsContext( new osgQt::GraphicsWindowQt(traits.get()) );

    _camera2->setViewMatrixAsLookAt( osg::Vec3d( 0,1000,5000 ), // eye
                                    osg::Vec3d( 0,200,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera2->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera2->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera2->setProjectionMatrixAsPerspective(50.0f, static_cast<double>(w)/static_cast<double>(h), 1.0f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,1000,5000);

    return _camera2;
}

// Keyboard Handler ////////////////////////////
bool KineBallRunKeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            switch(ea.getKey())
            {
                case 'q':
                    // appPtr->sendSelfTerminate(); 
                    break;
                // Camera Zoom
                case 'p':
                    cam_mult = 100;
                    break;
                case 'o':
                    cam_mult = 1;
                    break;
                // Hud Offset
                case 'u':
                    hud_off += 10;
                    cerr<<hud_off<<endl;
                    break;
                case 'i':
                    hud_off -= 10;
                    cerr<<hud_off<<endl;
                    break;

                case 't':
                    // appPtr->p1hud->setPos3DRel(osg::Vec3(0,0,-10));
                    // printv(appPtr->p1hud->getPosition());
                    break;
                case 'v':
                    // appPtr->p1hud->setPos3DRel(osg::Vec3(0,0,10));
                    // printv(appPtr->p1hud->getPosition());
                    break;
                case 'f':
                    // appPtr->p1hud->setPos3DRel(osg::Vec3(0,10,0));
                    // printv(appPtr->p1hud->getPosition());
                    break;
                case 'g':
                    // appPtr->p1hud->setPos3DRel(osg::Vec3(0,-10,0));
                    // printv(appPtr->p1hud->getPosition());
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
                case '8':{
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,0,-1000*cam_mult),
                                                                    cam1_eye,cam1_look,cam1_up);
                    // Get Camera pos info for hud
                    // osg::Vec3 e1,u1,c1;
                    // appPtr->_camera1->getViewMatrixAsLookAt(e1,c1,u1);
                    // osg::Vec3 look_dir = c1-e1;
                    // look_dir.normalize();
                    // osg::Vec3 cam_hud_pos = e1 - look_dir*hud_off;
                    // appPtr->p1hud->setPos3DAbs(cam_hud_pos);
                    // printv(cam_hud_pos - appPtr->p1_start_pos);
                    break;}
                case '2':{
                    appPtr->_camServo.translateCameraRel( appPtr->_camera1,osg::Vec3(0,0,1000*cam_mult),
                                                                    cam1_eye,cam1_look,cam1_up);
                    // Get Camera pos info for hud
                    // osg::Vec3 e1,u1,c1;
                    // appPtr->_camera1->getViewMatrixAsLookAt(e1,c1,u1);
                    // osg::Vec3 look_dir = c1-e1;
                    // look_dir.normalize();
                    // osg::Vec3 cam_hud_pos = e1 - look_dir*hud_off;
                    // appPtr->p1hud->setPos3DAbs(cam_hud_pos);
                    // printv(cam_hud_pos - appPtr->p1_start_pos);
                    break;}
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

void KineBallRunKeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v){v.visit(*this);}
void KineBallRunKeyboardHandler::setup(KineBallRun* appPt){this->appPtr = appPt;}

void KineBallRun::setImageDirectories()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Getting Asset Image Directories");
    // Asset Textures //
    QDir asset_dir(this->assetDir);
    QStringList assetFiles = asset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<assetFiles.count(); i++){
        this->assetPaths << asset_dir.absoluteFilePath(assetFiles[i]);
    }

    ROS_INFO_STREAM("<<< KineBallRun >>> Getting Tag Image Directories");
    QDir tagAsset_dir(this->tagAssetDir);
    QStringList tagAssetFiles = tagAsset_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<tagAssetFiles.count(); i++){
        this->tagAssetPaths << tagAsset_dir.absoluteFilePath(tagAssetFiles[i]);
    }
}

void KineBallRun::LoadTextures()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Loading Asset Images... ");
    osg::Image* img;    
    for (int i = 0; i < this->assetPaths.size(); ++i){
        img = osgDB::readImageFile(this->assetPaths.at(i).toStdString());
        ROS_INFO_STREAM("<<< KineBallRun >>> Loading Image: "<<this->assetPaths.at(i).toStdString());
        osg::TextureRectangle* t = new osg::TextureRectangle(img); 
        this->_assetTextures.push_back(t);
    }
    ROS_INFO_STREAM("<<< KineBallRun >>> "<<_tagAssetTextures.size()<<" images...");

    ROS_INFO_STREAM("<<< KineBallRun >>> Loading Tag Images... ");
    for (int i = 0; i < this->tagAssetPaths.size(); ++i){
        img = osgDB::readImageFile(this->tagAssetPaths.at(i).toStdString());
        ROS_INFO_STREAM("<<< KineBallRun >>> Loading Image: "<<this->tagAssetPaths.at(i).toStdString());
        osg::TextureRectangle* t = new osg::TextureRectangle(img); 
        this->_tagAssetTextures.push_back(t);
    }
    ROS_INFO_STREAM("<<< KineBallRun >>> loaded "<<_tagAssetTextures.size()<<" tag images...");
}

void KineBallRun::setTexImageDirs()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Getting Texture Directories");
    QDir tex_img_dir(this->texImageDir);
    QStringList tex_img_rel_paths = tex_img_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<tex_img_rel_paths.count(); i++){
        this->textureImagePaths << tex_img_dir.absoluteFilePath(tex_img_rel_paths[i]);
    }
}

void KineBallRun::LoadTextureImages()
{
    ROS_INFO_STREAM("<<< KineBallRun >>> Loading Textures");
    osg::Image* img;    
    for (int i = 0; i < this->textureImagePaths.size(); ++i){
        img = osgDB::readImageFile(this->textureImagePaths.at(i).toStdString());
        ROS_INFO_STREAM("<<< KineBallRun >>> Loading Texture: "<<this->textureImagePaths.at(i).toStdString()); 
        textureImages_.push_back(img);  
        this->_assetTextures.push_back(new osg::TextureRectangle(img));
    }
}


} // modulair namespace ends

using namespace modulair;

int main(int argc, char* argv[]){
  ros::init(argc,argv, "tron_app");
  ROS_WARN_STREAM("TronApp: Starting Up...");
  ros::NodeHandle node_handle;
  QApplication application(argc,argv);
  // This line will quit the application once any window is closed.
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
  modulair::KineBallRun tron_app("TronApp",node_handle,20);
  tron_app.build();
  tron_app.start();
  ROS_WARN_STREAM("ImageStormApp: App Running");
  application.exec();
  // Running
  tron_app.stop();
  ROS_WARN_STREAM("TronaPp: App Finished");
  return 0;
}
