#ifndef tron_app_h
#define tron_app_h 
// MODULAIR INCLUDES
//#include <modulair_core/modulair_core.h>
#include <wallframe_core/wallframe_app_base_qt.h>
#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>
// QT //
#include <QtCore/QTimer>
#include <QtGui/QApplication>
#include <QtGui/QGridLayout>
#include <QtGui>
// OSG //
#include <osgQt/GraphicsWindowQt>
#include <osgViewer/CompositeViewer>
#include <osgViewer/ViewerEventHandlers>
#include <osgViewer/Viewer>
#include <osgGA/TrackballManipulator>
#include <osgDB/ReadFile>
#include <osgDB/Registry>
#include <osg/ShapeDrawable>
#include <osg/PositionAttitudeTransform>
#include <osg/Notify>
#include <osg/TextureRectangle>
#include <osg/Geometry>
#include <osg/Material>
#include <osg/StateSet>
#include <osg/Light>
#include <osg/Depth>
#include <osg/LightSource>
#include <osg/Geode>
#include <osg/TexMat>
#include <osg/Group>
#include <osg/Projection>
#include <osg/MatrixTransform>
#include <osgText/Text> 
#include <osg/Fog>
#include <osg/CullFace>
// IOSTRAM //
#include <iostream>
// INTERNAL INCLUDES //
// #include "AppBase.h"
// #include "LairWidget.h"

// #include "OSGObjectWrapper.h"
// #include "StateObject.h"
// #include "OSGObject.h"
// #include "SkeletonObject.h"
// #include "SphereObject.h"
// #include "CubeObject.h"
// #include "InteractionObject.h"
// #include "PlanarObject.h"
// #include "CursorObject.h"
// #include "DockableObject.h"
// #include "Holster.h"
// #include "MoveTool.h"
// #include "CameraTool.h"
// #include "ActableObject.h"
// #include "TexturedSphere.h"
#include <modulair_app_tron/TexturedPlane.h>
#include <modulair_app_tron/TexturedCube.h>
#include <modulair_app_tron/SphereObject.h>
#include <modulair_app_tron/TiledPlane.h>
#include <modulair_app_tron/EasyParticle.h>
// #include "TiledPlane.h"
// #include "DoublePlane.h"
// #include "EasyParticle.h"

#define PSTATE_WAIT 0
#define PSTATE_RUN 1
#define PSTATE_MAPCHANGE 2
#define ASTATE_COUNTDOWN 3
#define ASTATE_START_COUNTDOWN 4
#define ASTATE_RUN 5
#define ASTATE_START_COUNTDOWN_FIRST 6
#define ASTATE_START_INSTRUCTIONS 7
#define ASTATE_WAIT_INSTRUCTIONS 8

#define W_WIDTH 1000 // VK to find this

//  dont know why kel has kept this in cpp

#include <modulair_app_tron/KineBallRun.h>
#include <modulair_osg_tools/vector_conversions.h>  
    

// namespace lair{
namespace modulair{

    typedef std::vector<char> charVec;
    typedef std::vector<charVec> charVec2D;

    class KineBallRun;
    class KineBallRunKeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(KineBallRun* appPt);
        KineBallRun* appPtr;
    };

    // class CountDownWidget : public LairWidget
    class CountDownWidget : public QWidget
    {
        Q_OBJECT
    public:
        CountDownWidget(QWidget* parent): QWidget(parent){
            QFont textFont;
            textFont.setPixelSize(W_WIDTH/24);
            textFont.setBold(true);
            QFont textFontS;
            textFontS.setPixelSize(W_WIDTH/75);
            textFontS.setBold(true);
            this->setContentsMargins(0,0,0,0);

            f1 = new QLabel("3");
            f1->setFont(textFont);
            f1->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
            f2 = new QLabel("3");
            f2->setFont(textFontS);
            f2->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
            f3 = new QLabel("3");
            f3->setFont(textFont);
            f3->setAlignment(Qt::AlignCenter | Qt::AlignCenter);
            f4 = new QLabel("3");
            f4->setFont(textFontS);
            f4->setAlignment(Qt::AlignCenter | Qt::AlignCenter);

            layout = new QHBoxLayout;
            layout->addWidget(f1);
            layout->addWidget(f2);
            layout->addWidget(f3);
            layout->addWidget(f4);

            this->setLayout(layout);
        };
        ~CountDownWidget(){};
        void setNumber(int num){
            f1->setText(QString("%1").arg(num));
            f2->setText(QString("%1").arg(num));
            f3->setText(QString("%1").arg(num));
            f4->setText(QString("%1").arg(num));
        };
        void setText(QString text){
            f1->setText(text);
            f2->setText(text);
            f3->setText(text);
            f4->setText(text);
        };
    private:

        QLabel* f1;
        QLabel* f2;
        QLabel* f3;
        QLabel* f4;
        QHBoxLayout* layout;
    };

    // class KineBallRun : public AppBase, public osgViewer::CompositeViewer
    class KineBallRun : public wallframe::WallframeAppBaseQt, public osgViewer::CompositeViewer
    {
        Q_OBJECT;
    public:
        KineBallRun(std::string app_name, ros::NodeHandle nh, int event_deque_size);
        // KineBallRun( QWidget *par = NULL, QWidget *appManager = NULL, QString appID = "null", bool useKin = false);
        ~KineBallRun(){};
            /* ModulairAppBase Virtual Methods */
        bool build();
        bool start();
    bool stop();
    bool pause();
    bool resume();
        
        // Qt Functions
        // osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
        //                                 osg::Node* scene );

        osgQt::GLWidget* addQWidget( osg::Camera* camera, 
                                        osg::Node* scene ); 
                                                                               

        virtual void paintEvent( QPaintEvent* event ) { frame(); }


        osg::Camera* createQCamera1( int x, int y, int w, int h,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        osg::Camera* createQCamera2( int x, int y, int w, int h,
                                   const std::string& name="",
                                   bool windowDecoration=false);

        osg::Camera* createMapCamera1( int x, int y, int w, int h,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        
        // Methods
        void updateCursors();
        void initiateUsers();
        void setImageDirectories();
        void setTexImageDirs();
        void LoadTextures();
        void LoadTextureImages();
        // void keyboardUpdateEnv();
        void resetPlayer1();
        void resetPlayer2();
        void updateFromUsers();
        void updateP1AI();
        void updateP2AI();
        void updateP1Score(int s);
        void updateP2Score(int s);


        // line seg intersect check for collision detection
        bool line_seg_intersect(osg::Vec3 s1,osg::Vec3 e1,osg::Vec3 s2,osg::Vec3 e2);
        // used by above method, checks if points are in counterclockwise order
        bool ccw(osg::Vec3 a,osg::Vec3 b,osg::Vec3 c);

        // void collisionDetection(vct3 pos,double& theta,double& dz, double &sin_t, double &cos_t);
        void updateTrail(osg::Vec3 pos,OSGObjectBase* wrapper, TexPlaneList* trail, 
                                    int& count, int& pop, double dz,int player);
        void testCollisionP1();
        void testCollisionP2();

        void updateParticles();
        osg::Vec3 loadMap(double sz, double ballH, int mapnum);
        
        // Camera Control Variables //
        osg::ref_ptr<osg::Camera> _camera1;
        osg::ref_ptr<osg::Camera> _camera2;
        osg::ref_ptr<osg::Camera> mapcam;
        ServoCamera _camServo;
        osg::Vec3 _cameraOrbit;
        osg::Vec3 _cameraStart;

        // USERS //
        int numActiveUsers;
        osg::Vec3 p1_start_pos;
        osg::Vec3 p2_start_pos;

        int p1score;
        int p2score;
        int mapnum;
        int totalmaps;
        int maxscore;
        int astate;
        int countdown_num;
        

    private:

        // inner stuff
        int numPlanes;
        osg::Vec3* default_pos;
        int p1id;
        int p2id;
        int p1id_prev;
        int p2id_prev;


    public:

        int gridsize;
        int grid_maxx;
        int grid_maxz;
        int ballsize;

        // Ball
        osg::Vec3 ballPosLast;

        osg::Vec3 p1prev;
        osg::Vec3 p2prev;

        // Icons //
        ObjectMap _actableIcons;
        ObjectMap _dockableIcons;
        SphereMap _cursorIcons;
        PlaneMap _planarCursors;

        ObjectList cubes_; //
        ParticleList particles_; //
        ObjectList spheres_; //



        ObjectList planes_; //
        ObjectList colliders_; //

        TexPlaneList lightTrail_1; //
        TexPlaneList lightTrail_2; //
        // TiledPlaneList lightTrail_;

        int particle_count1;
        int particle_count2;

        int trail_count1;
        int trail_count2;
        int trail_pop1;
        int trail_pop2;

        bool ai_mode1;
        bool ai_mode2;

        bool dismissable;

        QLabel* center; //
        QLabel* score; //
        QLabel* logo; //
        QLabel* p1scoreboard; //
        QLabel* p2scoreboard; //
        CountDownWidget* countdown; //
        QLabel* mapchange; //
        QPixmap mapChangeImg; //

        QGridLayout* help_layout; //
        QWidget* help_container; //
        QLabel* h1; //
        QLabel* h2; //
        QLabel* h3; //
        QLabel* h4; //
        QLabel* hl1; //
        QLabel* hl2; //
        QLabel* hl3; //
        QLabel* hl4; //

        // Texture Images //

        // Wrappers //
        OSGObjectBase* _envWrapper; //
        OSGObjectBase* _partWrapper; //
        OSGObjectBase* cursor_wrapper_; //
        OSGObjectBase* sphere_wrapper_; //
        OSGObjectBase* cube_wrapper_; //
        OSGObjectBase* map_wrapper_; //
        OSGObjectBase* wall_wrapper_; //
        OSGObjectBase* trail_wrapper_1; //
        OSGObjectBase* trail_wrapper_2; //
        OSGObjectBase* p1_hud_wrapper; //
        OSGObjectBase* p2_hud_wrapper; //
        OSGObjectBase* skybox_wrapper_; //
        OSGObjectBase* _cameraAttachedObjects;

        OSGObjectBase* player1_wrapper_; //
        OSGObjectBase* player2_wrapper_; //
        TexturedPlane* p1hud; //
        TexturedPlane* p2hud; //
        TexturedPlane* p1arrow; //
        TexturedPlane* p2arrow; //
        TexturedPlane* p1winlose; //
        TexturedPlane* p2winlose; //

        TexturedCube* skybox_; //

        int runs;


        SphereObject* coll_sphere; //
                                   
        // OPENGL and QT //
        osgQt::GLWidget* glWidget1; //
        osgQt::GLWidget* glWidget2; //

        // osgQt::GLWidget* mapWidget1;

        osg::ref_ptr<osg::Group> root; //
        osg::ref_ptr<osg::Group> actual_root; //
        
        // Variables
        QStringList assetPaths;
        QStringList tagAssetPaths;
        QStringList imagePaths;
        QStringList textureImagePaths;
        TextureList _tagAssetTextures;
        TextureList _assetTextures;
        TextureList _imageTextures;
        TextureList _indexTextures;
        TexImageList textureImages_;

        // Collections
        QStringList _collectionNames;

        // Large Image 
        TextureList _large;
        PlanarObject* _largeImage;
        PlanarObject* _pleaseWait;
        PlanarObject* _highlight;

        osg::Vec3 _imageLocation;

        QString assetDir;
        QString tagAssetDir;
        QString imageDir;
        QString texImageDir;
        QString mapDir;
        int dataTextureIndexStart;
        int dataTextureIndex;

        bool useKinect;

        static const double GUI_MAX_X =  1.0;
        static const double GUI_MIN_X = -1.0;
        static const double GUI_MAX_Y =  0.5;
        static const double GUI_MIN_Y = -0.5;
        
        int runtime;
        bool paused;
        int _envState;
        charVec2D minimap;

        int p1_state;
        int p2_state;

    Q_SIGNALS:
        void showImgs(int node);
        void showThumbs();
        void removeAllActive();

    public Q_SLOTS:
        void config();
        // void pause();
        // void unpause();
        // void resume();
        // void suspend();
        // void pullData(); 
        void increment();
        void recieveDiscreteGesture(QMap<int,int> events){};
        void resetP1Pos();
        void resetP2Pos();
        void mapChangeResume();
        void updateCountdown();
        void showHelp();
        void hideHelp();
        void dismissability();
        void keyboardUpdateEnv();
        
    protected:
        // Timers //
        QTimer _timer;
        QTimer _dataTimer;
        QTimer _mapChangeTimer;
        QTimer _countdownTimer;
        QTimer _instructionTimer;
        QTimer _mandatoryTimer;

        QTimer p1_reset_timer;
        QTimer p2_reset_timer;
    };    
}
 
#endif
