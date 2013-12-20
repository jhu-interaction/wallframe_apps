#ifndef _KineBallRunSandbox_h
#define _KineBallRunSandbox_h
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

#include <wallframe_core/wallframe_app_base_qt.h>
#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>
// INTERNAL INCLUDES //
// #include "AppBase.h"

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
#include "TexturedPlane.h"
// #include "TexturedCube.h"
// #include "TiledPlane.h"
// #include "DoublePlane.h"
// #include "EasyParticle.h"    

namespace modulair{

    typedef std::vector<char> charVec;
    typedef std::vector<charVec> charVec2D;

    class KineBallRunSandbox;
    class KineBallRunSandboxKeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(KineBallRunSandbox* appPt);
        KineBallRunSandbox* appPtr;
    };

    class KineBallRunSandbox : public ModulairAppBaseQt, public osgViewer::CompositeViewer
    {
        Q_OBJECT;
    public:
        KineBallRunSandbox( QWidget *par = NULL, QWidget *appManager = NULL, QString appID = "null", bool useKin = false);
        ~KineBallRunSandbox(){}
        
        // Qt Functions
        // osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
        //                                 osg::Node* scene );

        osgQt::GLWidget* addQWidget( osg::Camera* camera, 
                                        osg::Node* scene ); 
                                                                               
        osg::Camera* createCamera1( int x, int y, int w, int h,
                                   osgQt::GLWidget *QTObject,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        osg::Camera* createCamera2( int x, int y, int w, int h,
                                   osgQt::GLWidget *QTObject,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        virtual void paintEvent( QPaintEvent* event ) { frame(); }


        osg::Camera* createQCamera1( int x, int y, int w, int h,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        osg::Camera* createQCamera2( int x, int y, int w, int h,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        
        // Methods
        void updateCursors();
        void updateEnvironment(){};
        void readConfigFile();
        void setImageDirectories();
        void setTexImageDirs();
        void LoadTextures();
        void LoadTextureImages();
        void keyboardUpdateEnv();
        void resetPlayer1();
        void resetPlayer2();

        void collisionDetection(vct3 pos,double& theta,double& dz, double &sin_t, double &cos_t);
        void updateTrail(osg::Vec3 pos,OSGObjectBase* wrapper, TexPlaneList* trail, 
                                    int& count, int& pop, double dz);
        void testCollisionP1();
        void testCollisionP2();
        osg::Vec3 loadMap(double sz, double ballH);
        
        // Camera Control Variables //
        osg::ref_ptr<osg::Camera> _camera1;
        osg::ref_ptr<osg::Camera> _camera2;
        ServoCamera _camServo;
        osg::Vec3 _cameraOrbit;
        osg::Vec3 _cameraStart;

        // USERS //
        int numActiveUsers;

    private:

        // inner stuff
        int numPlanes;
        osg::Vec3* default_pos;
        osg::Vec3 p1_start_pos;
        osg::Vec3 p2_start_pos;


    public:

        int gridsize;
        int grid_maxx;
        int grid_maxz;
        int ballsize;

        // Ball
        osg::Vec3 ballPosLast;

        // Icons //
        ObjectMap _actableIcons;
        ObjectMap _dockableIcons;
        SphereMap _cursorIcons;
        PlaneMap _planarCursors;

        ObjectList cubes_;
        ObjectList spheres_;
        ObjectList planes_;
        ObjectList colliders_;
        TexPlaneList lightTrail_1;
        TexPlaneList lightTrail_2;
        // TiledPlaneList lightTrail_;

        int trail_count1;
        int trail_count2;
        int trail_pop1;
        int trail_pop2;

        // Texture Images //

        // Wrappers //
        OSGObjectBase* _envWrapper;
        OSGObjectBase* cursor_wrapper_;
        OSGObjectBase* sphere_wrapper_;
        OSGObjectBase* cube_wrapper_;
        OSGObjectBase* map_wrapper_;
        OSGObjectBase* wall_wrapper_;
        OSGObjectBase* trail_wrapper_1;
        OSGObjectBase* trail_wrapper_2;
        OSGObjectBase* skybox_wrapper_;
        OSGObjectBase* _cameraAttachedObjects;

        OSGObjectBase* player1_wrapper_;
        OSGObjectBase* player2_wrapper_;

        TexturedCube* skybox_;

        int runs;
                                   
        // OPENGL and QT //
        osgQt::GLWidget* glWidget1;
        osgQt::GLWidget* glWidget2;

        // osgQt::QWidget* qWidget1;
        // osgQt::QWidget* qWidget2;
        osg::ref_ptr<osg::Group> root;
        
        // Variables
        QStringList assetPaths;
        QStringList imagePaths;
        QStringList textureImagePaths;
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
        QString imageDir;
        QString texImageDir;
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

    Q_SIGNALS:
        void showImgs(int node);
        void showThumbs();
        void removeAllActive();

    public Q_SLOTS:
        // void config();
        // void pause();
        // void unpause();
        // void resume();
        // void suspend();
        void pullData(); 
        void increment();
        void recieveDiscreteGesture(QMap<int,int> events){};
        
    protected:
        // Timers //
        QTimer _timer;
        QTimer _dataTimer; 
    };    
}
 
#endif