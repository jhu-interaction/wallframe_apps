#ifndef _KBLevelEditor_h
#define _KBLevelEditor_h
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
// IOSTRAM //
#include <iostream>
// INTERNAL INCLUDES //
#include "AppBase.h"

#include "OSGObjectWrapper.h"
#include "StateObject.h"
#include "OSGObject.h"
#include "SkeletonObject.h"
#include "SphereObject.h"
#include "CubeObject.h"
#include "InteractionObject.h"
#include "PlanarObject.h"
#include "CursorObject.h"
#include "DockableObject.h"
#include "Holster.h"
#include "MoveTool.h"
#include "CameraTool.h"
#include "ActableObject.h"
#include "TexturedSphere.h"

namespace lair{

    class KBLevelEditor;
    class KBLevelEditorKeyboardHandler : public osgGA::GUIEventHandler
    {
    public:
        bool handle(const osgGA::GUIEventAdapter& ea,
                    osgGA::GUIActionAdapter& aa);
        virtual void accept(osgGA::GUIEventHandlerVisitor& v);
        virtual void setup(KBLevelEditor* appPt);
        KBLevelEditor* appPtr;
    };

    class KBLevelEditor : public AppBase, public osgViewer::Viewer
    {
        Q_OBJECT;
    public:
        KBLevelEditor( QWidget *par = NULL, QWidget *appManager = NULL, QString appID = "null", bool useKin = false);
        ~KBLevelEditor(){}
        
        // Qt Functions
        osgQt::GLWidget* addViewWidget( osg::Camera* camera, 
                                        osg::Node* scene );
        osg::Camera* createCamera( int x, int y, int w, int h,
                                   osgQt::GLWidget *QTObject,
                                   const std::string& name="",
                                   bool windowDecoration=false);
        virtual void paintEvent( QPaintEvent* event ) { frame(); }
        
        // Methods
        void updateEnvironment();
        
        // Camera Control Variables //
        osg::ref_ptr<osg::Camera> _camera;
        ServoCamera _camServo;
        osg::Vec3 _cameraOrbit;
        osg::Vec3 _cameraStart;

    private:

    public:

        // Icons //
        ObjectMap _actableIcons;
        ObjectMap _dockableIcons;
        SphereMap _cursorIcons;
        PlaneMap _planarCursors;

        ObjectList cubes_;
        ObjectList spheres_;

        // Texture Images //

        // Wrappers //
        OSGObjectBase* env_wrapper;
        OSGObjectBase* cube_wrapper_;
        OSGObjectBase* wall_wrapper_;
        OSGObjectBase* button_wrapper_;
                                   
        // OPENGL and QT //
        osgQt::GLWidget* glWidget;
        osg::ref_ptr<osg::Group> root;
        
        int runtime;
        bool paused;
        int _envState;

    Q_SIGNALS:
        void showImgs(int node);
        void showThumbs();
        void removeAllActive();

    public Q_SLOTS:
        void config();
        void pause();
        void unpause();
        void resume();
        void suspend();
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