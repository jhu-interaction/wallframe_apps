#include <modulair_app_pic_flyer/PicFlyer.h>

#include <modulair_osg_tools/osg_object_base.h>
#include <modulair_osg_tools/osg_planar_object.h>

#include <iostream>

using namespace std;
using namespace modulair;
using namespace PicFlyerApp;

// TODO All this code should be updated to use the gestures on AppUser.
// The model needs to be refactored. The manuscript stuff does not work. 
// But all of the app should be rewritten anyway. :)

// TODO: This should be elsewhere. Produced from /wallframe_user/scripts/wallframe_user_manager.py
enum {
  HEAD, NECK, TORSO, RIGHT_SHOULDER, LEFT_SHOULDER, RIGHT_ELBOW, LEFT_ELBOW, RIGHT_HAND,
  LEFT_HAND, RIGHT_HIP, LEFT_HIP, RIGHT_KNEE, LEFT_KNEE, RIGHT_FOOT, LEFT_FOOT 
};


// For testing
double camz = 1010;
double camz_prev = 0; 
double zoom_dist = 0; 
double wsoff = 480;

 // Default cover image.
osg::ref_ptr<osg::Image> PicFlyer::defaultCoverImage;
osg::ref_ptr<osg::Image> PicFlyer::defaultPageImage;

PicFlyer::PicFlyer(QString app_name, ros::NodeHandle nh, int event_deque_size,std::string app_id) : wallframe::WallframeAppBaseQt(app_name.toStdString(), nh, event_deque_size,app_id)
{ 
    //Initializing variables to defaults
    _imageLast = osg::Vec3(0,0,0);
    _handLast = osg::Vec2d(0,0);

    numActiveUsers = 0;
    paused = false;
    _snap = true;
    zoom_active = false;
    panning_active = false;
    panning_hand = QString("null");
    // Init button call history
    button_call.clear();
    button_call.push_back(1);
    button_call.push_back(1);

    //Set the environments state
    _envState = ENV_STATE_WAITING;

    //Makes the timers not repeat
    _delay.setSingleShot(true);
    _selectTimer.setSingleShot(true);
    _backTimer.setSingleShot(true);
    _turnTimer.setSingleShot(true);

    _arrayTimer.setSingleShot(true);
    _matrixTimer.setSingleShot(true);
    _relaxTimer.setSingleShot(true);

    //Connect slots and signals for the timers
    connect( &_selectTimer, SIGNAL(timeout()), this, SLOT(select()));
    connect( &_backTimer, SIGNAL(timeout()), this, SLOT(back()));
    connect( &_turnTimer, SIGNAL(timeout()), this, SLOT(turn()));

    _torsoLast = osg::Vec3d(0,0,0);
    pr_saved = osg::Vec3d(0,0,0);
    pl_saved = osg::Vec3d(0,0,0);

    this->paused = false;
    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_dataTimer, SIGNAL(timeout()), this, SLOT(updateApp()));
 }

bool PicFlyer::get_param(std::string param, std::string &value) {
  if (!node_.getParam(param, value)) {
      ROS_ERROR("%s: Parameter not found: %s (namespace: %s)",
		name_.c_str(), param.c_str(), node_.getNamespace().c_str());
      return false;
  } else {
    ROS_WARN_STREAM("PicFlyer: Parameter " << param << ": " << value);
    return true;
  }
}

bool PicFlyer::build() {
    std::string asset_path;

    if (!get_param("/modulair/apps/pic_flyer_app/paths/assets", asset_path)){
      return false;
    } else {
      asset_path_ = QString(asset_path.c_str());
    }

    std::string image_collection_path;

    if (!get_param("/modulair/apps/pic_flyer_app/paths/image_collections", image_collection_path)) {
      return false;
    }

    std::string image_collections;

    if (!get_param("/modulair/apps/pic_flyer_app/image_collections", image_collections)) {
      return false;
    }

    Model::CollectionStore::setDirectoryRoot(QString(image_collection_path.c_str()));
    Model::CollectionStore::setPictureCollectionIDs(QString(image_collections.c_str()).split(" "));

    QDir texture_dir = QDir(QDir(asset_path_).absoluteFilePath("textures"));
    QDir tooltip_dir = QDir(QDir(asset_path_).absoluteFilePath("tooltips"));

    config(texture_dir, tooltip_dir);

    return true;
}

PicFlyer::~PicFlyer() {
  for (unsigned int i = 0; i < _envWrapper->getNumChildren(); i++) {
    _envWrapper->removeChild(i);
  }

  _envWrapper = NULL;

  // TODO Incomplete.
}

void PicFlyer::config(QDir texture_dir, QDir tooltip_dir)
{
     root = new osg::Group;

    //Loads assets
    ROS_WARN_STREAM("<<< PicFlyer >>> Application Loading Textures.");
    LoadTextures(texture_dir);
    ROS_WARN_STREAM("<<< PicFlyer >>> Application Loading ToolTips.");
    loadToolTips(tooltip_dir);

    //Sets the Cursors
    _planarCursors["r"] = new PlanarObject(   -30,-30,0,30,30,0,
                                                &_assetTextures,0,
                                                osg::Vec3(0,0,35));
    _planarCursors["l"] = new PlanarObject(   -30,-30,0,30,30,0,
                                                &_assetTextures,0,
                                                osg::Vec3(0,0,35));

    //Sets the navigation buttons
    _page_arrows["left"] = new PlanarObject(  -40, -150, 0, 40, 150, 0,
                                                &_leftTex, 0,
                                                osg::Vec3(-910, 0, 0));
    _page_arrows["right"] = new PlanarObject( -40, -150, 0, 40, 150, 0,
                                                &_rightTex, 0,
                                                osg::Vec3(910, 0, 0));
    _planarCursors["r"]->setHidden();
    _planarCursors["l"]->setHidden();    

    //Cursors
    _cursorWrapper = new OSGObjectBase();
    _cursorWrapper->addChild(_planarCursors["r"]);
    _cursorWrapper->addChild(_planarCursors["l"]);


    // TRANSPARENCY ///////////////////////////////////////////////////////////
    osg::StateSet* ss = root->getOrCreateStateSet();
    ss->setMode(GL_BLEND, osg::StateAttribute::ON);
    ss->setRenderingHint(osg::StateSet::TRANSPARENT_BIN );
    ss->setMode( GL_DEPTH_TEST, osg::StateAttribute::ON );

    // ENVIRONMENT ////////////////////////////////////////////////////////////
    ROS_WARN_STREAM("<<< PicFlyer >>> Setting Up Environment... ");
    _envWrapper = new OSGObjectBase();
    _imageWrapper = new OSGObjectBase();
    _indexWrapper = new OSGObjectBase();
    _largeImageWrapper = new OSGObjectBase();
    _openingWrapper= new OSGObjectBase();
    _titleWrapper = new OSGObjectBase();

    // Set up title panel /////////////////////////////////////////////////////
    titleGeode=new osg::Geode();
    title=new osgText::Text();
    subtitle=new osgText::Text();
    pageInd= new osgText::Text();
    titleGeode->addDrawable(title);
    titleGeode->addDrawable(subtitle);
    titleGeode->addDrawable(pageInd);
    _titleWrapper->addChild(titleGeode);

    //TODO: fix these sizes
    title->setCharacterSize(30);
    subtitle->setCharacterSize(20);
    pageInd->setCharacterSize(16);


    //TODO: make this configurable
    title->setFont("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
    subtitle->setFont("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");
    pageInd->setFont("/usr/share/fonts/truetype/msttcorefonts/arial.ttf");

    title->setDrawMode(osgText::Text::TEXT);
    subtitle->setDrawMode(osgText::Text::TEXT);
    pageInd->setDrawMode(osgText::Text::TEXT);
    title->setAlignment(osgText::Text::CENTER_TOP);
    subtitle->setAlignment(osgText::Text::CENTER_TOP);
    pageInd->setAlignment(osgText::Text::CENTER_TOP);
    title->setPosition(osg::Vec3(-250,0,0));
    subtitle->setPosition(osg::Vec3(250,0,0));
    pageInd->setPosition(osg::Vec3(-600,0,0));

    _titleWrapper->setPosition(osg::Vec3(0,400,0));

    setTitle(   QString("Image Browser"), 
                QString("Select a collection to continue"), 
                1,1);
    updateButtons(1, 1);

    ROS_INFO_STREAM("<<< PicFlyer >>> Setting Up Images... ");
    //Load thumbnails for index page
    loadIndex();

    ROS_INFO_STREAM("<<< PicFlyer >>> Loading Index");

    int n=0;
    for(int py = 0; py < DISPLAY_HEIGHT; py++)
    {
        for (int px = 0; px < DISPLAY_WIDTH; px++)
        {
            if(n < _indexTextures.size())
            {

                double w= _indexTextures.at(n).get()->getImage()->s();
                double h= _indexTextures.at(n).get()->getImage()->t();

                thumbnailCalc(w, h);

                osg::Vec3 coord= osg::Vec3((px*THUMB_GAP_X) + THUMB_OFFSET_X, (py*THUMB_GAP_Y) + THUMB_OFFSET_Y, 0);

                PlanarObject* img;

		// TODO: Hack to center index thumbs
		coord[0] += 370;
		coord[1] -= 200;

                img = new PlanarObject( -w/2, -h/2, 0, w/2, h/2, 0,
                                        &_indexTextures, n, coord );

                img->setText(_indexLabels[n], THUMB_TEXT_SIZE, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec3(0, THUMB_TEXT_OFFSET,0));
                img->setTransparency(1.0);
                _indexThumbs.push_back(img);


                n++;
            }
        }
    }



    // Highlights potential selection.
    _highlight = new PlanarObject(   -THUMBNAIL_SIDE/2,-THUMBNAIL_SIDE/2,0, THUMBNAIL_SIDE/2, THUMBNAIL_SIDE/2,0,
                                    &_assetTextures,27,
                                    osg::Vec3(0,0,.25));
    _highlight->setTransparency(0);
    _highlight->setHidden();

    _highlight_active = new PlanarObject(   -THUMBNAIL_SIDE/2,-THUMBNAIL_SIDE/2,0, THUMBNAIL_SIDE/2, THUMBNAIL_SIDE/2,0,
                                    &_assetTextures,30,
                                    osg::Vec3(0,0,.25));
    _highlight_active->setTransparency(0);
    _highlight_active->setHidden();

    _pleaseWait = new PlanarObject(   -200,-200,0,200,200,0,
                                    &_assetTextures,28,
                                    osg::Vec3(400,0,1));
    _pleaseWait->setText(QString("Please Wait"),10,osg::Vec4(0,0,0,1),osg::Vec3(0,0,0));                                    
    _pleaseWait->setHidden();

    //Original value: 200. May change for testing.
    camz = 200; 

    //Actually displays index thumbnails
    ROS_INFO_STREAM("<<< PicFlyer >>> Loading Textures... ");
    for (int i = 0; i < _indexThumbs.size(); ++i){
        _indexWrapper->addChild(_indexThumbs[i]);
    }

    _envWrapper->addChild(_imageWrapper); 
    _envWrapper->addChild(_indexWrapper);
    _envWrapper->addChild(_cursorWrapper);
    _envWrapper->addChild(_highlight);
    _envWrapper->addChild(_highlight_active);
    _envWrapper->addChild(_pleaseWait);
    _envWrapper->addChild(_largeImageWrapper); 
    _envWrapper->addChild(_openingWrapper);
    _envWrapper->addChild(_titleWrapper);

    _envWrapper->addChild(_page_arrows["left"]);
    _envWrapper->addChild(_page_arrows["right"]);

    root->addChild(_envWrapper);
    
    // CAMERA ATTACHED OBJECTS ////////////////////////////////////////////////
    ROS_INFO_STREAM("<<< PicFlyer >>> Setting Up Camera Attached Objects.");
    _cameraAttachedObjects = new OSGObjectBase();
    root->addChild(_cameraAttachedObjects);

    // GL QT WIDGET ///////////////////////////////////////////////////////////
    glWidget = new osgQt::GLWidget(this);
    glWidget = addViewWidget( createCamera(0,0,60,40,
                              glWidget, "mainCamera", false), root);
    QGridLayout* grid = new QGridLayout;
    grid->addWidget( glWidget);
    setLayout( grid );
    ROS_INFO_STREAM("<<< PicFlyer >>> Created Widget");

    // TOOLTIP QWIDGETS ///////////////////////////////////////////////////////

    // TODO revisit this sizing and positioning
    int INFOBAR_HEIGHT = 100;

    tooltip1 = new QLabel(this);
    tooltip1->move(INFOBAR_HEIGHT*2,height_-INFOBAR_HEIGHT-height_/10);
    tooltip1->resize(width_/4-INFOBAR_HEIGHT*4,height_/10);
    tooltip1->setPixmap(*(toolTipImages[16]));
    tooltip1->setScaledContents(true); 
    tooltip1->hide();

    tooltip2 = new QLabel(this);
    tooltip2->move(width_/4+INFOBAR_HEIGHT*2,height_-INFOBAR_HEIGHT-height_/10);
    tooltip2->resize(width_/4-INFOBAR_HEIGHT*4,height_/10);
    tooltip2->setPixmap(*(toolTipImages[16]));
    tooltip2->setScaledContents(true);
    tooltip2->raise();
    tooltip2->show();

    tooltip3 = new QLabel(this);
    tooltip3->move(width_/2+INFOBAR_HEIGHT*2,height_-INFOBAR_HEIGHT-height_/10);
    tooltip3->resize(width_/4-INFOBAR_HEIGHT*4,height_/10);
    tooltip3->setPixmap(*(toolTipImages[16]));
    tooltip3->setScaledContents(true);
    tooltip3->raise();
    tooltip3->show();

    tooltip4 = new QLabel(this);
    tooltip4->move(width_/2+width_/4+INFOBAR_HEIGHT*2,height_-INFOBAR_HEIGHT-height_/10);
    tooltip4->resize(width_/4-INFOBAR_HEIGHT*4,height_/10);
    tooltip4->setPixmap(*(toolTipImages[16]));
    tooltip4->setScaledContents(true);
    tooltip4->hide();

    ROS_WARN_STREAM("<<< PicFlyer >>> Configured Successfully");
}

osg::Vec2d PicFlyer::mapEnvPos(Eigen::Vector3d in){
  osg::Vec2d result;
    
  result[0] = in[0];
  result[1] = in[1];

  return result;
}

/**
 * Sets a tooltip image based on a key and the tooltip pane id
 */
void PicFlyer::setTooltip(QString key, int id)
{
    QLabel* tip;
    if(id == 1){
        tip = tooltip1;
    }else if(id == 2){
        tip = tooltip2;
    }else if(id == 3){
        tip = tooltip3;
    }else{
        tip = tooltip4;
    }

    if(key != QString("hide")){
        if(key == QString("highlight_collection")){
            tip->setPixmap(*(toolTipImages[3]));
        }else if(key == QString("highlight_image")){
            tip->setPixmap(*(toolTipImages[2]));
        }else if(key == QString("forearm_select")){
            tip->setPixmap(*(toolTipImages[7]));
        }else if(key == QString("go_back")){
            tip->setPixmap(*(toolTipImages[11]));
        }else if(key == QString("zoom_enter")){
            tip->setPixmap(*(toolTipImages[0]));
        }else if(key == QString("zoom_leave")){
            tip->setPixmap(*(toolTipImages[1]));
        }else if(key == QString("zoom_mode")){
            tip->setPixmap(*(toolTipImages[10]));
        }else if(key == QString("pan_enter")){
            tip->setPixmap(*(toolTipImages[4]));
        }else if(key == QString("pan_leave")){
            tip->setPixmap(*(toolTipImages[8]));
        }else if(key == QString("pan_mode")){
            tip->setPixmap(*(toolTipImages[6]));
        }else if(key == QString("pan_active")){
            tip->setPixmap(*(toolTipImages[5]));
        }else if(key == QString("zoom_how")){
            tip->setPixmap(*(toolTipImages[9]));
        }else if(key == QString("back_active")){
            tip->setPixmap(*(toolTipImages[12]));
        }else if(key == QString("select_active")){
            tip->setPixmap(*(toolTipImages[13]));
        }else if(key == QString("zoom_active")){
            tip->setPixmap(*(toolTipImages[14]));
        }else if(key == QString("zoom_start")){
            tip->setPixmap(*(toolTipImages[15]));
        }else if(key == QString("wait_loading")){
            tip->setPixmap(*(toolTipImages[16]));
        }
        tip->show();
        tip->raise();
    }else{
        tip->hide();
    }

}

void PicFlyer::hideTooltip(int id)
{
    if(id == 1){
        tooltip1->hide();
        tooltip1->repaint();
    }
    if(id == 2){
        tooltip2->hide();
        tooltip2->repaint();
    }
    if(id == 3){
        tooltip3->hide();
        tooltip3->repaint();
    }
}

void PicFlyer::updateApp()
{
  updateEnvironment();
}

/**
 * Updates the view. Main driving force of the program.
 */
void PicFlyer::updateEnvironment()
{   
    //Updates skeleton if kinect is being used
  
    Eigen::Vector3d torso;
    Eigen::Vector3d right;
    Eigen::Vector3d rel;
    Eigen::Vector3d lel;
    Eigen::Vector3d left;
    Eigen::Vector3d rAbs;
    Eigen::Vector3d lAbs;

    //Should be the state that called it for selection, or the one it will be on a back
    static previousState_t prevManState;

    //A way to keep track of the current collection. Useful for going back.
    static QString activeCollection;
    static QString activeManuscript;

    static turn_t dir;

    AppUser user;

    if (!this->getFocusedUser(user)) {
      return;
    }

    // TODO Unsure which of this should be jtPosById and jtPosBodyById
    torso = user.jtPosById(TORSO);
    right = user.jtPosById(RIGHT_HAND);
    rel = user.jtPosById(RIGHT_ELBOW);
    lel = user.jtPosById(LEFT_ELBOW);
    left = user.jtPosById(LEFT_HAND);

    rAbs = user.jtPosBodyById(RIGHT_HAND).cwiseAbs();
    lAbs = user.jtPosBodyById(LEFT_HAND).cwiseAbs();

    double dist, d1,d2, distBack;
    Eigen::Vector3d v,v1,v2, vecBack;
    dist=INT_MAX;
    d1=INT_MAX;
    d2=INT_MAX;
    distBack=INT_MAX;


    //NOTE: Selection is now done from highlight location, rather than from cursor

    switch(_envState)
    {
        //Starts the camera
        case ENV_STATE_WAITING:
	  _camServo.startServo(_camera, osg::Vec3(0,0,VIEWPORT_DEFAULT_ZOOM),1);
            camz = VIEWPORT_DEFAULT_ZOOM;
            _envState = ENV_STATE_INDEX;
            if(!_relaxTimer.isActive())
                _relaxTimer.start(3000);
            break;

        //Selects collection with gesture.
        case ENV_STATE_INDEX:
        {
            setTooltip("highlight_collection",2);
            if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                setTooltip("forearm_select",3);
            else
                setTooltip("hide",3);

            updateCursors();
            if(_planarCursors["r"]->isVisible())
            {
                v = left - rel;
                dist = v.norm();
            }
            else if(_planarCursors["l"]->isVisible())
            {
                v = right - lel;
                dist = v.norm();
            }


            if(dist<CLICK_THRESHOLD && !_relaxTimer.isActive())
            {
                _highlight->setTransparency(0);
                _highlight_active->setTransparency(.25);
                _selectTimer.start(SELECTION_DELAY);
                _imageLocation = _highlight->getPosition();

                prevManState= INDEX_SELECTION;
                _envState = ENV_STATE_WAITING_SELECTED;
                setTooltip("select_active",3);
            }

            break;     


        }

        //Displays images in array mode
        case ENV_STATE_ARRAY:
 
            // Left Hand selection test. Sets five second timer to confirm selection
            updateCursors();
            if(_planarCursors["r"]->isVisible())
            {
                v = left - rel;
                dist = v.norm();
            }
            else if(_planarCursors["l"]->isVisible())
            {
                v = right - lel;
                dist = v.norm();
            }  

            if(dist<CLICK_THRESHOLD && !_relaxTimer.isActive()){
                _highlight->setTransparency(0);
                _highlight_active->setTransparency(.25);
                _selectTimer.start(SELECTION_DELAY);
                _imageLocation = _highlight->getPosition();

                _envState = ENV_STATE_WAITING_SELECTED;
                setTooltip("select_active",3);
                break;     
            }
            //Go Back
            //TODO: Better Back Gesture
            vecBack = right -  left;
            distBack = vecBack.norm();

            if(distBack<200 && !_relaxTimer.isActive() && lAbs[2] > 175 && rAbs[2] > 175)
            {
                _backTimer.start(SELECTION_DELAY);
                _envState= ENV_STATE_WAITING_BACK;
                    // setTooltip("back_active",1);
                    setTooltip("back_active",4);
                break;
            }

	    break;


        //Invoked while timing for selection. Fails if hand strays too far.
        case ENV_STATE_WAITING_SELECTED:
            // Right select failed
            if(_planarCursors["r"]->isVisible())
            {
                v = left - rel;
                dist = v.norm();
            }
            else if(_planarCursors["l"]->isVisible())
            {
                v = right - lel;
                dist = v.norm();
            }
            if(dist>CLICK_THRESHOLD){
                _selectTimer.stop();
                setTooltip("forearm_select",3);
                _highlight->setTransparency(0.25);
                _highlight_active->setTransparency(0.0);
                if (prevManState==PAGE_TURNER){
                    _envState=ENV_STATE_PAGE_TURNER;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                }else{
                    _envState = ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                }
                break;         
            }

            
            break;

        case ENV_STATE_WAITING_BACK:
            // Back Fail check
            vecBack = right - left;
            distBack = vecBack.norm();

            if(distBack>200 || lAbs[2] < 175 || rAbs[2] < 175)
            {
                _backTimer.stop();
                ROS_INFO_STREAM("<<< PicFlyer >>> Back Failed");
                // setTooltip("go_back",1);
                setTooltip("go_back",4);
                if (prevManState==LARGE_IMAGE || prevManState== LARGE_IMAGE_T){
                    _envState = ENV_STATE_IMAGE;
                    setTooltip("zoom_enter",2);
                    setTooltip("pan_enter",3);
                }else if (prevManState==PAGE_TURNER){
                    _envState = ENV_STATE_PAGE_TURNER;
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                }else{
                    _envState = ENV_STATE_ARRAY;
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                }
                break;         
            }

            
            break;

        case ENV_STATE_TURN:
            switch(prevManState)
            {

	      setTooltip("highlight_image",2);

	      if (_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible()) {
		setTooltip("forearm_select",3);
	      } else {
		setTooltip("hide",3);
	      }

	      setTooltip("go_back",4);
	      
	    case INDEX_SELECTION:
	    case LARGE_IMAGE:
	    case LARGE_IMAGE_T:
	      break;
    
                case MANUSCRIPT_SELECTION:
                    if(dir == TURN_RIGHT) {
                        col_start += NUM_THUMBNAILS;
                    } else {
                        col_start -= NUM_THUMBNAILS;
                    }

                    if (col_start < 0) {
                        col_start = 0;
                    }
                    showManuscriptCollection(activeCollection, col_start);
                    _envState=ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;
                case COLLECTION_SELECTION:
                    if(dir == TURN_RIGHT) {
                        col_start += NUM_THUMBNAILS;
                    } else {
                        col_start -= NUM_THUMBNAILS;
                    }

                    if (col_start < 0) {
                        col_start = 0;
                    }
		            showCollection(activeCollection, col_start);
                    _envState=ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;
                case OPENING_THUMBNAILS:
           		  if(dir == TURN_RIGHT) {
            		    pages_start += NUM_THUMBNAILS;
            		  } else {
            		    pages_start -= NUM_THUMBNAILS;
            		  }

            		  if (pages_start < 0) {
            		    pages_start = 0;
            		  }
            		  showPageCollection(activeCollection, activeManuscript, pages_start);
                    _envState=ENV_STATE_ARRAY;
                    break;
                case PAGE_TURNER:
                    loadTurn(dir);
                    _envState=ENV_STATE_PAGE_TURNER;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;
            }

            break;

        //If image is selected

	// TODO all this findImage stuff is uneeded because _highlight has closest!
        case ENV_STATE_SELECT:
        {
            // setTooltip("go_back",1);
            setTooltip("highlight_image",2);
            if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                setTooltip("forearm_select",3);
            else
                setTooltip("hide",3);
            setTooltip("go_back",4);

            _highlight->setTransparency(0.0);
            _highlight_active->setTransparency(0.0);

    	    PlanarObject *cursor = _planarCursors["r"];

            if(!cursor->isVisible()) {
                cursor = _planarCursors["l"];
            } 

            if(cursor->isVisible()) {
                PlanarObject *right =_page_arrows["right"];
                PlanarObject *left =_page_arrows["left"];

                if(left == dynamic_cast<PlanarObject*>(_highlight->getUserData())) {
                    dir=TURN_LEFT;
                    _envState=ENV_STATE_TURN;
                    return;
                } else if(right == dynamic_cast<PlanarObject*>(_highlight->getUserData())) {
                    dir=TURN_RIGHT;
                    _envState=ENV_STATE_TURN;
                    return;
        		}
            }

            switch(prevManState)
            {
                case INDEX_SELECTION:
                {
                    int nearInd=0;
                    nearInd = findNearestIndex(_imageLocation);
                    activeCollection=_indexNames[nearInd];
                    Model::collectionType_t tempType= Model::CollectionStore::getCollectionType(activeCollection);

                    if(tempType==Model::UNORDERED_COLLECTION)
                    {
                        prevManState = COLLECTION_SELECTION;
			            col_start = 0;
                        showCollection(activeCollection, 0);
                    }
                    else if (tempType==Model::MANUSCRIPT_COLLECTION)
                    {
                        prevManState=MANUSCRIPT_SELECTION;
			            col_start = 0;
                        showManuscriptCollection(activeCollection, 0);
                    }

                    _envState = ENV_STATE_ARRAY;
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    // prevManState = INDEX_SELECTION;
                    // for(int i = 0; i< _indexThumbs.size();i++)
                    //     _indexThumbs[i]->setHidden();
                    _indexWrapper->setHidden();
                    _imageWrapper->setVisible();
                    break;

                }
            
                case COLLECTION_SELECTION:
                    loadLarge(_imageLocation);
                    //May need to go into if statement
                    _imageWrapper->setHidden();
                    _planarCursors["l"]->setHidden();
                    _planarCursors["r"]->setHidden();
                    _highlight->setHidden();
                    _highlight_active->setHidden();
		            _highlight->setUserData(0);
                    _largeImageWrapper->setVisible();
                    prevManState=LARGE_IMAGE;
                    _envState = ENV_STATE_IMAGE;
                    // setTooltip("go_back",1);
                    setTooltip("pan_enter",2);
                    setTooltip("zoom_enter",3);
                    setTooltip("go_back",4);
                    break;

                //Is currently: Manuscript Collection
                case MANUSCRIPT_SELECTION:
                {
                    int index;
                    QString c, id;
                    findImage(_imageLocation, c, index, id);
                    activeManuscript=id;
		            pages_start = 0;
                    showPageCollection(c, id, 0);
                    _imageWrapper->setVisible();
                    prevManState=OPENING_THUMBNAILS;
                    _envState = ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;
                }

                //Is currently: Opening Browser
                case OPENING_THUMBNAILS:
                    loadPageTurner(_imageLocation);
                    _imageWrapper->setHidden();
                    _openingWrapper->setVisible();
                    _highlight->setHidden();
                    _highlight_active->setHidden();
		            _highlight->setUserData(0);
                    _envState = ENV_STATE_PAGE_TURNER;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;

                //Is Currently: Page Turner
                case PAGE_TURNER:
                    loadLargeFromTurner(dir);
                    //May need to go into if statement
                    _openingWrapper->setHidden();
                    _planarCursors["l"]->setHidden();
                    _planarCursors["r"]->setHidden();
                    _highlight->setHidden();
                    _highlight_active->setHidden();
		            _highlight->setUserData(0);
                    _largeImageWrapper->setVisible();
                    // prevManState=PAGE_TURNER;
                    prevManState=LARGE_IMAGE_T;
                    _envState = ENV_STATE_IMAGE;
                    // setTooltip("go_back",1);
                    setTooltip("pan_enter",2);
                    setTooltip("zoom_enter",3);
                    setTooltip("go_back",4);
                    break;

                case LARGE_IMAGE:
                    break;

                case LARGE_IMAGE_T:
                    break;
            }  
            break;
        }

        //If attempting to go back
        case ENV_STATE_BACK:
            switch(prevManState)
            {
               case INDEX_SELECTION:
                    _envState=ENV_STATE_ARRAY;
                    // setTooltip("hide",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("hide",4);
                    break;
            
                case COLLECTION_SELECTION:
                case MANUSCRIPT_SELECTION:
                    //TODO: Need to take index showing out of config and put into a show method, so it can be gone back to
                    //showIndex() <--might not actually need, index wrapper never dissapears
                    _imageWrapper->setHidden();
                    _indexWrapper->setVisible();
                    _envState = ENV_STATE_INDEX;
                    // setTooltip("hide",1);
                    setTooltip("highlight_collection",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("hide",4);
        		    setTitle(QString("Image Browser"), QString("Select a collection to continue"),1,1);
        		    updateButtons(1, 1);
                    break;

                //Is currently: Opening Browser
                case OPENING_THUMBNAILS:
                    prevManState=MANUSCRIPT_SELECTION;
                    showManuscriptCollection(activeCollection, col_start);
                    _envState = ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;

                //Is Currently: Page Turner
                case PAGE_TURNER:
                    _openingWrapper->setHidden();
                    _imageWrapper->setVisible();
                    _highlight->setVisible();
                    _highlight_active->setVisible();
                    prevManState=OPENING_THUMBNAILS;
                    _envState = ENV_STATE_ARRAY;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;

                case LARGE_IMAGE_T:
                    _titleWrapper->setVisible();
                    _largeImageWrapper->setHidden();
                    _openingWrapper->setVisible();
                    _envState = ENV_STATE_PAGE_TURNER;
                    _envWrapper->setPos3DAbs(osg::Vec3(0,0,0));
                    prevManState=PAGE_TURNER;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_image",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;

                case LARGE_IMAGE:
                    _titleWrapper->setVisible();
                    _largeImageWrapper->setHidden();
                    _imageWrapper->setVisible();
                    _envState = ENV_STATE_ARRAY;
                    _envWrapper->setPos3DAbs(osg::Vec3(0,0,0));
                    prevManState=COLLECTION_SELECTION;
                    // setTooltip("go_back",1);
                    setTooltip("highlight_collection",2);
                    if(_planarCursors["r"]->isVisible() || _planarCursors["l"]->isVisible())
                        setTooltip("forearm_select",3);
                    else
                        setTooltip("hide",3);
                    setTooltip("go_back",4);
                    break;
            }

            _camServo.startServo(_camera,osg::Vec3(0,0,VIEWPORT_DEFAULT_ZOOM),1);
            camz = VIEWPORT_DEFAULT_ZOOM;

            break;

        case ENV_STATE_PAGE_TURNER:
        {
            updateCursors();

            prevManState=PAGE_TURNER;
            // Hand to elbow selection test. Sets five second timer to confirm selection
            v1 = left - rel;
            d1 = v1.norm();
            v2 = right - lel;
            d2 = v2.norm();

            if(d1<CLICK_THRESHOLD && !_relaxTimer.isActive())
            {
                _envState = ENV_STATE_WAITING_SELECTED;
                _selectTimer.start(SELECTION_DELAY);
                dir=TURN_LEFT;
                    setTooltip("select_active",2);     
            }

            else if (d2<CLICK_THRESHOLD && !_relaxTimer.isActive())
            {
                _envState= ENV_STATE_WAITING_SELECTED;
                _selectTimer.start(SELECTION_DELAY);
                dir=TURN_RIGHT;
                    setTooltip("select_active",2);
            }

            vecBack = right - left;
            distBack=vecBack.norm();

            if(distBack<200 && !_relaxTimer.isActive() && lAbs[2] > 175 && rAbs[2] > 175)
            {
                _backTimer.start(SELECTION_DELAY);
                _envState= ENV_STATE_WAITING_BACK;
                    // setTooltip("back_active",1);
                    setTooltip("back_active",4);
                break;
            }

            break;
        }

        //When viewing an individual image, check for various controls
        case ENV_STATE_IMAGE:
            //Hide certain ui elements
            _titleWrapper->setHidden();
            _page_arrows["left"]->setHidden();
            _page_arrows["right"]->setHidden();

            // Test to see if user has moved forward to zoom mode
            if(torso[2] < 2300){
                _envState = ENV_STATE_IMAGE_ZOOM;
                // setTooltip("zoom_mode",1);
                setTooltip("zoom_start",2);
                setTooltip("zoom_leave",3);
                setTooltip("zoom_mode",4);
                break;
            }

            // Check PAN gesture
            v1 = right - lel;
            v2 = left - rel;
            d1 = v1.norm();
            d2 = v2.norm();

            if(d1<190 && !_relaxTimer.isActive()){
                panning_hand = QString("left");
                _handLast = mapEnvPos(left);
                _imageLast = _envWrapper->getPosition();
                _envState = ENV_STATE_IMAGE_PAN;
                setTooltip("pan_mode",1);
                setTooltip("pan_enter",2);
                setTooltip("pan_leave",3);
                setTooltip("pan_mode",4);
                break; 
            }else if(d2<190 && !_relaxTimer.isActive()){
                panning_hand = QString("right");
                _handLast = mapEnvPos(right);
                _imageLast = _envWrapper->getPosition();
                _envState = ENV_STATE_IMAGE_PAN;
                setTooltip("pan_mode",1);
                setTooltip("pan_enter",2);
                setTooltip("pan_leave",3);
                setTooltip("pan_mode",4);
                break; 
            }else{
                panning_hand = QString("null");
            }

            // CHECK BACK GESTURE
            vecBack = right - left;
            distBack=vecBack.norm();

            if(distBack < 200 && !_relaxTimer.isActive() && lAbs[2] > 175 && rAbs[2] > 175)
            {
                _backTimer.start(SELECTION_DELAY);
                _envState= ENV_STATE_WAITING_BACK;
                    // setTooltip("back_active",1);
                    setTooltip("back_active",4);
                    // Show the arow buttons with the last called parameters (sort of a hack)
                    updateButtons(button_call[0],button_call[1]);
                break;
            }

            break;

        /// ZOOM IMAGE BEHAVIOR
        case ENV_STATE_IMAGE_ZOOM:

            // Return to normal image mode if user steps back
            if(torso[2]>2300){
                _envState = ENV_STATE_IMAGE;
                // setTooltip("go_back",1);
                setTooltip("pan_enter",2);
                setTooltip("zoom_enter",3);
                setTooltip("go_back",4);
                zoom_active = false;
                if(!_relaxTimer.isActive())
                    _relaxTimer.start(3000);
                break;
            }

            // Zoom Behavior
	    // Changed hand check from right.SumOfElements() !=0 to !right.izZero()
            if(rAbs[2] > 350 && !right.isZero() 
               && lAbs[2] > 350 && !left.isZero()) {
                setTooltip("zoom_how",2);
                v = left - right;
                dist = v.norm();


                // Save init hand and zoom distance
                if(!zoom_active){
                    ROS_INFO_STREAM("Camera Starting Zoom::: " << camz);
                    zoom_active = true;
                    zoom_dist = dist;
                    camz_prev = camz; 
                }
                // Zoom relative to init hand distance
                camz = camz_prev - (dist-zoom_dist)/3;
                
                // Cap minimum and maximum zoom distance
                if(camz < 100)
                    camz = 100;
                if(camz > 500)
                    camz = 500;

                _camera->setViewMatrixAsLookAt( osg::Vec3d( 0,0,camz ), // eye
                                                osg::Vec3d( 0,0,0 ),  // look
                                                osg::Vec3d( 0,1,0 )); // up
                setTooltip("zoom_how",2);
            }else{
                zoom_active = false;
                setTooltip("zoom_start",2);
            }

            break;

        //If panning
        case ENV_STATE_IMAGE_PAN:
            // ACTION //
            if(panning_hand == QString("left")){
	      if(lAbs[2] > 250 && !left.isZero()){
                    panning_active = true;
                    osg::Vec2d handCurrent = mapEnvPos(left);
                    osg::Vec2d handCommand = handCurrent-_handLast;

                    handCommand *= (.75 - (camz - 100)/600);

                    osg::Vec3 command = osg::Vec3(handCommand[0],handCommand[1],0)+_imageLast;
                    _envWrapper->setPosition(command);
                    setTooltip("pan_active",2);

                }
                // Check PAN gesture ended //
                v1 = right - lel;
                d1 = v1.norm();

                if(d1 > 190){

                    panning_active = false;
                    _envState = ENV_STATE_IMAGE;
                        // setTooltip("go_back",1);
                        setTooltip("pan_enter",2);
                        setTooltip("zoom_enter",3);
                        setTooltip("go_back",4);
                    panning_hand = QString("null");
                    break; 
                }
            }else if(panning_hand == QString("right")){
	      if(rAbs[2] > 250 && !right.isZero()){
                    panning_active = true;
                    osg::Vec2d handCurrent = mapEnvPos(right);
                    osg::Vec2d handCommand = handCurrent-_handLast;

                    handCommand *= (.75 - (camz - 100)/600);

                    osg::Vec3 command = osg::Vec3(handCommand[0],handCommand[1],0)+_imageLast;
                    _envWrapper->setPosition(command);
                    setTooltip("pan_active",2);

                }
                // Check PAN gesture ended //
                v1 = left - rel;
                d1 = v1.norm();

                if(d1 > 190){

                    panning_active = false;
                    _envState = ENV_STATE_IMAGE;
                        // setTooltip("go_back",1);
                        setTooltip("pan_enter",2);
                        setTooltip("zoom_enter",3);
                        setTooltip("go_back",4);
                    panning_hand = QString("null");
                    break; 
                }
            }
            break;
    }
    _torsoLast = osg::Vec3d(torso[0], torso[1], torso[1]);
}

/**
 * Callback that tells the environment an image has been selected.
 */
void PicFlyer::select()
{
    _envState = ENV_STATE_SELECT; 
    if(!_relaxTimer.isActive())
        _relaxTimer.start(3000);  
}

void PicFlyer::back()
{
    _envState = ENV_STATE_BACK;
    if(!_relaxTimer.isActive())
        _relaxTimer.start(3000);
}

void PicFlyer::turn()
{
    _envState = ENV_STATE_TURN;
    if(!_relaxTimer.isActive())
        _relaxTimer.start(3000);
}

// You have got to be kidding me...
void PicFlyer::set_z_position(OSGObjectBase *p, int z) {
    osg::Vec3 b = p->getPosition();
    osg::Vec3d back = osg::Vec3d(b[0], b[1], z);
    p->setPos3DAbs(back);
}

void PicFlyer::updateCursor(PlanarObject *cursor, Eigen::Vector3d kinect_hand_position, osg::Group* group)
{
    osg::Vec2d screen_hand_pos = mapEnvPos(kinect_hand_position);
    osg::Vec2d screen_cursor_pos = screen_hand_pos;

    cursor->setPos2DAbs(osg::Vec3(screen_cursor_pos[0], screen_cursor_pos[1], 0));
    cursor->setVisible();

    // Highlight and popout closest object

    double closest;
    OSGObjectBase *closest_object = 0;

    for (unsigned int i = 0; i < group->getNumChildren(); i++) {
        OSGObjectBase* p = dynamic_cast<OSGObjectBase*>(group->getChild(i));

        if (!p) {
            continue;
        }

        osg::Vec2d pos = p->getPos2D();
        osg::Vec2d v;
        v = screen_cursor_pos - pos;
        double dist = v.normalize();

        if (!closest_object || dist < closest) {
            closest = dist;
            closest_object = p;
        }
    }

    // TODO: Awful hack to handle buttons

    PlanarObject *left = _page_arrows["left"];

    if (left->isVisible()) {
        osg::Vec2d pos = left->getPos2D();
        osg::Vec2d v;
        v = screen_cursor_pos - pos;
        double dist = v.normalize();

        if (!closest_object || dist < closest) {
            closest = dist;
            closest_object = left;
        }
    }

    PlanarObject *right = _page_arrows["right"];
    
    if (right->isVisible()) {
        osg::Vec2d pos = right->getPos2D();
        osg::Vec2d v;
        v = screen_cursor_pos - pos;
        double dist = v.normalize();

        if (!closest_object || dist < closest) {

            closest = dist;
            closest_object = right;
        }
    }

    // Pop out closest object and pop back in old closest object

    if (closest_object) {
        OSGObjectBase* old_closest_object =
                dynamic_cast<OSGObjectBase*>(_highlight->getUserData());

    	if (old_closest_object != closest_object) {
                if (old_closest_object) {
                    set_z_position(old_closest_object, 0);

    		// TODO hackupdateButtons
    		if (old_closest_object->isVisible()) {
    		  old_closest_object->setScaleAll(1.0);
    		} else {
    		  old_closest_object->setScaleAll(0.0);
    		}
                }

                set_z_position(closest_object, 30);
                closest_object->setScaleAll(1.5);

                osg::Vec3d pos = closest_object->getPos3D();
                pos[2] = 20;

                _highlight->setUserData(closest_object);
                _highlight->setPos3DAbs(pos);
                _highlight_active->setPos3DAbs(pos);
                _highlight->setVisible();
                _highlight_active->setVisible();
                _highlight->setTransparency(.25);
                _highlight->setScaleAll(1.75);
                _highlight_active->setScaleAll(1.75);
    	}
    }

    if (closest_object == left || closest_object == right) {
        if(closest_object == left){
            left->setTexture(1);
        }
        if(closest_object == right){
            right->setTexture(1);
        }
      _highlight->setHidden();
      _highlight_active->setHidden();
      set_z_position(closest_object, 0);
      closest_object->setScaleAll(1.2);
    }else{
        left->setTexture(0);
        right->setTexture(0);
    }
}


/**
 * Updates cursor position
 */
void PicFlyer::updateCursors()
{
    Eigen::Vector3d torso;
    Eigen::Vector3d right;
    Eigen::Vector3d left;
    Eigen::Vector3d rAbs;
    Eigen::Vector3d lAbs;

    AppUser user;

    if (!this->getFocusedUser(user)) {
      return;
    }

    // TODO Unsure which of this should be jtPosById and jtPosBodyById
    torso = user.jtPosById(TORSO);
    right = user.jtPosById(RIGHT_HAND);
    left = user.jtPosById(LEFT_HAND);

    // This seems to be what lair used to do for pts3D_norm
    rAbs = user.jtPosBodyById(RIGHT_HAND).cwiseAbs();
    lAbs = user.jtPosBodyById(LEFT_HAND).cwiseAbs();


    OSGObjectBase* group;

    if (_indexWrapper->isVisible()) {
        group =_indexWrapper;
    } else if(_imageWrapper->isVisible()) {
        group =_imageWrapper;
    } else {
        return;
    }

    PlanarObject *right_cursor = _planarCursors["r"];
    PlanarObject *left_cursor = _planarCursors["l"];

    // Check that the users is in the working area
    if(torso[2] < WKSP_MAX_Z_PIC && torso[2] > WKSP_MIN_Z_PIC) {
        if(rAbs[2] > lAbs[2]) {
	  if(rAbs[2] > 220 && !right.isZero()) {
                updateCursor(right_cursor, right, group);
                left_cursor->setHidden();
                right_cursor->setVisible();
            }else{
                right_cursor->setHidden();
            }
        } else {
	  if(lAbs[2] > 220 && !left.isZero()) {
                updateCursor(left_cursor, left, group);
                right_cursor->setHidden();
                left_cursor->setVisible();
            }else{
                left_cursor->setHidden();
            }
        }
    }
}

bool PicFlyer::start(){
  // Start up app
  _timer.start( 10 );
  _dataTimer.start(10);
  ROS_WARN_STREAM("<<< PicFlyer >>> Timers Started");
  return true;
}


/**
 * Pauses the application. Required by AppBase.
 */
bool PicFlyer::pause()
{
    this->hide();
    _timer.stop();
    _dataTimer.stop();
    this->update();
    paused = true;
    ROS_INFO_STREAM("<<< PicFlyer >>> Pausing");
    return true;
}

/**
 * Resumes the application. Required by AppBase.
 */
bool PicFlyer::resume()
{
    _timer.start(10);
    _dataTimer.start(30);
    this->show();
    this->glWidget->show();
    this->update();
    ROS_INFO_STREAM("<<< PicFlyer >>> Resumed");
    return true;
}

/**
 * Loads asset textures
 */
void PicFlyer::LoadTextures(QDir texture_dir)
{
  QStringList assetPaths;

    QStringList assetFiles = texture_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    for (int i=0; i<assetFiles.count(); i++){
        assetPaths << texture_dir.absoluteFilePath(assetFiles[i]);
    }

    std::string base = texture_dir.absolutePath().toStdString();

    ROS_INFO_STREAM("Loading Textures from " << base);

    osg::ref_ptr<osg::Image> img;    
    for (int i = 0; i < assetPaths.size(); ++i){
        img = osgDB::readImageFile(assetPaths.at(i).toStdString());
        if(!img)
            ROS_INFO_STREAM(assetPaths.at(i).toStdString()<<" not loaded!");
        this->_assetTextures.push_back(new osg::TextureRectangle(img));
    }

    _leftTex.push_back(new osg::TextureRectangle(osgDB::readImageFile(base + "/previous_inactive.png")));
    _leftTex.push_back(new osg::TextureRectangle(osgDB::readImageFile(base + "/previous_active.png")));
    _rightTex.push_back(new osg::TextureRectangle(osgDB::readImageFile(base + "/next_inactive.png")));
    _rightTex.push_back(new osg::TextureRectangle(osgDB::readImageFile(base + "/next_active.png")));

    PicFlyer::defaultCoverImage = osgDB::readImageFile(base + "/DEFAULT_PAGE_IMAGE.tif");
    PicFlyer::defaultPageImage = osgDB::readImageFile(base + "/DEFAULT_PAGE_IMAGE.tif");

    if (!PicFlyer::defaultPageImage) {
      ROS_WARN_STREAM("Failed to load default page image");
    }
}

/**
 * Loads full scale resolution of an image
 * @param v: cursor point
 */
void PicFlyer::loadLarge(osg::Vec3 v)
{
    QString collection, id;
    int index;
    findImage(v, collection, index, id);

    _largeImageWrapper->removeChildren(0,_largeImageWrapper->getNumChildren());   
    _large.clear(); 
    
    // actually id is a number set to the absolute index
    index = id.toInt();

    ROS_INFO_STREAM("Loading image " << index);

    osg::ref_ptr<osg::Image> img = collectionImages->at(index)->getImage();  
    _large.push_back(new osg::TextureRectangle(img));

    double w= _large.at(0).get()->getImage()->s();
    double h= _large.at(0).get()->getImage()->t();

    largeCalc(w, h);

    _largeImage = new PlanarObject(   -w/2,-h/2,0,w/2,h/2,0,
                                            &_large,0,
                                            osg::Vec3(0,0,3));
                                      
    _largeImageWrapper->addChild(_largeImage);
  
}

/**
 * Loads large image from the page turner view
 * @param v: cursor point
 */
void PicFlyer::loadLargeFromTurner(turn_t side)
{
    QString collection, id;
    int page;

    if(side== TURN_LEFT)
    {
        collection= _openingImage1->_collection;
        id= _openingImage1->_id;
        page=0;
    }
    else
    {
        collection= _openingImage2->_collection;
        id= _openingImage2->_id;
        page=1;
    }

    int idInd= id.toInt();

    ROS_INFO_STREAM("Adding Large Image From Turner");  
    
    _largeImageWrapper->removeChildren(0,_largeImageWrapper->getNumChildren());   
    _large.clear(); 
    
    osg::ref_ptr<osg::Image> img = dispOpenings->at(idInd)->getPages()->at(page)->getImage();
    _large.push_back(new osg::TextureRectangle(img));

    double w= _large.at(0).get()->getImage()->s();
    double h= _large.at(0).get()->getImage()->t();

    largeCalc(w, h);


    // TODO: Eliminate after change
    _largeImage = new PlanarObject(   -w/2,-h/2,0,w/2,h/2,0,
                                            &_large,0,
                                            osg::Vec3(0,0,3));
    // _largeImage = new TexturedPlane( osg::Vec3(0,0,3), 1400, 2100,
    //                                  &_large, 0);

    ROS_INFO_STREAM("Adding to Graph");
    _largeImageWrapper->addChild(_largeImage);
  
}

/**
 * Loads the page turner for the first display
 * @param v: cursor point
 */
void PicFlyer::loadPageTurner(osg::Vec3 location)
{
    // Cannot use findImage because position is on intermediate nodes

    double closest;
    OSGObjectBase *closest_object = 0;

    osg::Vec2d sel = osg::Vec2d(location[0], location[1]);

    osg::Group *group = _imageWrapper;
    for (unsigned int i = 0; i < group->getNumChildren(); i++) {
        OSGObjectBase* p = dynamic_cast<OSGObjectBase*>(group->getChild(i));

        if (!p) {
            continue;
        }

        osg::Vec2d pos = p->getPos2D();
        osg::Vec2d v;
        v = sel - pos;
        double dist = v.normalize();

        if (!closest_object || dist < closest) {
            closest = dist;
            closest_object = p;
        }
    }

    if (closest_object && closest_object->getNumChildren() > 0) {
      PlanarObject *thumb = (PlanarObject *) closest_object->getChild(0);

      showPageTurner(thumb->_collection, thumb->_id);
    }
}

void PicFlyer::loadTurn(turn_t turn)
{
    int idNum = 0;

    //TODO: Turning animation
    if(turn==TURN_LEFT)
    {
        idNum=(_openingImage1->_id.toInt())-1;
    }

    else if (turn==TURN_RIGHT)
    {
        idNum=(_openingImage1->_id.toInt())+1;
    }

    
    if(idNum<0 || idNum>=_imageThumbs.size())
    {
        ROS_INFO_STREAM("Too much turning. #: "<<idNum);
        return;
    }
    // _openingWrapper->removeChildren(0, _openingWrapper->getNumChildren());
    showPageTurner(_openingImage1->_collection, QString::number(idNum));
}

/**
 * Loads the page turner view
 */
void PicFlyer::showPageTurner(QString collection, QString id)
{

    int idNum=id.toInt();

    _openingImage1=NULL;
    _openingImage2=NULL;

    ROS_INFO_STREAM("Loading Opening"<<" ("<<idNum<<" of "<<_imageThumbs.size()<<")");

    _openingWrapper->removeChildren(0, _openingWrapper->getNumChildren());
    _opening.clear(); 

    ROS_INFO_STREAM("Opening info: "<<dispOpenings->at(idNum)->getText().toStdString());

    Model::Opening* tmpOpening = dispOpenings->at(idNum);

    setTitle(QString::fromStdString(title->getText().createUTF8EncodedString()), dispOpenings->at(idNum)->getText(), idNum, dispOpenings->size());
    updateButtons(idNum, dispOpenings->size());

    if(tmpOpening->getLength()==1)
    {
        osg::ref_ptr<osg::Image> img = tmpOpening->getPageByIndex(0)->getImage();
        _opening.push_back(new osg::TextureRectangle(img));
        
        double w= _opening.at(0).get()->getImage()->s();
        double h= _opening.at(0).get()->getImage()->t();

        turnCalc(w, h);

        _openingImage1 = new PlanarObject(   -w/2,-h/2,0,w/2,h/2,0,
                                                &_opening,0,
                                                osg::Vec3(0,TURNER_OFFSET,3));

        // _openingImage1 = new TexturedPlane( osg::Vec3(0,0,3), 1400, 2100,
        //                                      &_opening, 0);

        _openingImage1->setCollection(collection);
        _openingImage1->setID(id);
        _openingImage2 = NULL;

        // if(_openingWrapper->getNumChildren()<1)
        _openingWrapper->addChild(_openingImage1);
        _openingWrapper->setHidden();
        _openingWrapper->setVisible();
    }

    else
    {
        osg::ref_ptr<osg::Image> img1 = tmpOpening->getPageByIndex(0)->getImage();
        _opening.push_back(new osg::TextureRectangle(img1));

        osg::ref_ptr<osg::Image> img2 = tmpOpening->getPageByIndex(1)->getImage();
        _opening.push_back(new osg::TextureRectangle(img2));

        double w1= _opening.at(0).get()->getImage()->s();
        double h1= _opening.at(0).get()->getImage()->t();

        turnCalc(w1, h1);

        _openingImage1 = new PlanarObject(   -w1/2,-h1/2,0,w1/2,h1/2,0,
                                                &_opening,0,
                                                osg::Vec3(-w1/2,TURNER_OFFSET,3));

        // _openingImage1 = new TexturedPlane( osg::Vec3(-700,0,3), 1400, 2100,
        //                                      &_opening, 0);

        _openingImage1->setCollection(collection);
        _openingImage1->setID(id);

        double w2= _opening.at(1).get()->getImage()->s();
        double h2= _opening.at(1).get()->getImage()->t();

        turnCalc(w2, h2);

        _openingImage2 = new PlanarObject(   -w2/2,-h2/2,0,w2/2,h2/2,0,
                                                &_opening,1,
                                                osg::Vec3(w2/2,TURNER_OFFSET,3));

        // _openingImage2 = new TexturedPlane( osg::Vec3(700,0,3), 1400, 2100,
        //                                      &_opening, 1);

        _openingImage2->setCollection(collection);
        _openingImage2->setID(id);

        // if(_openingWrapper->getNumChildren()<1)
        // {
            _openingWrapper->addChild(_openingImage1);
            _openingWrapper->addChild(_openingImage2);
        // }
        // else if(_openingWrapper->getNumChildren()<2)
        // {
        //      _openingWrapper->addChild(_openingImage2);
        // }


    }

    camz = 1160;
}

/**
 * Shows the opening browser
 * @param v: cursor point
 */
void PicFlyer::showPageCollection(QString c, QString ident, int start)
{
    QString collection=c, id=ident;
    int end;
    //TODO: I don't like that there isn't a seperate load method, but the nature of openings necessitates this probably.
    _imageWrapper->removeChildren(0, _imageWrapper->getNumChildren());
    _imageTextures.clear();
    _imageThumbs.clear();


    ROS_INFO_STREAM("PAGE COLLECTION: "<<collection.toStdString());
    ROS_INFO_STREAM("ID: "<<id.toStdString());

    Model::Manuscript* tmpMan= Model::CollectionStore::getManuscriptCollectionByID(collection)->getManuscriptByName(id);
    dispOpenings = tmpMan->getOpenings();

    //This loads all of the images. ORDERING IS IMPORTANT.
    //TODO: I don't particularly like this, but I don't know if there is a better way
    //Adjusts starting position
    int opSize = dispOpenings->size();
    numThumbnailCalc(start, end, opSize);
    int tStart= (start/NUM_THUMBNAILS)+1;
    int tTotal;
    if (opSize%NUM_THUMBNAILS==0)
        tTotal = opSize/NUM_THUMBNAILS;
    else
        tTotal = (opSize/NUM_THUMBNAILS)+1;

    setTitle(tmpMan->getName(), tmpMan->getPermission(), tStart, tTotal);
    updateButtons(tStart, tTotal);

    for(int i=start; i<end; i++)
    {
        for(int k=0; k<dispOpenings->at(i)->getLength(); k++)
        {
            osg::ref_ptr<osg::Image> tmpImg= dispOpenings->at(i)->getPages()->at(k)->getImage(QBool(true));
            if(!tmpImg)
            {
                tmpImg = defaultPageImage;
            }
            _imageTextures.push_back(new osg::TextureRectangle(tmpImg));
        }
    }

    ROS_INFO_STREAM("Loading Planar Objects ");
    int n = 0;
    int d = start;

    for(int py = 0; py < DISPLAY_HEIGHT; py++)
    {
        for (int px = 0; px < DISPLAY_WIDTH; px++)
        {

            OSGObjectBase* tmpMiddle = new OSGObjectBase();

            if(d < dispOpenings->size())
            {
                osg::Vec3 coord= osg::Vec3((px*THUMB_GAP_X) + THUMB_OFFSET_X, (py*THUMB_GAP_Y) + THUMB_OFFSET_Y, 0);
                tmpMiddle->setPosition(coord);
                if(dispOpenings->at(d)->getLength()==1)
                {    
                    double w= _imageTextures.at(n).get()->getImage()->s();
                    double h= _imageTextures.at(n).get()->getImage()->t();

                    thumbnailCalc(w, h);



                    PlanarObject* img;

                    img = new PlanarObject( -w/2, -h/2, 0, w/2, h/2, 0,
                                            &_imageTextures, n, osg::Vec3(0,0,0) );
                    
                    // TexturedPlane* img = new TexturedPlane( osg::Vec3(px*320+150,-(py*400+300),0), 240, 180,
                    //                                         &_imageTextures, n);
                    n++; 
                    // img->setText(collection);
                    img->setID(QString::number(d));
                    img->setCollection(collection);
                    img->setText(   labelFormatter(dispOpenings->at(d)->getText(), LABEL_LENGTH), THUMB_TEXT_SIZE, 
                                    osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec3(0, THUMB_TEXT_OFFSET,0));
                    _imageThumbs.push_back(img);
                    tmpMiddle->addChild(img);
                }
                else 
                {
                    double w= _imageTextures.at(n).get()->getImage()->s()*2;
                    double h= _imageTextures.at(n).get()->getImage()->t();

                    thumbnailCalc(w, h);

                    osg::Vec3 coord1= osg::Vec3(-w/4, 0, 0);
                    osg::Vec3 coord2= osg::Vec3(w/4, 0, 0);

                    PlanarObject* img1;
                    PlanarObject* img2;

                    img1 = new PlanarObject( -w/4, -h/2, 0, w/4, h/2, 0,
                                            &_imageTextures, n, coord1 );

                    // TexturedPlane* img1 = new TexturedPlane( osg::Vec3((px*320+130)-60,-(py*400+300),0), 120, 180,
                    //                                         &_imageTextures, n);
                    img1->setID(QString::number(d));
                    img1->setCollection(collection);
                    img1->setText(  labelFormatter(dispOpenings->at(d)->getText(), LABEL_LENGTH), THUMB_TEXT_SIZE, 
                                    osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec3(w/4, THUMB_TEXT_OFFSET,0));
                    _imageThumbs.push_back(img1);
                    tmpMiddle->addChild(img1);
                    n++; 

                    img2 = new PlanarObject( -w/4, -h/2, 0, w/4, h/2, 0,
                                            &_imageTextures, n, coord2 );

                    // TexturedPlane* img2 = new TexturedPlane( osg::Vec3((px*320+130)+60,-(py*400+300),0), 120, 180,
                                                            // &_imageTextures, n);

                    img2->setID(QString::number(d));
                    img2->setCollection(collection);
                    _imageThumbs.push_back(img2);
                    tmpMiddle->addChild(img2);
                    n++;
                }

                _imageWrapper->addChild(tmpMiddle);
                 
                d++;
                std::cerr<< n <<".";

            }
        }
    }


    ROS_INFO_STREAM("Adding "<<_imageThumbs.size()<<"images to Graph");
    camz = 1160;
}


/**
 * Figures out what the closest image to the cursor is
 * @param pt: cursor point
 * @param col: reference that has the closest collection title on function completion
 * @param index: reference that holds index of closest image on function completion
 * @param id: reference that holds id of closest image on function completetion
 */
void PicFlyer::findImage(osg::Vec3 pt, QString& col, int& index, QString& id)
{
    QString closestID, closestCol;
    int closestIndex = -1;
    double dist, small = INT_MAX;
    osg::Vec3d v;
    osg::Vec3d p = osg::Vec3d(pt[0],pt[1],pt[2]);


    for(int i = 0;i<_imageThumbs.size();i++){
        osg::Vec3d ip = _imageThumbs[i]->getPosition();
        osg::Vec3d ipv = osg::Vec3d(ip[0],ip[1],ip[2]);

        v = p - ipv;
        dist = v.normalize();

        if(dist<small){
            small = dist;
            closestID = _imageThumbs[i]->_id;
            closestCol = _imageThumbs[i]->_collection;
            closestIndex = i;
        }
    }

    ROS_INFO_STREAM("\tFound thumb, id: "<<closestID.toStdString()<<" Col: "<<closestCol.toStdString()<<" ind: "<<closestIndex);

    col = closestCol;
    index = closestIndex;
    id= closestID;
}

/**
 * Finds the closest index to the pointer
 * @param pt: the cursor point
 */

int PicFlyer::findNearestIndex(osg::Vec3 pt)
{
    ROS_INFO_STREAM("Finding Cloest Index");
    int closestIndex = -1;

    double dist, small= INT_MAX;

    osg::Vec3d v;
    osg::Vec3d p = osg::Vec3d(pt[0], pt[1], pt[2]);

    for(int i=0; i<_indexThumbs.size(); i++)
    {
        osg::Vec3 ip = _indexThumbs[i]->getPosition();
        osg::Vec3d ipv = osg::Vec3d(ip[0],ip[1],ip[2]);
        v = p - ipv;
        dist = v.normalize();

        if(dist<small)
        {
            small = dist;
            closestIndex = i;
        }

    }

    return closestIndex;

}

/**
 * Loads the selection index pictures
 */
void PicFlyer::loadIndex()
{
    QStringList unorderedCollectionIDs= Model::CollectionStore::getPictureCollectionIDs();
    QStringList manuscriptIDs= Model::CollectionStore::getManuscriptIDs();

    //unordered collections
    for(int i = 0; i<unorderedCollectionIDs.size(); i++)
    {
        ROS_INFO_STREAM("Making "<<unorderedCollectionIDs.at(i).toStdString());
        Model::PictureCollection* temp= Model::CollectionStore::getPictureCollectionByID(unorderedCollectionIDs.at(i));
        // _unorderedCollections->insert(unorderedCollectionIDs.at(i), temp);
        _indexNames.push_back(unorderedCollectionIDs.at(i)); 
        _indexLabels.push_back(PicFlyer::labelFormatter(temp->getName(), LABEL_LENGTH));
        osg::ref_ptr<osg::Image> tempTex= temp->getFirst(QBool(true));
        _indexTextures.push_back(new osg::TextureRectangle(tempTex));
    }

    //manuscripts
    for(int i = 0; i<manuscriptIDs.size(); i++)
    {
        ROS_INFO_STREAM("Making "<<manuscriptIDs.at(i).toStdString());
        Model::ManuscriptCollection* temp= Model::CollectionStore::getManuscriptCollectionByID(manuscriptIDs.at(i));
        // _manuscriptCollections->insert(manuscriptIDs.at(i), temp);
        _indexNames.push_back(manuscriptIDs.at(i));
        _indexLabels.push_back(PicFlyer::labelFormatter(temp->getTitle(),LABEL_LENGTH));
        osg::ref_ptr<osg::Image> tempTex= temp->genCover();
        ROS_INFO_STREAM("Image File: "<<tempTex->getFileName());
        if(tempTex == NULL)
            tempTex= defaultCoverImage;
        _indexTextures.push_back(new osg::TextureRectangle(tempTex));
    }

}

/**
 * Loads a collection of images
 * @param collection: the collection to be loaded
 */
void PicFlyer::loadCollection(QString collection, int start)
{
    int end;
    _imageWrapper->removeChildren(0, _imageWrapper->getNumChildren());
    _imageTextures.clear(); 
    _imageLabels.clear();
    _imageThumbs.clear();

    Model::PictureCollection* tempCol = Model::CollectionStore::getPictureCollectionByID(collection);
    collectionImages = tempCol->getImages();

    int colSize = collectionImages->size();
    numThumbnailCalc(start, end, colSize);
    int tStart= (start/NUM_THUMBNAILS)+1;
    int tTotal;
    if (colSize%NUM_THUMBNAILS==0)
        tTotal = colSize/NUM_THUMBNAILS;
    else
        tTotal = (colSize/NUM_THUMBNAILS)+1;


    // cerr << "start " << start << endl;
    // cerr << "col size " << colSize << ", thumbs per page  " << NUM_THUMBNAILS << endl;
    // cerr << "title page pos " << tStart << " of  " << tTotal << endl;

    setTitle(tempCol->getName(), tempCol->getPermission(), tStart, tTotal);
    updateButtons(tStart, tTotal);

    for(int i=start; i<end; i++)
    {
        osg::ref_ptr<osg::Image> tmpImg= collectionImages->at(i)->getImage(QBool(true));
        _imageTextures.push_back(new osg::TextureRectangle(tmpImg));
        _imageLabels.push_back(labelFormatter(collectionImages->at(i)->getName(), LABEL_LENGTH));
    }
}

/**
 * Shows the currently selected collection on screen
 * @param collection: the collection to be displayed
 */
void PicFlyer::showCollection(QString collection, int start)
{    
    // _imageWrapper->removeChildren(0,_imageWrapper->getNumChildren());

    ROS_INFO_STREAM("Showing Collection "<< collection.toStdString());
    loadCollection(collection, start);

    int n=0;
    for(int py = 0; py < DISPLAY_HEIGHT; py++)
    {
        for (int px = 0; px < DISPLAY_WIDTH; px++)
        {
            if(n < _imageTextures.size())
            {

                double w= _imageTextures.at(n).get()->getImage()->s();
                double h= _imageTextures.at(n).get()->getImage()->t();

                thumbnailCalc(w, h);

                osg::Vec3 coord= osg::Vec3((px*THUMB_GAP_X) + THUMB_OFFSET_X, (py*THUMB_GAP_Y) + THUMB_OFFSET_Y, 0);

                PlanarObject* img;

                img = new PlanarObject( -w/2, -h/2, 0, w/2, h/2, 0,
                                        &_imageTextures, n, coord );

		// Set thumb id to index of image in the whole collection
                img->setID(QString::number(start + n));
                img->setCollection(collection);
                img->setText(_imageLabels[n], THUMB_TEXT_SIZE, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec3(0, THUMB_TEXT_OFFSET,0));
                _imageThumbs.push_back(img); 
                n++;
            }
        }
    }

    for (int i = 0; i < _imageThumbs.size(); ++i){
        _imageWrapper->addChild(_imageThumbs[i]);
    }

    //_camServo.startServo(_camera,Vec3(0,0,1160),1);
    //camz = 1160;
}

/**
 * Loads a collection of manuscripts
 * @param collection: the collection of manuscripts to be loaded
 */
// TODO must delete result
QStringList* PicFlyer::loadManuscriptCollection(QString collection, int start)
{
    _imageWrapper->removeChildren(0, _imageWrapper->getNumChildren());
    _imageTextures.clear(); 
    _imageThumbs.clear();
    _imageLabels.clear();
    int end;

    QStringList* ids;

    Model::ManuscriptCollection* tmpManCol= Model::CollectionStore::getManuscriptCollectionByID(collection);
    ids= tmpManCol->getManuscriptNames();

    int manSize = tmpManCol->getManuscriptNames()->size();
    numThumbnailCalc(start, end, manSize);
    int tStart= (start/NUM_THUMBNAILS)+1;
    int tTotal;
    if (manSize%NUM_THUMBNAILS==0)
        tTotal = manSize/NUM_THUMBNAILS;
    else
        tTotal = (manSize/NUM_THUMBNAILS)+1;

    setTitle(tmpManCol->getTitle(), QString(""), tStart, tTotal);
    updateButtons(tStart, tTotal);

    for (int i = start; i< end; i++)
    {
        Model::Manuscript* tempMan= tmpManCol->getManuscriptByName(ids->at(i));
        osg::ref_ptr<osg::Image> tmpImg= tempMan->getCover(QBool(true));
        if (tmpImg==NULL)
            tmpImg = defaultCoverImage;
        _imageTextures.push_back(new osg::TextureRectangle(tmpImg));
        _imageLabels.push_back(labelFormatter(tempMan->getCommonName(), LABEL_LENGTH));
    }

    return ids;
}

/**
 * Shows the currently selected manuscript collection on screen
 * @param collection: the manuscript collection to be displayed
 */
void PicFlyer::showManuscriptCollection(QString collection, int start)
{
    // _imageWrapper->removeChildren(0,_imageWrapper->getNumChildren());

    QStringList* ids;

    ROS_INFO_STREAM("Showing Manuscript Collection "<<collection.toStdString() );

    ids = loadManuscriptCollection(collection, start);

    int n = 0;
    for(int py = 0; py < DISPLAY_HEIGHT; py++)
    {
        for (int px = 0; px < DISPLAY_WIDTH; px++)
        {
            if(n < _imageTextures.size())
            {

                double w= _imageTextures.at(n).get()->getImage()->s();
                double h= _imageTextures.at(n).get()->getImage()->t();

                thumbnailCalc(w, h);

                osg::Vec3 coord= osg::Vec3((px*THUMB_GAP_X) + THUMB_OFFSET_X, (py*THUMB_GAP_Y) + THUMB_OFFSET_Y, 0);

                PlanarObject* img;

                img = new PlanarObject( -w/2, -h/2, 0, w/2, h/2, 0,
                                        &_imageTextures, n, coord );

                img->setID(ids->at(n + start));
                img->setCollection(collection);
                img->setText(_imageLabels[n], THUMB_TEXT_SIZE, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f), osg::Vec3(0, THUMB_TEXT_OFFSET,0));
                _imageThumbs.push_back(img); 
                n++;
            }
        }
    }

    for (int i = 0; i < _imageThumbs.size(); ++i){
        _imageWrapper->addChild(_imageThumbs[i]);
    }

    // _camServo.startServo(_camera,Vec3(0,0,1160),1);
    camz = 1160;
}

/**
 * Adds view widget. Not sure on details.
 */
osgQt::GLWidget* PicFlyer::addViewWidget( osg::Camera* camera, osg::Node* scene )
{
    setCamera( camera );
    
    setSceneData( scene );

    //Idk if this is going to work
    setDone(false);

    addEventHandler( new osgViewer::StatsHandler );
    //setCameraManipulator( new osgGA::TrackballManipulator );
    
    PicFlyerKeyboardHandler* myFirstEventHandler =
        new PicFlyerKeyboardHandler();
    myFirstEventHandler->setup(this);
    addEventHandler(myFirstEventHandler); 
    setThreadingModel(SingleThreaded);
    osgQt::GraphicsWindowQt* gw = 
        dynamic_cast<osgQt::GraphicsWindowQt*>( camera->getGraphicsContext() );
    
    return gw ? gw->getGLWidget() : NULL;
}

/**
 * Camera generation.
 */
osg::Camera* PicFlyer::createCamera( int x, int y, int w, int h, osgQt::GLWidget *QTObject, const std::string& name, bool windowDecoration)
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

    _camera->setViewMatrixAsLookAt( osg::Vec3d( 0,0,camz ), // eye
                                    osg::Vec3d( 0,0,0 ),  // look
                                    osg::Vec3d( 0,1,0 )); // up

    _camera->setClearColor( osg::Vec4(0.1, 0.1, 0.1, 0.0) );
    _camera->setViewport( new osg::Viewport(0, 0, w, h) );
    _camera->setProjectionMatrixAsPerspective(90.0f, static_cast<double>(w)/static_cast<double>(h), 0.1f, 100000.0f );   
    
    _cameraOrbit = osg::Vec3(0,0,1);
    _cameraStart = osg::Vec3(0,0,camz);

    return _camera;
}

/**
 * Formats strings to fit neatly under thumbnails by inserting newlines.
 * @param toForm: The QString to be formatted.
 * @return QString properly formatted for display
 */
QString PicFlyer::labelFormatter(QString toForm, int length)
{
    QString in = QString(toForm);
    int index=length;
    int cursize= in.size();
    int last= -1;


    while(index < cursize)
    {
        QString temp= in.left(index);
        int t= temp.lastIndexOf(" ");
        if( t > last)
        {
            in.replace(t, 1, '\n');
            index = t+1;
            // cursize++;
            last = t;
            // processed+=t;
        }
        // else
            // processed+=PicFlyer::LABEL_LENGTH;

        
        index+=length;

    }

    return in;
}

void PicFlyer::setTitle(QString t, QString st, int pO, int pT)
{
    title->setText(labelFormatter(t, 12).toStdString(), osgText::String::ENCODING_UTF8);
    subtitle->setText(labelFormatter(st, 18).toStdString(), osgText::String::ENCODING_UTF8);

    QString page = QString("Page\n")+QString::number(pO)+QString("/")+QString::number(pT);

    pageInd->setText(page.toStdString(), osgText::String::ENCODING_UTF8);
}

void PicFlyer::thumbnailCalc(double& x, double& y)
{
    double d, max;
    if(x > y)
        max=x;
    else
        max=y;

    d= max/THUMBNAIL_SIDE;

    x/=d;
    y/=d;
}

void PicFlyer::largeCalc(double& x, double& y)
{
    double d, max;
    if(x > y)
        max=x;
    else
        max=y;

    d= max/LARGE_SIDE;

    x/=d;
    y/=d;
}

void PicFlyer::turnCalc(double& x, double& y)
{
    double d, max;
    if(x > y)
        max=x;
    else
        max=y;

    d= max/TURNER_SIDE;

    x/=d;
    y/=d;
}

void PicFlyer::numThumbnailCalc(int& start, int& end, int size)
{
    //Adjusts starting position
    if(start>size)
    {
        ROS_INFO_STREAM("Start value is too large. Setting to max start value");
        start=(size/NUM_THUMBNAILS)*NUM_THUMBNAILS; //Integer division
    }

    else if(start<0)
    {
        ROS_INFO_STREAM("Start value is too small. Setting to 0");
        start=0;
    }

    if(start+NUM_THUMBNAILS>size)
        end=size;
    else
        end=start+NUM_THUMBNAILS;

}

void PicFlyer::updateButtons(int tStart, int tTotal)
{
  button_call.clear();  
  button_call.push_back(tStart);
  button_call.push_back(tTotal);
  // TODO: Issues with setHidden logic and scaling exposed by updateCursor

  if (tStart < tTotal) {
    _page_arrows["right"]->setVisible();
    _page_arrows["right"]->setScaleAll(1.0);

  } else {
    _page_arrows["right"]->setHidden();
    _page_arrows["right"]->setScaleAll(0.0);
  }

  if (tStart > 1) {
    _page_arrows["left"]->setVisible();
    _page_arrows["left"]->setScaleAll(1.0);
  }  else {
    _page_arrows["left"]->setHidden();
    _page_arrows["left"]->setScaleAll(0.0);
  }
}

// Keyboard Handler ////////////////////////////
bool PicFlyerKeyboardHandler::handle(const osgGA::GUIEventAdapter& ea,osgGA::GUIActionAdapter& aa)
{
    switch(ea.getEventType())
    {
        case(osgGA::GUIEventAdapter::KEYDOWN):
        {
            switch(ea.getKey())
            {
                //Quit
                case 'q':
		  //this->appPtr->sendSelfTerminate();
                    break;
                
                // Camera Zoom
                case 'n':
                    this->appPtr->_camServo.startServo(
                                            this->appPtr->_camera,
                                            osg::Vec3(0,0,camz-=10),
                                            .2);
                    std::cout<<camz<<std::endl;
                    break;
                case 'm':
                    this->appPtr->_camServo.startServo(
                                            this->appPtr->_camera,
                                            osg::Vec3(0,0,camz+=10),
                                            .2);
                    std::cout<<camz<<std::endl;
                    break;

                //Manual collection selection
                case 'x':
                    // for(int i = 0; i< this->appPtr->_indexThumbs.size();i++)
                    //     this->appPtr->_indexThumbs[i]->setHidden();
                    this->appPtr->_indexWrapper->setHidden();
                    this->appPtr->showCollection(this->appPtr->_indexNames[0], 0);
                    break;
                case 'c':
                    // for(int i = 0; i< this->appPtr->_indexThumbs.size();i++)
                        // this->appPtr->_indexThumbs[i]->setHidden();
                    this->appPtr->_indexWrapper->setHidden();
                    this->appPtr->showCollection(this->appPtr->_indexNames[1], 0);
                    break;
                case 'v':
                    // for(int i = 0; i< this->appPtr->_indexThumbs.size();i++)
                    //     this->appPtr->_indexThumbs[i]->setHidden();
                    this->appPtr->_indexWrapper->setHidden();
                    this->appPtr->showManuscriptCollection(this->appPtr->_indexNames[2], 0);
                    break;

                //Manually step update
                case 'i':
                    this->appPtr->updateEnvironment();
                    break;

                //Load large image
                case 'd':
                    this->appPtr->_imageWrapper->setHidden();
                    this->appPtr->loadLarge(osg::Vec3(0,0,0));
                    break;

                //Load Page Turner
                case 'g':
                    for(int i = 0; i< this->appPtr->_imageThumbs.size();i++)
                        this->appPtr->_imageThumbs[i]->setHidden();
                    this->appPtr->loadPageTurner(osg::Vec3(0,0,0));
                    break;

                //Load Image from page turner
                case 'h':
                    this->appPtr->_openingImage1->setHidden();
                    if(this->appPtr->_openingImage2)
                        this->appPtr->_openingImage2->setHidden();
                    this->appPtr->loadLargeFromTurner(TURN_LEFT);
                    break;
                case 'j':
                    this->appPtr->_openingImage1->setHidden();
                    if(this->appPtr->_openingImage2)
                        this->appPtr->_openingImage2->setHidden();
                    this->appPtr->loadLargeFromTurner(TURN_RIGHT);
                    break;

                //Turn Pages
                case 'o':
                    this->appPtr->loadTurn(TURN_LEFT);
                    break;
                case 'p':
                    this->appPtr->loadTurn(TURN_RIGHT);
                    break;
            } 
        }
        default:
            return false;
    }
}

void PicFlyerKeyboardHandler::accept(osgGA::GUIEventHandlerVisitor& v)
{ 
    v.visit(*this); 
}

void PicFlyerKeyboardHandler::setup(PicFlyer* appPt)
{
    this->appPtr = appPt;
}

void PicFlyer::loadToolTips(QDir tooltip_dir)
{
    QStringList tooltipFiles = tooltip_dir.entryList(QDir::Files | QDir::Readable, QDir::Name);
    
    QStringList tooltipPaths;

    for (int i=0; i<tooltipFiles.count(); i++){
        tooltipPaths << tooltip_dir.absoluteFilePath(tooltipFiles[i]);
    }

    ROS_INFO_STREAM(">>> Loading Tooltip Images... ");
      
    for (int i = 0; i < tooltipPaths.size(); ++i){
        QPixmap* pix = new QPixmap(tooltipPaths.at(i)); 
        toolTipImages.push_back(pix);
    }
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "pic_flyer_app");

  ROS_WARN_STREAM("PicFlyerApp: Starting Up...");

  QApplication application(argc, argv);
  application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

  ros::NodeHandle node_handle;

  PicFlyer pic_flyer_app("PicFlyerApp", node_handle, 20,"pic_flyer");

  pic_flyer_app.build();
  pic_flyer_app.start();

  ROS_WARN_STREAM("PicFlyerApp: App Running");

  pic_flyer_app.ready();
  application.exec();

  pic_flyer_app.stop();

  ROS_WARN_STREAM("PicFlyerApp: App Finished");
  return 0;
}

