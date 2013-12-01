/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2013, Johns Hopkins University
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the Johns Hopkins University nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/*
 * Author: Kelleher Guerin, futureneer@gmail.com, Johns Hopkins University
 */


#include <modulair_app_screen_saver/screen_saver_app.h>
#include <modulair_app_screen_saver/glWidget.h>
#include <modulair_app_screen_saver/mouse/manymouse.h>
#include <modulair_osg_tools/vector_conversions.h>

#include <GL/glut.h>
#include <GL/glu.h>

#include <iostream>
#include <osg/Vec3>

using namespace std;
namespace modulair{

ExampleApp::ExampleApp(std::string app_name, ros::NodeHandle nh, int event_deque_size) :wallframe::WallframeAppBaseQt(app_name, nh, event_deque_size){

    cout<<"creating the screen saver app\n";
    widget = new GLWidget(this);


    connect( &_timer, SIGNAL(timeout()), this, SLOT(update()));
    connect( &_dataTimer, SIGNAL(timeout()), widget, SLOT(updateGL()));
    connect( &_dataTimer1, SIGNAL(timeout()), this, SLOT(updateUsers()));

//    //       connect( &_dataTimer, SIGNAL(timeout()), widget, SLOT(update()));
    _timer.start( 50 );
    _dataTimer.start(60);
    _dataTimer1.start(50);



////    mouseThread = new MouseProvider(this);
////    mouseThread->start();

////    connect(mouseThread,SIGNAL(mouseMoveEvent(int,int,int)),this,SLOT(mouseMoved(int,int,int)));
////    connect(mouseThread,SIGNAL(mouseClickEvent(int,int, int ,bool)),this,SLOT(mouseClicked(int,int, int ,bool)));

    for(int j=0;j<NUM_USERS;j++){
        activeUsers[j]=false;
    }
    for(int j=0;j<NUM_USERS;j++){
        prev_activeUsers[j]=false;
    }
}

bool ExampleApp::build(){
    std::string asset_path;
    if (!node_.getParam("/modulair/apps/screen_saver_app/paths/assets", asset_path)){
        ROS_ERROR("Modulair%s: No asset path found on parameter server (namespace: %s)",
                  name_.c_str(), node_.getNamespace().c_str());
        return false;
    }
    asset_path_ = QString(asset_path.c_str());

//    std::string height;
//    if (!node_.getParam("/wallframe/core/params/height", height)){
//        ROS_ERROR("Modulair%s: No height found on parameter  server (namespace: %s)",
//                  name_.c_str(), node_.getNamespace().c_str());
//        return false;
//    }

//    std::string width;
//    if (!node_.getParam("/wallframe/core/params/width", width)){
//        ROS_ERROR("Modulair%s: No width on parameter server (namespace: %s)",
//                  name_.c_str(), node_.getNamespace().c_str());
//        return false;
//    }

//    cout<< "The width of the app is "<<this->width_;
//    this->widget->resize(width_, height_);

    // set the widget height and width

    return true;
}

bool ExampleApp::start(){return true;}

bool ExampleApp::stop(){return true;}

bool ExampleApp::pause(){return true;}

bool ExampleApp::resume(){return true;}


void ExampleApp::updateUsers(){
    numActiveUsers=0;

    for(int j=0;j<NUM_USERS;j++){
        activeUsers[j]=false;
    }

    wallframe::AppUserMap::iterator uit;
    for(uit = users_.begin();uit!=users_.end();uit++){
        int id = uit->first;
        numActiveUsers++;
        wallframe::AppUser user = uit->second;
        activeUsers[id]=true;
//        cout<<"Active user id"<<id;
        //   ROS_WARN_STREAM( activeUsers[id] <<"aftersettrue");
        if(prev_activeUsers[id]==false){
            ROS_WARN_STREAM("New user found! associating particle set #"<<id);
            widget->createParticleSystemForUser(id);
        }
        prev_activeUsers[id]=true;

        // now update the particle system position for this user
        // get the 2D position of this user in the space

        // this is actually left mappings are reversed in the original map
        osg::Vec3 joint_vec = eigToOsg3(user.jtPosById(user.jtIdByName("right_hand")));

        int xmin = -1200;
        int xmax = 1200;
        int ymin = -600;
        int ymax = 600;
        int xtotal = abs(xmin) + abs(xmax);
        int ytotal = abs(ymin) + abs(ymax);

        int width = 5760;
        int height = 3197;

        int xcenter = width/2;
        int ycenter = height/2;
        int x = (int)(xcenter + (float)(width/xtotal)*joint_vec[0]);
        int y = (int)(ycenter + (float)(height/ytotal)*joint_vec[1]);

        mouseMoved(x,y,id,true); // left hand

        // right hand
        osg::Vec3 joint_vec_right = eigToOsg3(user.jtPosById(user.jtIdByName("left_hand")));


        int xright = (int)(xcenter + (float)(width/xtotal)*joint_vec_right[0]);
        int yright = (int)(ycenter + (float)(height/ytotal)*joint_vec_right[1]);
        mouseMoved(xright,yright,id,false); // right hand

    }
    // if there are no user the default mode is on
    if(numActiveUsers > 0)
        widget->defaultMode = false;
    else
        widget->defaultMode = true;

//    cout<<"The number of active users is " <<numActiveUsers;

    // Release the  paarticle system for the users who leave
    for(int j=0;j<NUM_USERS;j++){
        //   ROS_WARN_STREAM( "user j is "<<activeUsers[j] );
        if((activeUsers[j]==false) && prev_activeUsers[j]){
            //   ROS_WARN_STREAM( activeUsers[j] <<"checkinguserj "<<j);
            prev_activeUsers[j]=false;
            ROS_WARN_STREAM("User Left deleting the particle system for user #"<<j);
            widget->destroyParticleSystemForUser(j);
            // TODO if the number of users in zero then go to the default mode

        }
    }


}

void ExampleApp::mouseMoved(int x,int y,int id,bool left){
    widget->splashParticleSystem(x,y,id,left);

}
void ExampleApp::mouseClicked(int x,int y,int id,bool pressed){

    if(pressed){
        widget->toggleMode();
    }

}
ExampleApp::~ExampleApp(){

    delete widget;

}

} // end namepsace modulair

using namespace modulair;

int main(int argc, char* argv[]){


    // ros::init must be called before instantiating any apps
    ros::init(argc,argv, "screen_saver");

    // intializing glut
//    glutInit(&argc, argv);


    ROS_WARN_STREAM("Screen Saver: Starting Up...");
    ros::NodeHandle node_handle;
    QApplication application(argc,argv);
    // This line will quit the application once any window is closed.
        application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
    ExampleApp example_app("Screen Saver",node_handle,20);

    ROS_WARN_STREAM("Screen Saver: created ...");

    int width = 5760;
    int height = 3197;
//    if(example_app.widget){
//        example_app.widget->setFixedSize(width,height);
//    }
//    example_app.resize(width,height);
    example_app.widget->resize(width,height);
    //  example_app.widget->setWindowState(Qt::WindowFullScreen);
//        example_app.widget->resize(Qt::WindowFullScreen);

    example_app.build();
    example_app.widget->show();

    ROS_WARN_STREAM("Screen Saver: App Running");
    application.exec();
    // Running
    example_app.stop();
    ROS_WARN_STREAM("Screen Saver: App Finished");
    return 0;
}
