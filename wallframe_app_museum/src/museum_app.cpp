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
 * Author: Preetham Chalasani, Johns Hopkins University
 */


#include <museum_app.h>
#include <QTimer>

using namespace std;
namespace wallframe{

ExampleApp::ExampleApp(std::string app_name, ros::NodeHandle nh, int event_deque_size,std::string app_id) :
    wallframe::WallframeAppBaseQt(app_name, nh, event_deque_size,app_id)
{
    widget = new MuseumApp(this);

    widget->resize(width_,height_ * height_perc_);

//    QTimer* timer = new QTimer(this);
//    connect( timer, SIGNAL( timeout() ), this, SLOT( update() ) );
//    timer->start(1000.0 / 60.0);

   user_state_subscriber = node_.subscribe("/wallframe/users/state", 1000, &ExampleApp::StateCallback, this);
   user_event_subscriber = node_.subscribe("/wallframe/users/events", 1000, &ExampleApp::EventCallback, this);


}

ExampleApp::~ExampleApp()
{
    delete widget;
}

bool ExampleApp::build(){
    std::string asset_path;
    if (!node_.getParam("/wallframe/apps/museum_app/paths/assets", asset_path)){
        ROS_ERROR("Wallframe%s: No asset path found on parameter server (namespace: %s)",
                  name_.c_str(), node_.getNamespace().c_str());
        return false;
    }
    asset_path_ = QString(asset_path.c_str());

    return true;
}

void ExampleApp::EventCallback(const wallframe_msgs::WallframeUserEventConstPtr &user_event)
{
    current_user_event = *user_event;
    ROS_WARN_STREAM(current_user_event.message);
}

void ExampleApp::StateCallback(const wallframe_msgs::WallframeUserArrayConstPtr &user_packet)
{
    current_user_packet = *user_packet;
    user_data = this->current_user_packet.users;
}


bool ExampleApp::start()
{
    return true;
}

bool ExampleApp::stop()
{
    return true;
}

bool ExampleApp::pause()
{
    this->hide();
    return true;
}

bool ExampleApp::resume()
{
    this->show();
    return true;
}


} // end namepsace modulair

using namespace wallframe;

int main(int argc, char* argv[]){


    // ros::init must be called before instantiating any apps
    ros::init(argc,argv, "museum_app");

    ros::NodeHandle node_;

    QApplication application(argc,argv);
    // This line will quit the application once any window is closed.

    application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

    // The app_id should be same as the name in the menu.cfg
    // TODO we should have a menu.cfg parser for C++ for now these are hardcoded
    ExampleApp example_app("Virtual Museum",node_,20,"museum");

    example_app.build();
    example_app.widget->show();

    ROS_WARN_STREAM("Virtual Museum: App Running");
    example_app.ready();
    application.exec();
    // Running
    example_app.stop();
    ROS_WARN_STREAM("Virtual Museum: App Finished");
    return 0;
}
