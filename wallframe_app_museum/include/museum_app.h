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
 * Author: Preetham Chalasani, pchalas1@hotmail.com, Johns Hopkins University
 */

#ifndef museum_app_h
#define museum_app_h

#include <ros/node_handle.h>

#include <wallframe_core/wallframe_app_base_qt.h>

#include <MuseumApp.h>

//#include <modulair_osg_tools/vector_conversions.h>
#include <iostream>
#include <osg/Vec3>

namespace wallframe{

class ExampleApp : public wallframe::WallframeAppBaseQt{

    Q_OBJECT
public:
    ExampleApp(std::string app_name, ros::NodeHandle nh, int event_deque_size, std::string app_id);
    ~ExampleApp();
    bool build();
    bool start();
    bool stop();
    bool pause();
    bool resume();

    void EventCallback(const wallframe_msgs::WallframeUserEventConstPtr &user_event);
    void StateCallback(const wallframe_msgs::WallframeUserArrayConstPtr &user_packet);

    MuseumApp* widget;

protected:


private:
    ros::Subscriber user_event_subscriber;
    ros::Subscriber user_state_subscriber;

    wallframe_msgs::WallframeUserEvent current_user_event;
    wallframe_msgs::WallframeUserArray current_user_packet;

    std::vector<wallframe_msgs::WallframeUser> user_data;

    std::string previousEvent;
};
}

#endif

