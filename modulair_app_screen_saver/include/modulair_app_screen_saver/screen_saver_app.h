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

#ifndef cpp_example_app_h
#define cpp_example_app_h 

#include <wallframe_core/wallframe_app_base_qt.h>

#include <modulair_app_screen_saver/glWidget.h>

#include  <modulair_app_screen_saver/MouseProvider.h>

#include <modulair_osg_tools/vector_conversions.h>

namespace modulair{

    class ExampleApp : public wallframe::WallframeAppBaseQt{

        Q_OBJECT
  public:
    ExampleApp(std::string app_name, ros::NodeHandle nh, int event_deque_size, std::string app_id);
    ~ExampleApp();
    virtual bool build();
    virtual bool start();
    //virtual bool stop();
    virtual bool pause();
    virtual bool resume();
//    void updateUsers();
    GLWidget* widget;
    MouseProvider* mouseThread;


    // USERS
    int numActiveUsers;
    int images_per_user_;

    bool activeUsers[12];
    int joint_increments[12];
    bool prev_activeUsers[12];

    protected:
    QTimer _timer;
    QTimer _dataTimer;
    QTimer _dataTimer1;

    public Q_SLOTS:
//    void mouseMoved(Mouse* mouse);
//    void mouseClicked(Mouse* mouse,bool pressed);

    // left represents if the mouse is moved for the left or the right hand
    void mouseMoved(int x,int y,int id,bool left);
    void mouseClicked(int x,int y,int id,bool pressed);

    void updateUsers();

    };
}

#endif

