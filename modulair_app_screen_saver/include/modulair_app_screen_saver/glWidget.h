/****************************************************************************
**
** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/
#ifndef glWidget_h
#define glWidget_h

#include <QtOpenGL>

#include <GL/gl.h>
#include <GL/glu.h>

#include <time.h>
#include <map>
#include <iostream>
#define MAX_PARTICLES 2000
//#define MAX_PARTICLES 20
#define MAX_PARTICLE_AGE 150
#define MAX_BOUNCE_COUNT 5

#define NUM_USERS 5


using namespace std;
// TODO wrap this into particle system
typedef struct
{
    float lifetime;                       // total lifetime of the particle
    float decay;                          // decay speed of the particle
    float r,g,b;                          // color values of the particle
    float xpos,ypos,zpos;                 // position of the particle
    float xspeed,yspeed,zspeed;           // speed of the particle
    bool active;                       // is particle active or not?
    float origX, origY,origZ;


    //  void   initialize(){}

} PARTICLE;

//class User{

//    ParticleSystem  system;
//    int userId;


//}

class ParticleSystem{
private:



    int userId;


    //    int origX;
    //    int origY;

    // intialize the particles
public:
    vector<PARTICLE*> particles;

    bool isActive;

    float xscale;
    float yscale;
    float r,g,b;
    float dr, dg, db;

    ParticleSystem(int id,float r,float g,float b, float dr,float dg,float db){
        this->isActive = true;
        this->userId = id;
        this->r = r;
        this->g = g;
        this->b = b;
        this->dr = dr;
        this->dg = dg;
        this->db = db;
//        particles = new particles[MAX_PARTICLES];
//        cout<<"New particles :)  \n";
        //        particles.intialize();
        // TODO initialize the particles here currently this  is happening in createParticle system BADDDD!!!

    }


};




class GLWidget : public QGLWidget
{
    Q_OBJECT
public:

    bool defaultMode;
    GLWidget(QWidget*parent);
    ~GLWidget();
    void initializeGL();
    void resizeGL(int w, int h);
    void paintGL();




    void DrawGLScene(void);

    // this function updates the particles in the scene
    void EvolveParticles();
    void CreateParticle(int i);


    void createParticleSystemForUser(int id);
//    void initializeSingleUserParticle(int i,int user);
    void initializeSingleUserParticle(int i,PARTICLE* p,float r,float g,float b);


    void destroyParticleSystemForUser(int id);
    void splashParticleSystem(int x, int y, int deviceID,bool left);
    void toggleMode();

protected:
    void keyPressEvent (QKeyEvent *e);

private:


    map<int , ParticleSystem*> ParticleSystems;

    // map containing the particle systems given to the left hand
    map<int , ParticleSystem*> ParticleSystemsLeft  ;

    vector<PARTICLE*> defaultParticles;
//    PARTICLE defaultParticles[MAX_PARTICLES];

    int screenWidth;
    int screenHeight;



};
#endif
