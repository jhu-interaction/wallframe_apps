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

#include <modulair_app_screen_saver/glWidget.h>
#include <modulair_app_screen_saver/screen_saver_app.h>
#include <QtGui/QImage>
#include <GL/glut.h>
#include <GL/glu.h>
#include <math.h>
#include <iostream>

#define PI 3.14159265

using namespace std;


 float R = 0.8f;
 float G = 0.0f;
 float B = 0.5f;
 float cR = 0.01f;
 float cG = 0.01f;
 float cB = 0.01f;
 float theta = 0;


GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent)
{

//    screenHeight = 800;
//    screenWidth = 1200;

    //TODO set the screen size and width as the size of the widget

    // default mode particles
    defaultMode = true;
    // create the default particles system
    for(int i = 0; i <=MAX_PARTICLES;i++){

        defaultParticles[i].origX = 0;
        defaultParticles[i].origY = 0;
        defaultParticles[i].origZ = 0;
        CreateParticle(i);
    }


}

GLWidget::~GLWidget()
{
    map<int , ParticleSystem*>::iterator it;

    for(it = ParticleSystems.begin() ; it != ParticleSystems.end() ; it++){
        ParticleSystem* system = it->second;

        for(int i=0;i<=MAX_PARTICLES;i++){
            PARTICLE* p = system->particles.at(i);
            delete p;
        }

         delete system;
     }
    for(it = ParticleSystemsLeft.begin() ; it != ParticleSystemsLeft.end() ; it++){
        ParticleSystem* system = it->second;
        for(int i=0;i<=MAX_PARTICLES;i++){
            PARTICLE* p = system->particles.at(i);
            delete p;
        }
        delete system;
     }
}

void GLWidget::initializeGL()
{

    //// I do not know if this is the right place to do so
    screenWidth = 1200;
    screenHeight = 800;

    glViewport(0,0,screenWidth,screenHeight);						// Reset The Current Viewport

    glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
    glLoadIdentity();									// Reset The Projection Matrix

///////////////////////////////////////////////////////////////////////////////////////////////////


}

void GLWidget::resizeGL(int w, int h)
{
 	glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
 	glMatrixMode (GL_PROJECTION);
 	glLoadIdentity ();
 	glMatrixMode (GL_MODELVIEW);

    cout<< "Width "<< w << "Height "<< h << endl;
    screenWidth = w;
    screenHeight = h;
}
void GLWidget::keyPressEvent(QKeyEvent *e)
{
    DrawGLScene();
   switch (e->key()) {
      // case Key_Escape:
         // exit(0);
         break;
   }
}

void GLWidget::paintGL()
{
   glClear (GL_COLOR_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   DrawGLScene();
}


void GLWidget::DrawGLScene(void){


//    glPointSize(2.0);
    glPointSize(6.0);
  //glBindTexture(GL_TEXTURE_2D,ParticleTexture);          // choose particle texture
    glBegin(GL_POINTS);

    if(defaultMode){
        for (int i=0;i<=MAX_PARTICLES;i++){
            //      if(defaultParticles[i].ypos<0.0) defaultParticles[i].lifetime=0.0;
            if((defaultParticles[i].active==true) && (defaultParticles[i].lifetime>0.0)){

            } else CreateParticle(i);

            glColor4f(defaultParticles[i].r,defaultParticles[i].g,defaultParticles[i].b,0.5f);
            glVertex3f(defaultParticles[i].xpos, defaultParticles[i].ypos, defaultParticles[i].zpos);

        }
    }
    else{

        map<int , ParticleSystem*>::iterator it;

        // iterate over the particle system map to draw the active particle systems
        for(it = ParticleSystems.begin() ; it != ParticleSystems.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive){

                int id = it->first;
//                cout<<"Drawing for id "<<id<<endl;

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    if((p->lifetime>0.0)){

                    } else{

                        initializeSingleUserParticle(i,p,p->r,p->g,p->b);
                    }

                    glColor4f(p->r,p->g,p->b,0.5f);
                    glVertex3f(p->xpos, p->ypos, p->zpos);
                }

            }
        }



        // iterate over the particle system map to draw the active particle systems
        for(it = ParticleSystemsLeft.begin() ; it != ParticleSystemsLeft.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive){

                int id = it->first;
//                cout<<"Drawing for id "<<id<<endl;

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    if((p->lifetime>0.0)){

                    } else{

                        initializeSingleUserParticle(i,p,p->r,p->g,p->b);
                    }

                    glColor4f(p->r,p->g,p->b,0.5f);
                    glVertex3f(p->xpos, p->ypos, p->zpos);
                }

            }
        }

    }

    glEnd();
    EvolveParticles();

}


void GLWidget::EvolveParticles()
{
    // if the mode is default then update the default particle system
    if(defaultMode == true){

        for(int i=0;i<=MAX_PARTICLES;i++){      // evolve the particle parameters
            defaultParticles[i].lifetime-=defaultParticles[i].decay;
            defaultParticles[i].xpos+=defaultParticles[i].xspeed * 0.1 ;
            defaultParticles[i].ypos+=defaultParticles[i].yspeed * 0.1;
            defaultParticles[i].zpos+=defaultParticles[i].zspeed * 0.1;

            float r = 0.2  * theta / 360;//* sin(theta * PI / 180.0 );
            //if(r < 0 )
              //  r *= -1;
            defaultParticles[i].origX = r * cos(theta * PI / 180.0);
//            cout <<"default orig x" << defaultParticles[i].origX <<"\n";
            defaultParticles[i].origY = r * sin(theta * PI / 180.0);
            theta += 1;

            if(theta >= 1200)
                theta = 0;
            //     defaultParticles[i].yspeed-=0.007;
        }
    }
    else { // update the user particle system
        map<int , ParticleSystem*>::iterator it;

        // iterate over the particle system map to draw the active particle systems
        for(it = ParticleSystems.begin() ; it != ParticleSystems.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive == true){

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    p->lifetime-=p->decay;

                    p->r -= system->dr;

                    p->g -= system->dg;

                    p->b -= system->db;

                    if(p->r > 1.0f){p->r=1.0f; system->dr=-system->dr;}if(p->r<0.0f){p->r=0.0f; system->dr=-system->dr;}
                    if(p->g > 1.0f){p->g=1.0f; system->dg=-system->dg;}if(p->g<0.0f){p->r=0.0f; system->dg=-system->dg;}
                    if(p->b > 1.0f){p->b=1.0f; system->db=-system->db;}if(p->b<0.0f){p->r=0.0f; system->db=-system->db;}


                    p->xpos += p->xspeed * 0.06 * system->xscale  * p->lifetime  ;

                    p->ypos += p->yspeed * 0.06 * system->yscale * p->lifetime    ;
                    p->zpos += p->zspeed * 0.05;

//                    if(i == 0)
//                        cout<<"p orig" << p->origX <<" ";


                }

            }


        }



        // iterate over the particle system map to draw the active particle systems
        for(it = ParticleSystemsLeft.begin() ; it != ParticleSystemsLeft.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive == true){

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    p->lifetime-=p->decay;

                    p->r -= system->dr;

                    p->g -= system->dg;

                    p->b -= system->db;

                    if(p->r > 1.0f){p->r=1.0f; system->dr=-system->dr;}if(p->r<0.0f){p->r=0.0f; system->dr=-system->dr;}
                    if(p->g > 1.0f){p->g=1.0f; system->dg=-system->dg;}if(p->g<0.0f){p->r=0.0f; system->dg=-system->dg;}
                    if(p->b > 1.0f){p->b=1.0f; system->db=-system->db;}if(p->b<0.0f){p->r=0.0f; system->db=-system->db;}

                    p->xpos += p->xspeed * 0.06 * system->xscale  * p->lifetime  ;

                    p->ypos += p->yspeed * 0.06 * system->yscale * p->lifetime    ;
                    p->zpos += p->zspeed * 0.05;



                }

            }


        }
    }

}
void GLWidget::CreateParticle(int i)
{

     defaultParticles[i].lifetime= (float)(rand() % 500000 )/500000.0;
     defaultParticles[i].decay=0.02;
     defaultParticles[i].r = 0.7;
     defaultParticles[i].g = 0.7;
     defaultParticles[i].b = 1.0;
     defaultParticles[i].xpos= defaultParticles[i].origX + 0.0;
     defaultParticles[i].ypos= defaultParticles[i].origY + 0.0;
     defaultParticles[i].zpos= defaultParticles[i].origZ + 0.0;


     defaultParticles[i].xspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
     defaultParticles[i].yspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
     defaultParticles[i].zspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;

     defaultParticles[i].r = R;
     defaultParticles[i].g = G;
     defaultParticles[i].b = B;
     R+=cR;
     G+=cG;
     B+=cB;

     if(R>1.0f){R=1.0f; cR=-cR;}if(R<0.0f){R=0.0f; cR=-cR;}
     if(G>1.0f){G=1.0f; cG=-cG;}if(G<0.0f){G=0.0f; cG=-cG;}
     if(B>1.0f){B=1.0f; cB=-cB;}if(B<0.0f){B=0.0f; cB=-cB;}


     int xdirection = 1;
     int ydirection = 1;
     if(i%4 == 1){
        xdirection = -1;
        ydirection = 1;
     }
     else if(i%4 == 2) {
         xdirection = -1;
         ydirection = -1;
     }
     else if(i%4 == 3) {
         xdirection = 1;
         ydirection = -1;
     }

     defaultParticles[i].xspeed *= -xdirection;
     defaultParticles[i].yspeed *= -ydirection;


     if(i%2 == 0)
        defaultParticles[i].zspeed *= -1;

     defaultParticles[i].active = true;
}

void GLWidget::toggleMode(){

    if(defaultMode)
        defaultMode = false;
    else
        defaultMode = true;

}
void GLWidget::splashParticleSystem(int x, int y, int id,bool left){

    map<int,ParticleSystem*>::iterator it;

    if(left)
        it = ParticleSystemsLeft.find(id);
    else
        it = ParticleSystems.find(id);

    // insert the Particle system for the user in the map
    if((left && (it == ParticleSystemsLeft.end())) || ((left == false) && (it == ParticleSystems.end())) ){ //  if the id is not already there in the map
//        cout<<"Id not there in the map"<<id<<endl;
        return;

}
    ParticleSystem *system = it->second;

    if(!system){
        cout<<"There is no system associated with this user";
        return;
    }

    int screenWidth = this->size().width();
    int screenHeight = this->size().height();
    float screenX =(float) ((float)(x - (screenWidth/2)))/(screenWidth/2);
    float screenY =(float)  (y - (screenHeight/2))/(screenHeight/2);


    // TODO ideally the origin be with the particle system not the particles
    for(int i=0;i<=MAX_PARTICLES;i++){
        PARTICLE* p = system->particles.at(i);

        if(i ==0){
//        cout <<"xdiff" << (screenX - p->origX) << " ";
//        cout <<"ydiff" << (screenY - p->origY) << " " <<endl;

        system->xscale = ((float)(p->origX - screenX)) /0.01;
        if(system->xscale < 0)
            system->xscale *= -1;

        system->yscale = ((float)(p->origY - screenY)) /0.01;

        if(system->yscale < 0)
            system->yscale *= -1;


        if(system->yscale < 2)
            system->yscale = 2;

        if(system->xscale < 2)
            system->xscale = 2;

        // set the minimum threshold for the scale factor we do not want the speed to scale to zero ever

//        cout<<" x "<<system->yscale << " ";
//        cout<<"y " <<system->xscale << " ";

       }
        p->origX = screenX;
        p->origY = screenY;


        }
    return;

}

// creates both the left and the right particle system
void GLWidget::createParticleSystemForUser(int id){


    cout<<"Creating the particle system for the user"<< id<<endl;
    map<int,ParticleSystem*>::iterator it = ParticleSystems.find(id);

    // insert the Particle system for the user in the map
    if(it == ParticleSystems.end()) //  if the id is not already there in the map
    {
        float r = 0.8f, g = 0.0f , b = 0.0f;
        float dr = 0.002, dg = 0.003 , db = 0.001;

        cout<<"Initializing the particles\n";
        ParticleSystem* system = new ParticleSystem(id,r,g,b,dr,dg,db);

        if(!system)
            cout<<"I do not know why I did not get memory\n";

        // TODO ideally this should be a part of the particle class
        for(int i=0;i<=MAX_PARTICLES;i++){

            PARTICLE* p= new PARTICLE();
            system->particles.push_back(p);

            initializeSingleUserParticle(i,p,r,g,b);

        }
        ParticleSystems[id] = system;

    }
    else{
        ParticleSystem* system = (*it).second;
        if(system->isActive == false)
            system->isActive = true;
        else
            cout<<"USER already exists in the particle map";

    }

    it = ParticleSystemsLeft.find(id);

    // insert the Particle system for the user in the map
    if(it == ParticleSystemsLeft.end()) //  if the id is not already there in the map
    {
        float r = 0.0f, g = 0.0f , b = 1.0f;
        float dr = 0.002, dg = 0.001 , db = 0.003;
        cout<<"Initializing the particles\n";
        ParticleSystem* system = new ParticleSystem(id,r,g,b,dr,dg,db);

        if(!system)
            cout<<"I do not know why I did not get memory\n";

        // TODO ideally this should be a part of the particle class
        for(int i=0;i<=MAX_PARTICLES;i++){

            PARTICLE* p= new PARTICLE();
            system->particles.push_back(p);

            initializeSingleUserParticle(i,p,r,g,b);

        }
        ParticleSystemsLeft[id] = system;

    }


    cout<<"Left and right Particle system succesfully created\n";
    return;

}

// TODO thinking to make the particle system inactive instead of deleting
void GLWidget::destroyParticleSystemForUser(int id){

    cout<<"Destroying the right particle system";
    map<int,ParticleSystem*>::iterator it = ParticleSystems.find(id);

    // if the user does not exist in the map then return
    if(it == ParticleSystems.end())
        return;
     //  get the particle system for the user from the map
    ParticleSystem *system = it->second;

    if(!system){

    // instead of deallocating the memory just make the particle system inactive
    system->isActive = false;
    }

    cout<<"Destroying the left particle system";
    it = ParticleSystemsLeft.find(id);

    // if the user does not exist in the map then return
    if(it == ParticleSystemsLeft.end())
        return;
     //  get the particle system for the user from the map
    ParticleSystem *system2 = it->second;

    if(!system){
//        cout<<"There is no system associated with this user";
        return;
    }

    // instead of deallocating the memory just make the particle system inactive
    system2->isActive = false;


    cout<<"Particle system deleted"<< id<<endl;

    return;

}
// TODO start from the position of the user
void GLWidget::initializeSingleUserParticle(int i,PARTICLE* p,float r, float g, float b){

    p->lifetime= (float)(rand() % 500000 )/500000.0;
//    cout<<"lifetime" << p->lifetime << "\n";
    p->decay=0.01;

    p->xpos= p->origX + 0.0;
    p->ypos= p->origY + 0.0;
    p->zpos= p->origZ + 0.0;

    p->xspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
    p->yspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
    p->zspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;

    p->r = r;
    p->g = g;
    p->b = b;



    int xdirection = 1;
    int ydirection = 1;
    if(i%4 == 1){
       xdirection = -1;
       ydirection = 1;
    }
    else if(i%4 == 2) {
        xdirection = -1;
        ydirection = -1;
    }
    else if(i%4 == 3) {
        xdirection = 1;
        ydirection = -1;
    }

    p->xspeed *= -xdirection;
    p->yspeed*= -ydirection;

//    if(i%2 == 0)
//       p->zspeed *= -1;

    p->active = true; //    TODO we do not need this active field

}


