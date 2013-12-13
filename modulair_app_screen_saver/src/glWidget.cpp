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



using namespace std;
#define PI 3.14

 float R = 0.8f;
 float G = 0.2f;
 float B = 0.1f;
 float cR = 0.0005f;
 float cG = 0.0005f;
 float cB = 0.0005f;
float theta = 0;

GLWidget::GLWidget(QWidget* parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers),parent)
{

    defaultMode = true;

    PARTICLE* p = NULL;

    // initialize the default particle system
    float r = 0.4f, g = 0.05f , b = 0.5f;
    float dr = 0.0025, dg = 0.0005 , db = 0.0015;
    float decay = 0.004;

    defaultSystem = new ParticleSystem(-1,r,g,b,dr,dg,db,decay,0,0,0);

    // create the default particles system
    for(int i = 0; i <=MAX_PARTICLES;i++){
        p= new PARTICLE();

        defaultSystem->particles.push_back(p);
        initializeSingleUserParticle(i,p,r,g,b,0,0,0);

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
                   p = NULL;
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

    for(int i=0;i<=MAX_PARTICLES;i++){
               PARTICLE* p = defaultSystem->particles.at(i);
               delete p;
     }
    delete defaultSystem;

}

void GLWidget::initializeGL()
{

    //// I do not know if this is the right place to do so
    screenWidth = 1200;
    screenHeight = 800;

    glViewport(0,0,screenWidth,screenHeight);						// Reset The Current Viewport

    glMatrixMode(GL_PROJECTION);						// Select The Projection Matrix
    glLoadIdentity();									// Reset The Projection Matrix

    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);

///////////////////////////////////////////////////////////////////////////////////////////////////


}
    
void GLWidget::resizeGL(int w, int h){

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

void  GLWidget::paintGL()
{
   glClear (GL_COLOR_BUFFER_BIT);
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();

   DrawGLScene();
}


void GLWidget::DrawGLScene(void){


    glPointSize(10.0);
  //glBindTexture(GL_TEXTURE_2D,ParticleTexture);          // choose particle texture
    

    if(defaultMode){

        for (int i=0;i<=MAX_PARTICLES;i++){
            PARTICLE* p = defaultSystem->particles.at(i);
            if( p->lifetime <0)

                initializeSingleUserParticle(i,p,defaultSystem->r,defaultSystem->g,defaultSystem->b,defaultSystem->origX,defaultSystem->origY,defaultSystem->origZ);


            // cout<<p->lifetime * 10<< " ";
            glPointSize(10 - GLfloat(p->lifetime*10));
            glBegin(GL_POINTS);
            glColor4f(p->r,p->g,p->b,p->lifetime);
            glVertex3f(p->xpos, p->ypos, p->zpos);
            glEnd();
        }
    }
    else{

        glBegin(GL_POINTS);

        map<int , ParticleSystem*>::iterator it;

        // iterate over the particle system map to draw the active particle systems
        for(it = ParticleSystems.begin() ; it != ParticleSystems.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive){

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    if((p->lifetime>0.0)){

                    } else{

                        initializeSingleUserParticle(i,p,p->r,p->g,p->b,system->origX,system->origY,system->origZ);
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

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    if((p->lifetime>0.0)){

                    } else{

                        initializeSingleUserParticle(i,p,p->r,p->g,p->b,system->origX,system->origY,system->origZ);
                    }

                    glColor4f(p->r,p->g,p->b,0.5f);
                    glVertex3f(p->xpos, p->ypos, p->zpos);
                }

            }
        }

    glEnd();

    }

    EvolveParticles();

}


void GLWidget::EvolveParticles()
{
    // if the mode is default then update the default particle system
    if(defaultMode == true){

        // update the origin of the system
        // there are currently three modes in which the origin moves

        if(defMode == 0){  // cross helix
        int k1 = 7;
        int k2 = 4;
        defaultSystem->origX =(cos(k1*theta)) * 0.4;
        defaultSystem->origY = (k2*sin(theta)) * 0.2;
        defaultSystem->origZ = (k2 *cos(theta)) * 0.05;
        theta += 0.007;
        if(theta >= 8){
            theta = 0;
            defMode += 1;   // change to next mode
             
            defaultSystem->dr -= 0.001;   // changing these value to add some randomness to color changes
            defaultSystem->dg += 0.001;
            }    
        }

        if(defMode == 1){
        // spiral 
        float r = 0.2  * theta / 360;
        defaultSystem->origX = r * cos(theta * PI / 180.0);
        defaultSystem->origY = r * sin(theta * PI / 180.0);
        theta += 1.8;
        if(theta >= 1500){
            theta = 0;
            defMode += 1;    // change to next mode
            
            defaultSystem->dg -= 0.001;
            defaultSystem->db += 0.001;

            }
        }    

        if(defMode == 2){
        // /* Loop */
        double t = theta * PI / 180.0;
        double r = 1.5;
        double k = 8;
        defaultSystem->origX = (r * cos(t) - cos (k* t))/3;
        defaultSystem->origY = (r * sin(t) - sin (k* t))/3;
        theta += 0.5;

            if(theta >= 400){
                theta = 0;
                defMode = 0;      // change to the first mode again :) 
                defaultSystem->db -= 0.001;
                defaultSystem->dr += 0.001;

            }
        }

        // change the color, these are the colors new particle generated pick up
        defaultSystem->r -= defaultSystem->dr;
        defaultSystem->g -= defaultSystem->dg;
        defaultSystem->b -= defaultSystem->db;

        if(defaultSystem->r > 1.0f){defaultSystem->r=1.0f; defaultSystem->dr=-defaultSystem->dr;}if(defaultSystem->r<0.0f){defaultSystem->r=0.0f; defaultSystem->dr=-defaultSystem->dr;}
        if(defaultSystem->g > 1.0f){defaultSystem->g=1.0f; defaultSystem->dg=-defaultSystem->dg;}if(defaultSystem->g<0.0f){defaultSystem->r=0.0f; defaultSystem->dg=-defaultSystem->dg;}
        if(defaultSystem->b > 1.0f){defaultSystem->b=1.0f; defaultSystem->db=-defaultSystem->db;}if(defaultSystem->b<0.0f){defaultSystem->r=0.0f; defaultSystem->db=-defaultSystem->db;}

        for(int i=0;i<=MAX_PARTICLES;i++){

            PARTICLE* p = defaultSystem->particles.at(i);

            p->lifetime-= defaultSystem->decay ;
            p->xpos+=defaultSystem->particles.at(i)->xspeed * 0.1 ;
            p->ypos+=defaultSystem->particles.at(i)->yspeed * 0.1 ;
            p->zpos+=defaultSystem->particles.at(i)->zspeed * 0.1 ;

// commenting this out because once a particle is made I do not want to change its color
//            p->r -= defaultSystem->dr;
//            p->g -= defaultSystem->dg;
//            p->b -= defaultSystem->db;

//            if(p->r > 1.0f){p->r=1.0f; defaultSystem->dr=-defaultSystem->dr;}if(p->r<0.0f){p->r=0.0f; defaultSystem->dr=-defaultSystem->dr;}
//            if(p->g > 1.0f){p->g=1.0f; defaultSystem->dg=-defaultSystem->dg;}if(p->g<0.0f){p->r=0.0f; defaultSystem->dg=-defaultSystem->dg;}
//            if(p->b > 1.0f){p->b=1.0f; defaultSystem->db=-defaultSystem->db;}if(p->b<0.0f){p->r=0.0f; defaultSystem->db=-defaultSystem->db;}

        }
    }
    else {  // if not in the default mode update the user particle systems as per user pos
        map<int , ParticleSystem*>::iterator it;

        // iterate over the right particle system map to draw the active particle systems
        for(it = ParticleSystems.begin() ; it != ParticleSystems.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive == true){

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    p->lifetime -=system->decay;

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

        // iterate over the left particle system map to draw the active particle systems
        for(it = ParticleSystemsLeft.begin() ; it != ParticleSystemsLeft.end() ; it++){
            ParticleSystem* system = it->second;

            if(system && system->isActive == true){

                for(int i=0;i<=MAX_PARTICLES;i++){
                    PARTICLE* p = system->particles.at(i);
                    p->lifetime-=system->decay;

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

// this is the old default particle system create function TODO delete
// void GLWidget::CreateParticle(int i)
// {


//      defaultSystem->particles.at(i)->lifetime= (float)(rand() % 500000 )/500000.0;

//      defaultSystem->particles.at(i)->r = 0.7;
//      defaultSystem->particles.at(i)->g = 0.7;
//      defaultSystem->particles.at(i)->b = 1.0;

//      defaultSystem->particles.at(i)->xpos= 0;
//      defaultSystem->particles.at(i)->ypos= 0;
//      defaultSystem->particles.at(i)->zpos= 0;

// //     defaultSystem->particles.at(i)->xpos= defaultSystem->particles.at(i)->xpos > screenWidth/2 ? 0 : defaultSystem->particles.at(i)->xpos;
// //     defaultSystem->particles.at(i)->ypos= defaultSystem->particles.at(i)->ypos > screenHeight/2 ? 0 :defaultSystem->particles.at(i)->ypos ;
// //     defaultSystem->particles.at(i)->zpos= defaultSystem->particles.at(i)->zpos > screenHeight/2 ? 0 :defaultSystem->particles.at(i)->zpos ;

// //     defaultSystem->particles.at(i)->xpos= defaultSystem->particles.at(i)->origX + 0.0;
// //     defaultSystem->particles.at(i)->ypos= defaultSystem->particles.at(i)->origY + 0.0;
// //     defaultSystem->particles.at(i)->zpos= defaultSystem->particles.at(i)->origZ + 0.0;


//      defaultSystem->particles.at(i)->xspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
//      defaultSystem->particles.at(i)->yspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;
//      defaultSystem->particles.at(i)->zspeed = (((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f;

//      defaultSystem->particles.at(i)->r = R;
//      defaultSystem->particles.at(i)->g = G;
//      defaultSystem->particles.at(i)->b = B;
//      R+=cR;
//      G+=cG;
//      B+=cB;

//      if(R>1.0f){R=1.0f; cR=-cR;}if(R<0.0f){R=0.0f; cR=-cR;}
//      if(G>1.0f){G=1.0f; cG=-cG;}if(G<0.0f){G=0.0f; cG=-cG;}
//      if(B>1.0f){B=1.0f; cB=-cB;}if(B<0.0f){B=0.0f; cB=-cB;}


//      int xdirection = 1;
//      int ydirection = 1;
//      if(i%4 == 1){
//         xdirection = -1;
//         ydirection = 1;
//      }
//      else if(i%4 == 2) {
//          xdirection = -1;
//          ydirection = -1;
//      }
//      else if(i%4 == 3) {
//          xdirection = 1;
//          ydirection = -1;
//      }

//      defaultSystem->particles.at(i)->xspeed *= -xdirection;
//      defaultSystem->particles.at(i)->yspeed *= -ydirection;


//      if(i%2 == 0)
//         defaultSystem->particles.at(i)->zspeed *= -1;

// }

void GLWidget::toggleMode(){

    if(defaultMode)
        defaultMode = false;
    else
        defaultMode = true;

}
// this is the function called when user positon is updated
void GLWidget::splashParticleSystem(int x, int y, int id,bool left){

    map<int,ParticleSystem*>::iterator it;

    if(left)
        it = ParticleSystemsLeft.find(id);
    else
        it = ParticleSystems.find(id);

    // insert the Particle system for the user in the map
    if((left && (it == ParticleSystemsLeft.end())) || ((left == false) && (it == ParticleSystems.end())) ){ //  if the id is not already there in the map
        return;
    }

    /* Update the origin of the system */
    /* Evolve particles will update the positions of the particles*/
    ParticleSystem *system = it->second;

    if(!system){
        cout<<"There is no system associated with this user";
        return;
    }

    int screenWidth = this->size().width();
    int screenHeight = this->size().height();
    float screenX =(float) ((float)(x - (screenWidth/2)))/(screenWidth/2);
    float screenY =(float)  (y - (screenHeight/2))/(screenHeight/2);


    system->xscale = ((float)(system->origX - screenX)) /0.01;
    if(system->xscale < 0)
        system->xscale *= -1;

    system->yscale = ((float)( system->origY- screenY)) /0.01;

    if(system->yscale < 0)
        system->yscale *= -1;

    if(system->yscale < 2)
        system->yscale = 2;

    if(system->xscale < 2)
        system->xscale = 2;

    system->origX = screenX;
    system->origY = screenY;

    return;

}

// creates both the left and the right particle system for a new user
void GLWidget::createParticleSystemForUser(int id){

    cout<<"Creating the particle system for the user"<< id<<endl;
    map<int,ParticleSystem*>::iterator it = ParticleSystems.find(id);

    // insert the Particle system for the user in the map
    if(it == ParticleSystems.end()) //  if the id is not already there in the map
    {
        float r = 0.8f, g = 0.0f , b = 0.0f;
        float dr = 0.0025, dg = 0.002 , db = 0.001;
        float decay = 0.01;

        ParticleSystem* system = new ParticleSystem(id,r,g,b,dr,dg,db,decay,0,0,0);

        if(!system)
            cout<<"I do not know why I did not get memory\n";

        // TODO ideally this should be a part of the particle class
        for(int i=0;i<=MAX_PARTICLES;i++){

            PARTICLE* p= new PARTICLE();
            system->particles.push_back(p);

            initializeSingleUserParticle(i,p,r,g,b,system->origX,system->origY,system->origZ);

        }
        ParticleSystems[id] = system;

    }
    else{
        ParticleSystem* system = (*it).second;
        if(system->isActive == false)
            system->isActive = true;
    }

    it = ParticleSystemsLeft.find(id);

    // insert the Particle system for the user in the map
    if(it == ParticleSystemsLeft.end()) //  if the id is not already there in the map
    {
        float r = 0.0f, g = 0.0f , b = 1.0f;
        float dr = 0.002, dg = 0.001 , db = 0.002;
        float decay = 0.01;
        cout<<"Initializing the particles\n";

        ParticleSystem* system = new ParticleSystem(id,r,g,b,dr,dg,db,decay,0,0,0);

        if(!system)
            cout<<"I do not know why I did not get memory\n";

        // TODO ideally this should be a part of the particle class
        for(int i=0;i<=MAX_PARTICLES;i++){

            PARTICLE* p= new PARTICLE();
            system->particles.push_back(p);

            initializeSingleUserParticle(i,p,r,g,b,system->origX,system->origY,system->origZ );

        }
        ParticleSystemsLeft[id] = system;

    }
    else{
        ParticleSystem* system = (*it).second;
        if(system->isActive == false)
            system->isActive = true;
        else
            cout<<"USER already exists in the particle map";

    }

    cout<<"Left and right Particle system succesfully created\n";
    return;

}

// makes the left and the right particle system of the user inactive
void GLWidget::destroyParticleSystemForUser(int id){

    map<int,ParticleSystem*>::iterator it = ParticleSystems.find(id);

    // if the user does not exist in the map then return
    if(it == ParticleSystems.end())
        return;

    //  get the particle system for the user from the map
    ParticleSystem *system = it->second;

    if(!system){
        return;
    }

    // instead of deallocating the memory just make the particle system inactive
    system->isActive = false;

    it = ParticleSystemsLeft.find(id);

    // if the user does not exist in the map then return
    if(it == ParticleSystemsLeft.end())
        return;

    //  get the particle system for the user from the map
    ParticleSystem *system2 = it->second;

    if(!system){
        return;
    }

    // instead of deallocating the memory just make the particle system inactive
    system2->isActive = false;

    return;

}

// TODO start from the position of the user
void GLWidget::initializeSingleUserParticle(int i,PARTICLE* p,float r, float g, float b,float origX,float origY,float origZ){

    p->lifetime= (float)(rand() % 500000 )/500000.0;

    p->xpos= origX;

    // This is done only when the default mode is on 
    if(i % 2 == 0  && defMode != -1)
        p->xpos *= -1;

    p->ypos= origY;
    p->zpos= origZ;

    // This is done only if the default mode is on and is equal to 1
    if(defMode == 1){
        if(i % 2 != 0){
            p->ypos *= -1;
        }
    }

    p->xspeed = ((((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f) * 0.4;
    p->yspeed = ((((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f) * 0.4;
    p->zspeed = ((((float)((rand() % 100) + 1)) / 4000.0f) - 0.005f) * 0.4;

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
    p->yspeed *= -ydirection;


}


