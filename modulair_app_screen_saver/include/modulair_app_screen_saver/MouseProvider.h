#ifndef _Mouseprovider_h
#define _Mouseprovider_h

#include <iostream>
#include <modulair_app_screen_saver/mouse/manymouse.h>
#include <SDL/SDL.h>
#include <SDL/SDL_timer.h>
#include <QThread>
#include <QTimer>
//namespace modulair{
typedef struct
{
    int connected;
    int x;
    int y;
    SDL_Color color;
    char name[64];
    Uint32 buttons;
    Uint32 scrolluptick;
    Uint32 scrolldowntick;
    Uint32 scrolllefttick;
    Uint32 scrollrighttick;
    int deviceID;
} Mouse;



/*!
 \brief Wrapper class for ROS kinect input code

*/
class MouseProvider : public QThread{
  Q_OBJECT // must include this if you use Qt signals/slots
public:
  //! Constructor
  MouseProvider(QWidget *parent);
  //! Destructor
  ~MouseProvider(){}

  //int getTheNumberOfAvailableMice();
  void setMouseUpdatePeriod(const int period){
    MouseUpdatePeriod = period;
  }

private:
  //! Spawns a new timer on a new thread, and cleans up the Mouse when done
  void run();

  //! Overide Mouse_fusor callback
 // void stateCallback(const Mouse::StateConstPtr &state);

//  void timerEvent(QTimerEvent *);


public Q_SLOTS:

  //! Slot which is triggered by the timer
  /*!
    This is where all of the desired functionality of the MouseProvider
    happens, once every time the timer fires
    \sa QTimer::timerHit()
    \sa MouseProvider::timerHit()
  */
 // void timerHit();
void updateMouse();

Q_SIGNALS:
  //! Signal for button pressed & released event
  //  void wiiButton1Sig(int button, int state);
    void mouseClickEvent(Mouse* mouse,bool pressed);
    void mouseClickEvent(int x,int y,int id,bool pressed);
    void mouseMoveEvent(Mouse* mouse);
    void mouseMoveEvent(int x,int y,int id);
private:
  int MouseUpdatePeriod; /**< Mouse update period */
  int availableMice;
  Mouse mice[3];

};


//} // namespace::modulair

#endif //_Mouseprovider_h
