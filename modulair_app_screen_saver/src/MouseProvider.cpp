
#include  <modulair_app_screen_saver/MouseProvider.h>

//using namespace modulair;
using namespace std;
#include <QThread>

MouseProvider::MouseProvider(QWidget *parent = NULL):QThread(0)

{
    int screenWidth = 1200;
    int screenHeight = 800;
    std::cout<<"Creating new mouse event\n";
    MouseUpdatePeriod = 30;
    availableMice = ManyMouse_Init();

    int  i;
    for (i = 0; i < availableMice; i++)
    {
        const char *name = ManyMouse_DeviceName(i);
        printf("#%d: %s\n", i, name);
    }
    for (i = 0; i < 3; i++)
    {
        Mouse *mouse = &mice[i];
        mouse->x = screenWidth/2;
        mouse->y = screenHeight/2;
        mouse->deviceID = i;

    }

}


void MouseProvider::run()
{
    std::cout<<"Running the mouse provider\n";
    //ROS_WARN_STREAM("<<< MouseProvider >>> Starting Up...");
    QTimer timer;
    QObject::connect(&timer, SIGNAL(timeout()),
                     this, SLOT(updateMouse()));
    timer.setInterval(MouseUpdatePeriod);
    // Start timer
    timer.start();
    //  timer.start(MouseUpdatePeriod);
    // Start thread loop
    exec();
    // Stop when finished
    timer.stop();
}

void MouseProvider::updateMouse(){
    //  std::cout<<"Updating the mouse";

    int screenWidth = 1200;
    int screenHeight = 800;

    ManyMouseEvent event;
    while (ManyMouse_PollEvent(&event))
    {
        Mouse *mouse;
        if (event.device >= (unsigned int) availableMice)
            continue;

        mouse = &mice[event.device];

        if (event.type == MANYMOUSE_EVENT_RELMOTION)
        {
            if (event.item == 0)
                mouse->x += event.value;
            else if (event.item == 1)
                mouse->y += event.value;

            if(mouse->x > screenWidth ){
                mouse->x = screenWidth;
            }
            else if(mouse->x < 0) {
                mouse->x =  0;
            }

            if(mouse->y > screenHeight){
                mouse->y = screenHeight;
            }
            else if(mouse->y < 0){
                mouse->y = 0;
            }


            //   std::cout<<"Mouse position x "<<mouse->x ;
            //  std::cout<<"Mouse position y "<<mouse->y ;
//            Q_EMIT mouseMoveEvent(mouse);
            Q_EMIT mouseMoveEvent(mouse->x,mouse->y,mouse->deviceID);

        }

        else if (event.type == MANYMOUSE_EVENT_BUTTON)
        {
            std::cout<<"Mouse";
            if (event.item < 32)
            {
                if (event.value)
                    mouse->buttons |= (1 << event.item);
                else
                    mouse->buttons &= ~(1 << event.item);
            }

            bool pressed = (event.value > 0) ? true : false;
            if(pressed){
                std::cout<<"Button pressed\n";

            }
            else{
                std::cout<<"Button released\n";
            }

            /*Q_EMIT mouseClickEvent(mouse,pressed); */// value tells about button press or release
            Q_EMIT mouseClickEvent(mouse->x,mouse->y,mouse->deviceID,pressed);
        }

        else if (event.type == MANYMOUSE_EVENT_DISCONNECT)
        {
            mice[event.device].connected = 0;
        }
    }

}


