#ifndef WALLBALLWIDGET_H
#define WALLBALLWIDGET_H

#include <QWidget>
#include <QGLWidget>
#include <QPaintEvent>
#include <QResizeEvent>

#include <wallframe_core/wallframe_app_base_qt.h>

#include <WallBall.h>


class WallBallWidget : public wallframe::WallframeAppBaseQt
{
    //Q_OBJECT;

public:
    WallBallWidget(std::string app_name, ros::NodeHandle nh, int event_deque_size);
    ~WallBallWidget();

    void doResize(int w, int h);
    
    bool build();
    bool start();
    bool stop();
    bool pause();
    bool resume();

protected:

    // USERS //
    void updateUsers();

    int m_NumActiveUsers;
    
    // Qt Events
    void resizeEvent (QResizeEvent* event);
	void paintEvent ( QPaintEvent * event );
	void keyPressEvent(QKeyEvent *event);
	void keyReleaseEvent(QKeyEvent *event);

private:
	static bool s_RunPhysics;
	static void* physicsThreadMethod(void* data);
}; // class WallBallWidget


#endif // WALLBALLWIDGET_H
