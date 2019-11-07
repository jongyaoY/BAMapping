#include "Viewer.h"
#include <math.h>
Viewer::Viewer()
{

}
void Viewer::visualize()
{
    pangolin::CreateWindowAndBind("BAMapping",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);

    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,510,510,512,389,0.1,1000),
                pangolin::ModelViewLookAt(0,0,1,0,0,0,pangolin::AxisY)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new pangolin::Handler3D(s_cam));

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    while( !pangolin::ShouldQuit() )
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);
        for(int i = 0; i < m_frames.size(); i++)
        {
            drawFrame(m_frames[i]);
            if(i>0)
            {
//                Eigen::Vector3f last,curr;
//                last = m_frames[i-1].getConstTwc().topRightCorner(3,1);
//                curr = m_frames[i].getConstTwc().topRightCorner(3,1);
//                glLineWidth(1);
//                glColor3f(0.0f,1.0f,0.0f);
//                glBegin(GL_LINES);
//                glVertex3f(last[0],last[1],last[2]);
//                glVertex3f(curr[0],curr[1],curr[2]);
//                glEnd();
            }

        }
//        pangolin::glDrawColouredCube();

        for(auto point : m_points)
        {
            drawPoint(point);
        }
        for(auto point : m_refPoints)
        {
            drawRefPoint(point);
        }
        // Swap frames and Process Events
        pangolin::FinishFrame();
    }


}
void Viewer::drawRefPoint(Point point)
{
    Point p(point);
    p.color<<0,0,1;
    Point::Position pos = p.getConstPoint();
    glPointSize(500*p.pointSize); //cm
    glBegin(GL_POINTS);
    glColor3f(p.color[0],p.color[1],p.color[2]);
    glVertex3f(pos[0],pos[1],pos[2]);
    glEnd();
}

void Viewer::drawPoint(Point point)
{
    Point::Position p = point.getConstPoint();
    glPointSize(10*point.pointSize); //cm
    glBegin(GL_POINTS);
    glColor3f(point.color[0],point.color[1],point.color[2]);
    glVertex3f(p[0],p[1],p[2]);
    glEnd();
}

void Viewer::drawFrame(Frame frame)
{
    glPushMatrix();

//    Frame::Pose Tcw ;//= Eigen::Matrix4d::Identity();
    Frame::Pose Twc;
//    Tcw = frame.getConstPose();
    Twc = frame.getConstTwc();
//    Twc = frame.getConstTcw();
    GLfloat *m = Twc.data();
//    GLfloat *m = Tcw.data();
    glMultMatrixf(m);

    float w,h,z;
    w = frame.frameSize; //in meter
    h = 0.6*w;
    z = 0.5*w;
    glLineWidth(1);
    glColor3f(0.0f,0.0f,1.0f);
    glBegin(GL_LINES);
    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);

    glVertex3f(w,h,z);
    glVertex3f(w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);

    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);

    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glEnd();

    glPopMatrix();
}
