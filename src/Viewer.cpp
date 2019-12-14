#include "Viewer.h"
#include <math.h>
Viewer::Viewer()
{
    mKeyFrameSize = 0.05;
    mKeyFrameLineWidth = 1;
    mPointSize = 0.01;

    mFrameColor[0] = 0.;
    mFrameColor[1] = 0.;
    mFrameColor[2] = 1.;

    mPointColor[0] = 1.;
    mPointColor[1] = 0.;
    mPointColor[2] = 0.;

    mRefPointColor[0] = 0.;
    mRefPointColor[1] = 0.;
    mRefPointColor[2] = 1.;
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
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowConnect("menu.Show Connect",true,true);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowRefPoints("menu.Show RefPoints",true,true);
//    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
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
        if(menuShowKeyFrames)
        {
            for(int i = 0; i < m_frames.size(); i++)
            {
                drawFrame(m_frames[i]);
                if(i>0 && menuShowConnect)
                {
                    Eigen::Vector3f last,curr;
                    last = m_frames[i-1].getConstTwc().topRightCorner(3,1);
                    curr = m_frames[i].getConstTwc().topRightCorner(3,1);
                    glLineWidth(1);
                    glColor3f(0.0f,1.0f,0.0f);
                    glBegin(GL_LINES);
                    glVertex3f(last[0],last[1],last[2]);
                    glVertex3f(curr[0],curr[1],curr[2]);
                    glEnd();
                }

            }
        }
        if(menuShowPoints)
        {
            for(auto point : m_points)
            {
                drawPoint(point,mPointColor);
            }
        }
        if(menuShowRefPoints)
        {
            for(auto point : m_refPoints)
            {
                drawPoint(point,mRefPointColor);
            }
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();
        if(menuReset)
        {
            menuShowKeyFrames = true;
            menuShowConnect = true;
            menuShowPoints = true;
            menuShowRefPoints = true;

        }
    }


}

void Viewer::drawPoint(Point point, GLfloat *color)
{
    Point::Position p = point.getConstPoint();
    glPointSize(mPointSize); //cm
    glBegin(GL_POINTS);
    glColor3f(color[0],color[1],color[2]);
    glVertex3f(p[0],p[1],p[2]);
    glEnd();
}

void Viewer::drawFrame(Frame frame)
{
    glPushMatrix();
    Frame::Pose Twc;
    Twc = frame.getConstTwc();

    GLfloat *m = (GLfloat*) Twc.data();


    glMultMatrixf(m);

    float w,h,z;
    w = mKeyFrameSize; //in meter
    h = 0.6*w;
    z = w;
    glLineWidth(mKeyFrameLineWidth);
    glColor3f(mFrameColor[0],mFrameColor[1],mFrameColor[2]);
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

namespace BAMapping
{
    Viewer::Viewer()
    {
        mKeyFrameSize = 0.05;
        mKeyFrameLineWidth = 1;
        mPointSize = 3;

        mFrameColor[0] = 0.;
        mFrameColor[1] = 0.;
        mFrameColor[2] = 1.;

        mPointColor[0] = 1.;
        mPointColor[1] = 0.;
        mPointColor[2] = 0.;

        mRefPointColor[0] = 0.;
        mRefPointColor[1] = 0.;
        mRefPointColor[2] = 1.;
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
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
        pangolin::Var<bool> menuShowConnect("menu.Show Connect",true,true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
        pangolin::Var<bool> menuShowRefPoints("menu.Show RefPoints",true,true);
//    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
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
            if(menuShowKeyFrames)
            {
                for(int i = 0; i < m_frames.size(); i++)
                {
                    drawFrame(m_frames[i]);
                    if(i>0 && menuShowConnect)
                    {
                        Eigen::Vector3d last,curr;
                        last = m_frames[i-1].getConstTwc().topRightCorner(3,1);
                        curr = m_frames[i].getConstTwc().topRightCorner(3,1);
                        glLineWidth(1);
                        glColor3f(0.0f,1.0f,0.0f);
                        glBegin(GL_LINES);
                        glVertex3f(last[0],last[1],last[2]);
                        glVertex3f(curr[0],curr[1],curr[2]);
                        glEnd();
                    }

                }
            }
            if(menuShowPoints)
            {
                for(auto point : m_points)
                {
                    drawPoint(point,mPointColor);
                }
            }
            if(menuShowRefPoints)
            {
                for(auto point : m_refPoints)
                {
                    drawPoint(point,mRefPointColor);
                }
            }

            // Swap frames and Process Events
            pangolin::FinishFrame();
            if(menuReset)
            {
                menuShowKeyFrames = true;
                menuShowConnect = true;
                menuShowPoints = true;
                menuShowRefPoints = true;

            }
        }


    }

    void Viewer::drawPoint(Point point, GLfloat *color)
    {
        auto p = point.getPoseInWorld();
        glPointSize(mPointSize); //cm
        glBegin(GL_POINTS);
        glColor3f(color[0],color[1],color[2]);
        glVertex3f(p[0],p[1],p[2]);
        glEnd();
    }

    void Viewer::drawFrame(Frame frame)
    {
        glPushMatrix();

        Eigen::Matrix4d Twc;
        Twc = frame.getConstTwc();
        Eigen::Affine3f T;
        T.matrix() = Twc.cast<float>();
        glMultMatrixf(T.data());

        float w,h,z;
        w = mKeyFrameSize; //in meter
        h = 0.6*w;
        z = w;
        glLineWidth(mKeyFrameLineWidth);
        glColor3f(mFrameColor[0],mFrameColor[1],mFrameColor[2]);
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

    void Viewer::appendFrames(FrameVector frames)
    {
        m_frames.insert(m_frames.end(),frames.begin(),frames.end());
    }

    void Viewer::appendPoints(PointVector points)
    {
        m_points.insert(m_points.end(),points.begin(),points.end());
    }
}