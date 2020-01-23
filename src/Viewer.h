#ifndef VIEWER_H
#define VIEWER_H

#include <pangolin/pangolin.h>
#include <Eigen/Core>

#include "Frame.h"
#include "Point.h"

namespace BAMapping
{
    class Viewer
    {
    public:
        Viewer();
        void visualize();
        inline void setRefPoints(PointVector points){m_refPoints = points;}
        inline void setPoints(PointVector points){m_points = points;}
        inline void setFrames(FrameVector frames){m_frames = frames;}
        void appendFrames(FrameVector frames);
        void appendPoints(PointVector points);
    private:
        void drawPoint(const Point point, GLfloat* color);
        void drawFrame(const Frame frame);
        void drawXYPlane();
        PointVector m_points;
        PointVector m_refPoints;
        FrameVector m_frames;

        float mKeyFrameSize;
        float mKeyFrameLineWidth;
        float mPointSize;
        GLfloat mFrameColor[3];
        GLfloat mPointColor[3];
        GLfloat mRefPointColor[3];
    };

}
#endif // VIEWER_H
