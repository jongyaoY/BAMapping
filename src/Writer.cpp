#include "Writer.h"

Writer::Writer()
{

}

void Writer::writeToFile(Graph graph,const char* cam_filename,const char* point_filename)
{
    FILE* fptr = fopen(point_filename, "w");

    if (fptr == NULL)
    {
        return;
    }
    PointVector points = graph.getConstPoints();
    FrameVector frames = graph.getConstFrames();

    for(auto point : points)
    {
        fprintf(fptr,"%lf %lf %lf\n",point.getConstPoint()[0],
                                     point.getConstPoint()[1],
                                     point.getConstPoint()[2]);
    }
    fclose(fptr);
    fptr = fopen(cam_filename, "w");
    for(auto frame : frames)
    {
        const double angle = frame.getConstAngleAxis().angle();
        const double angleAxis[3] = {angle*frame.getConstAngleAxis().axis()[0],
                                     angle*frame.getConstAngleAxis().axis()[1],
                                     angle*frame.getConstAngleAxis().axis()[2]};
        const Eigen::Vector3d translation = frame.getConstTranslation();

        fprintf(fptr,"%lf %lf %lf %lf %lf %lf\n",angleAxis[0],
                                                 angleAxis[1],
                                                 angleAxis[2],
                                                 translation[0],
                                                 translation[1],
                                                 translation[2]);
    }
    fclose(fptr);
}
