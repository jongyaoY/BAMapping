#include <stdio.h>
#include "System.h"
#include "Viewer.h"
#include "Reader.h"

int main(int argc, char** argv)
{
    printf("hello world\n");
    Viewer viewer;
    Reader reader;
    FrameVector frame_vec;
    reader.readFrames(frame_vec,"../files/camera");
    viewer.setFrames(frame_vec);
    viewer.visualize();
    return 0;
}
