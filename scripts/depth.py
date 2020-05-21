import numpy as np
import cv2 as cv
from matplotlib import pyplot as plt

def smooth(path,id):
    depth = cv.imread(path + "/depth_new/DepthRealsense"+"%06.f"%id+".png",cv.IMREAD_UNCHANGED)

    ret = cv.medianBlur(depth,5)

    cv.imwrite(path + "/depth_filtered/DepthRealsense"+"%06.f"%id+".png",ret)

    # plt.subplot(121), plt.imshow(depth), plt.title('Original')
    # plt.xticks([]), plt.yticks([])
    # plt.subplot(122), plt.imshow(ret), plt.title('Blurred')
    # plt.xticks([]), plt.yticks([])
    # plt.show()
if __name__ == "__main__":
    for i in range(301,303):
        print("aligning %d\n"%i)
        smooth(".",i)

