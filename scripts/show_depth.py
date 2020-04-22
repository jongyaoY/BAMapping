import cv2 as cv
import numpy as np
import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument('img_path', help='image path')
    parser.add_argument('out_img_path', help='output image path')

    args = parser.parse_args()

    depth = cv.imread(args.img_path,-1)
    img = depth / np.float32(1000.0)

    # image = cv.cvtColor(img,cv.COLOR_GRAY2BGR)
    outputImg8U = cv.convertScaleAbs(img, alpha=(255.0/img.max()))
    # image = cv.cvtColor(outputImg8U,cv.COLOR_GRAY2BGR)
    cv.imshow("depth",outputImg8U)

    cv.imwrite(args.out_img_path,outputImg8U)
    # cv.imwrite("rgb_feature",img_rgb)

    cv.waitKey(0)