import cv2 as cv
import numpy as np
import argparse
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument('img_path', help='image path')

    args = parser.parse_args()

    depth = cv.imread(args.img_path,cv.IMREAD_UNCHANGED)
    v_max, u_max = depth.shape
    img = depth / np.float32(1000.0)
    for u in range(u_max):
        for v in range(v_max):
            if (img[v, u] > 3.0):
                img[v, u] = 0.0
    # img_filtered = depth

    img8U = cv.convertScaleAbs(img, alpha=(255.0/img.max()))


    img_filtered = cv.bilateralFilter(img8U, 9, 75, 75,cv.BORDER_DEFAULT)



    # outputImg8U = cv.convertScaleAbs(img_filtered, alpha=(255.0/img_filtered.max()))

    cv.imshow("depth",img_filtered)


    cv.waitKey(0)