import cv2 as cv
import numpy as np
import glob
import argparse
import os
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument('img_path', help='images folder path')
    parser.add_argument('out_img_path', help=' output images folder path')
    parser.add_argument('depth_factor', help='depth factor')

    args = parser.parse_args()
    flist=glob.glob(args.img_path + '*.png')
    out_img_path = args.out_img_path
    depth_factor = args.depth_factor
    os.makedirs(out_img_path, exist_ok=True)
    for filename in flist:
        # print(os.path.basename(filename))
        img_name = os.path.basename(filename)
        img = cv.imread(filename,cv.IMREAD_UNCHANGED)
        img = img / np.float32(depth_factor)
        img_filtered = cv.bilateralFilter(img,9,80,80)
        img_out = img_filtered * np.float32(depth_factor)
        img_out = img_out.astype(np.uint16)
        cv.imshow("source",img)
        cv.imshow("result",img_filtered)
        cv.imwrite(out_img_path + os.path.basename(filename),img_out)
        cv.waitKey(1)