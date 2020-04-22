import cv2 as cv
import numpy as np
import argparse
# import matplotlib.pyplot as plt
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Optional app description')
    parser.add_argument('img_path_1', help='image path 1')
    parser.add_argument('img_path_2', help='image path 2')
    parser.add_argument('--bf', dest='bruteforce', action='store_true')
    parser.add_argument('--flann', dest='flann', action='store_true')
    parser.set_defaults(flann=False)
    parser.set_defaults(bruteforce=False)
    args = parser.parse_args()

    img1 = cv.imread(args.img_path_1,0)
    img2 = cv.imread(args.img_path_2,0)

    orb = cv.ORB_create(1000)

    kp1, des1 = orb.detectAndCompute(img1, None)
    kp2, des2 = orb.detectAndCompute(img2, None)

    if(args.bruteforce):
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)

        matches = bf.match(des1, des2)

        img3 = cv.drawMatches(img1, kp1, img2, kp2, matches, None, flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

        cv.imshow("result",img3)
        cv.waitKey(0)
    else:
        FLANN_INDEX_LSH = 6
        index_params = dict(algorithm=FLANN_INDEX_LSH,
                            table_number=6,  # 12
                            key_size=12,  # 20
                            multi_probe_level=1)  # 2
        search_params = dict(checks=50)  # or pass empty dictionary
        flann = cv.FlannBasedMatcher(index_params, search_params)
        matches = flann.knnMatch(des1, des2, k=2)

        # Need to draw only good matches, so create a mask
        matchesMask = [[0, 0] for i in range(len(matches))]
        # ratio test as per Lowe's paper
        for i, (m, n) in enumerate(matches):
            if m.distance < 0.7 * n.distance:
                matchesMask[i] = [1, 0]
        draw_params = dict(
                           singlePointColor=(255, 0, 0),
                           matchesMask=matchesMask,
                           flags=cv.DrawMatchesFlags_DEFAULT)
        img3 = cv.drawMatchesKnn(img1, kp1, img2, kp2, matches, None, **draw_params)
        cv.imshow("result",img3)
        cv.waitKey(0)