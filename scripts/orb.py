import cv2 as cv

ir = cv.imread("ir.png",0)
rgb = cv.imread("rgb.png",1)
orb = cv.ORB_create(1000)

kp_ir = orb.detect(ir,None)
kp_rgb = orb.detect(rgb,None)

img_ir = cv.drawKeypoints(ir,kp_ir,None,color=(0,0,255), flags=0)
img_rgb = cv.drawKeypoints(rgb,kp_rgb,None,color=(0,0,255), flags=0)

cv.imshow("ir",img_ir)
cv.imshow("rgb",img_rgb)

# cv.imwrite("ir_feature",img_ir)
# cv.imwrite("rgb_feature",img_rgb)

cv.waitKey(0)