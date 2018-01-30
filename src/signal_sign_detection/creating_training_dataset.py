import numpy as np
import cv2
import math

def find_distance_dot2dot(point1, point2):
    return math.sqrt((point1[0] - point2[0]) * (point1[0] - point2[0]) + (point1[1] - point2[1]) * (point1[1] - point2[1]))

def center(points):
    center_x = (points[0][0][0] + points[1][0][0] + points[2][0][0] + points[3][0][0])/4.0
    center_y = (points[0][0][1] + points[1][0][1] + points[2][0][1] + points[3][0][1])/4.0
    return center_x, center_y

def find_position(points):
    center_x, center_y = center(points)
    index = np.zeros(4)
    existance0 = 'no'
    existance1 = 'no'
    existance2 = 'no'
    existance3 = 'no'
    existanceall = 'no'
    for i in range(4):
        if points[i][0][0] < center_x:
            if points[i][0][1] > center_y:
                index[3] = i
                existance3 = 'yes'
            else:
                index[0] = i
                existance0 = 'yes'
        else:
            if points[i][0][1] > center_y:
                index[2] = i
                existance2 = 'yes'
            else:
                index[1] = i
                existance1 = 'yes'

    if existance0 == 'yes' and existance1 == 'yes' and existance2 == 'yes' and existance3 == 'yes':
        existanceall = 'yes'
    return existanceall, index


def find_angle(point1, point0, point2):
    y1 = point1[1] - point0[1]
    y2 = point2[1] - point0[1]
    x1 = point1[0] - point0[0]
    x2 = point2[0] - point0[0]
    angle = math.atan2(y1*x2 - x1*y2, x1*x2+y1*y2)*180/np.pi
    return abs(angle)

def distinguish_rectangular(screenCnt):
    threshold_angle = 20
    existance, index = find_position(screenCnt)
    for i in range(4):
        if find_angle(screenCnt[(i+0)%4][0], screenCnt[(i+1)%4][0], screenCnt[(i+2)%4][0]) > 90 + threshold_angle or find_angle(screenCnt[(i+0)%4][0], screenCnt[(i+1)%4][0], screenCnt[(i+2)%4][0]) < 90 - threshold_angle:
            satisfaction_angle = 'no'
            break
        satisfaction_angle = 'yes'
    if satisfaction_angle == 'yes' and existance == 'yes':
        return 'yes'



cap = cv2.VideoCapture(0)
pic_num = 0
while(True):

    ret, frame = cap.read()
    image_origin = np.copy(frame)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.bilateralFilter(gray, 11, 17, 17)
    edged = cv2.Canny(gray, 20, 100)
    kernel = np.ones((5,5),np.uint8)
    dilation = cv2.dilate(edged,kernel,iterations = 1)
    image, cnts, hierarchy = cv2.findContours(dilation.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    cnts = sorted(cnts, key = cv2.contourArea, reverse = True)[:10]
    screenCnt = None
    area_pre = 100000



    for c in cnts:
    	peri = cv2.arcLength(c, True)
    	approx = cv2.approxPolyDP(c, 0.02 * peri, True)
     	if len(approx) == 4:
    	    screenCnt = approx
            area_now = cv2.contourArea(c)
            check_rectangular = distinguish_rectangular(screenCnt)
            if check_rectangular == 'yes' and area_pre - area_now < 10000:
                cv2.drawContours(frame, [screenCnt], -1, (0, 255, 0), 3)
                for j in range(4):
                    frame = cv2.circle(frame, (screenCnt[j][0][0], screenCnt[j][0][1]), 2, (0, 0, 255), thickness=3, lineType=8, shift=0)
                _, index = find_position(screenCnt)
                pts_src = np.array([[screenCnt[int(index[0])][0][0], screenCnt[int(index[0])][0][1]], [screenCnt[int(index[1])][0][0], screenCnt[int(index[1])][0][1]], [screenCnt[int(index[2])][0][0], screenCnt[int(index[2])][0][1]], [screenCnt[int(index[3])][0][0], screenCnt[int(index[3])][0][1]]])
                pts_dst = np.array([[0, 0], [149, 0], [149, 149], [0, 149]])
                h, status = cv2.findHomography(pts_src, pts_dst)
                cv_Homography = cv2.warpPerspective(image_origin, h, (150, 150))
                cv2.imwrite("trainingimage/"+str(pic_num)+'.jpg', cv2.resize(cv_Homography,(28,28)))
                pic_num += 1

            area_pre = area_now
            screenCnt_pre = screenCnt
    cv2.imshow("edged", edged), cv2.waitKey(1)
    cv2.imshow("dilation", dilation), cv2.waitKey(1)
    cv2.imshow("frame", frame), cv2.waitKey(1)
    if 'cv_Homography' in globals():
        cv2.imshow("cv_Homography", cv_Homography), cv2.waitKey(1)
cv2.destroyAllWindows()
cap.release()
