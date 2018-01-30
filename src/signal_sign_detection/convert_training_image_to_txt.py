import cv2
import numpy as np
import os

def convert():
    normalizing = 'yes'
    label = 7
    f = open('TrainingData.txt', 'a')
    for file_type in ['7WARNING']:
        for img in os.listdir(file_type):
            try:
                current_image_path = str(file_type) + '/' + str(img)
                image = cv2.imread(current_image_path)
                image_gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
                np_image = np.reshape(image_gray,784)
                if normalizing == 'yes':
                    np_image = np_image/np.mean(np_image)
                for i in range(0,784):
                    data = "%d" % np_image[i]
                    f.write(data)
                    f.write(",")
		for i in range(10):
                    if i == label:
                        data = "%d" % 1
                    else:
                        data = "%d" % 0
                    f.write(data)
                    if i != 9:
                        f.write(",")
                f.write("\n")
            except Exception as e:
                print(str(e))
    f.close()

convert()



# 0 : RIGHT
# 1 : LEFT
# 2 : TOP
# 3 : BOTTOM
# 4 : UNDER20
# 5 : UNDER50
# 6 : STOP
# 7 : WARNING
