## From https://dev.to/simarpreetsingh019/detecting-geometrical-shapes-in-an-image-using-opencv-4g72

import numpy as np
import cv2

def rectangle_detector(img):
    imgGry = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret , thrash = cv2.threshold(imgGry, 120 , 255, cv2.CHAIN_APPROX_NONE)#cv2.THRESH_BINARY)

    contours, hierarchy = cv2.findContours(thrash, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[-2:]
    out_approx = None
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01* cv2.arcLength(contour, True), True)

        cv2.drawContours(thrash, [approx], 0, (0, 0, 0), 5)

        x = approx.ravel()[0]
        y = approx.ravel()[1] - 5
        if len(approx) == 4 :
            x, y , w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            if (w > 500) and (h > 300) and (1.5 < aspectRatio < 1.8):
                #return approx, thrash
                out_approx = approx
    
    return out_approx, thrash


if __name__ == '__main__':

    cap = cv2.VideoCapture(0)
    while(True):
        # Capture frame-by-frame
        ret, img = cap.read()

        approx, thrash = rectangle_detector(img)

        if approx is not None:
            cv2.drawContours(img, [approx], 0, (0, 255, 0), 5)
            x, y , w, h = cv2.boundingRect(approx)
            aspectRatio = float(w)/h
            print(aspectRatio, x, y, w, h, approx)
            if aspectRatio >= 0.95 and aspectRatio < 1.05:
                cv2.putText(img, "square", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))

            else:
                cv2.putText(img, "rectangle", (x, y), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))

        cv2.imshow('shapes', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
