import cv2

vid  = cv2.VideoCapture(0,cv2.CAP_DSHOW)
vid.set(3,200)
vid.set(4,200)

while(True):
    #inside infinity loop
    ret, frame = vid.read()
    cv2.imshow('frame', frame)
    print(ret)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break


vid.release()
# Destroy all the windows
cv2.destroyAllWindows() 