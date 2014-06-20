import cv2
import ContourClassifier as ccMod
from ContourClassifier import ContourClassifier

FRAME_WIDTH = 320
FRAME_HEIGHT = 240

CAPTURE_RADIUS = 28  #30
MAX_WALL_PERCENTAGE = 0.9 # when wall takes up this much of the frame then starrt pinging

cv2.namedWindow("webcam1")
#cv2.namedWindow("ROI")
cv2.namedWindow("Threshold")
vc = cv2.VideoCapture(0)
vc.set(3,FRAME_WIDTH)
vc.set(4,FRAME_HEIGHT)

minArea = 0
maxEmptyFrames = 10
threshold = 100
cc = ContourClassifier(FRAME_WIDTH, FRAME_HEIGHT)
#rectROI = (0, 100, 320, 240)
cc.resetRoi()
cc.setGrayThreshold(threshold)
cc.setThresholdType(ccMod.THRESHOLD_TYPE_NORMAL)
cc.setImgRotated(True)

if vc.isOpened():
    rval, frame = vc.read()

else:
    rval = False

emptyFrameCount = 0

while rval:
    rval, frame = vc.read()
    #frame = cc.rotate180(frame)
    cv2.imshow("webcam1", frame)

    cc.processColorImg = False
    clist = cc.getCircles(frame)

    if cc.threshImg is not None:
        cv2.imshow("Threshold", cc.threshImg)

    if clist is not None:
        print '####################################'
        print 'nr circles: ' + str(len(clist))
        for idx in range(len(clist)):
            print 'Circle %d:' % (idx + 1)
            print '  x center: %d' % clist[idx][0]
            print '  y center: %d' % clist[idx][1]
            print '  radius  : %d\n' % clist[idx][2]

        print '####################################\n'
                
            #cv2.imshow("webcam1", frame)

    else:
        print 'no circles found!'                
            
    #cv2.imshow("webcam1", frame)
    

    rval, frame = vc.read()
    
    key = cv2.waitKey(100)
    if key == 27:
        break
    elif key == -1:
        continue
    elif key == 2490368:
        print 'Up arrow pressed'
        threshold += 5
        if threshold < 255:
            cc.setGrayThreshold(threshold)
    elif key == 2621440:
        print 'Down arrow pressed'
        threshold -= 5
        if threshold > 0:
            cc.setGrayThreshold(threshold)
    elif key == 2424832:
        print 'Left arrow pressed'
    elif key == 2555904:
        print 'Right arrow pressed'
        

cv2.destroyAllWindows()

