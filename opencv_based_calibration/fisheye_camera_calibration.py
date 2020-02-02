import cv2
assert cv2.__version__[0] >= '3', f'The fisheye module requires opencv version >= 3.0.0; current version is {cv2.__version__[0]}'
import numpy as np
import os
# import glob

CHECKERBOARD = (6,8)
cameraId = 2

# K = np.array([[-149390.8758233938, 1.700981741370965e+74, 4.686916689497339e+63],
#  [0, 1.623923680888597e+69, 1.775898752762208e+68],
#  [0, 0, 1]])

K = np.array([[8.1793481631740565e+02, 0., 6.0070689997785121e+02],
  [0., 8.1651774059837908e+02, 5.1784529566329593e+02], 
  [0., 0., 1.,]])

# D = np.array([0.3814478301545055,
#  -0.5882337345616587,
#  0.5294893504797997,
#  -0.1638780151720456])

D = np.array([-1.8102158829399091e+00, 9.1966147162623262e+00,
    -4.4293900343777355e-04, 1.3638377686816653e-03, 1.3303863414979364e+00, -1.4189051636354870e+00,
    8.4725535468475819e+00, 4.7911023525901033e+00])

# Intrinsics_K:
# [-149390.8758233938, 1.700981741370965e+74, 4.686916689497339e+63;
#  0, 1.623923680888597e+69, 1.775898752762208e+68;
#  0, 0, 1]

# Intrinsics_distCoeff:
# [0.3814478301545055;
#  -0.5882337345616587;
#  0.5294893504797997;
#  -0.1638780151720456]

'''
Code to capture webcam view
'''
cap = cv2.VideoCapture(cameraId)



def start():
    
    distort = True

    while(True):
        # Capture frame-by-frame
        ret, frame = cap.read()

        # Our operations on the frame come here
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Display the resulting frame
        if distort:
            cv2.imshow('frame',gray)
            print(gray)
        else:
            undistorted_img = displayUndistortedImage(gray, K, D)
            cv2.imshow('frame',undistorted_img)
            print(undistorted_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q') or key == ord('r'):
            break
        elif key == ord('s'):
            distort = not distort

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()








def calibrate():

    subpix_criteria = (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1)
    calibration_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC+cv2.fisheye.CALIB_CHECK_COND+cv2.fisheye.CALIB_FIX_SKEW
    objp = np.zeros((1, CHECKERBOARD[0]*CHECKERBOARD[1], 3), np.float32)
    objp[0,:,:2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
    _img_shape = None
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.

    # images = glob.glob('*.jpg')
    images = [f for f in os.listdir('./') if '.jpg' in f or '.png' in f]
            

    for fname in images:
        img = cv2.imread(fname)
        if _img_shape == None:
            _img_shape = img.shape[:2]
        else:
            assert _img_shape == img.shape[:2], "All images must share the same size."
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
        # Find the chess board corners
        ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, cv2.CALIB_CB_ADAPTIVE_THRESH+cv2.CALIB_CB_FAST_CHECK+cv2.CALIB_CB_NORMALIZE_IMAGE)
        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            cv2.cornerSubPix(gray,corners,(3,3),(-1,-1),subpix_criteria)
            imgpoints.append(corners)


    N_OK = len(objpoints)

    print("Found " + str(N_OK) + " valid images for calibration")

    if N_OK > 0:
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        tvecs = [np.zeros((1, 1, 3), dtype=np.float64) for i in range(N_OK)]
        rms, _, _, _, _ = \
            cv2.fisheye.calibrate(
                objpoints,
                imgpoints,
                gray.shape[::-1],
                K,
                D,
                rvecs,
                tvecs,
                calibration_flags,
                (cv2.TERM_CRITERIA_EPS+cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            )


        print("DIM=" + str(_img_shape[::-1]))
        print("K=np.array(" + str(K.tolist()) + ")")
        print("D=np.array(" + str(D.tolist()) + ")")


def displayUndistortedImage(img, K, D):
    # img = cv2.imread(img_path)
    h,w = img.shape[:2]
    h += 100
    w += 100
    _img_shape = img.shape[:2]

    # map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, _img_shape[::-1], cv2.CV_16SC2)
    map1, map2 = cv2.initUndistortRectifyMap(K,D,None,K,(w,h),5)
    
    undistorted_img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)

    # cv2.imshow("undistorted", undistorted_img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return undistorted_img


if __name__ == '__main__':
    print('Fisheye camera calibration started...')
    start()
    print('Fisheye camera calibration completed!!!')