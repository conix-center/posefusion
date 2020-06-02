import numpy as np
import cv2

import multiprocessing

# ls -ltrh /dev/video*
# ./build/examples/openpose/openpose.bin --disable_multi_thread --video posefusion/outputLambda2.avi --render_pose 0 --face --face_render 1 --hand --hand_render 1 -write_json ./posefusion/grgTmp/
# ./build/examples/openpose/openpose.bin --disable_multi_thread --video posefusion/outputLambda2.avi --render_pose 0 --face --face_render 1 --hand --hand_render 1 -write_json ./posefusion/grgTmp/
# ./build/examples/openpose/openpose.bin --disable_multi_thread --camera 2
#  ./build/posefusion/posefusion-client2 2 --camera 2 --disable_multi_thread

def captureVideo(cameraIdx = 0):

    print(f'Camera Idx is {cameraIdx}')
    cap = cv2.VideoCapture(cameraIdx)

    # Set Resolution
    # cap.set(3, 1280)
    # cap.set(4, 720)
    cap.set(3, 640)
    cap.set(4, 480)

    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(f'outputLambda{cameraIdx}.avi',fourcc, 20.0, (640,480))

    while(cap.isOpened()):
        ret, frame = cap.read()
        if ret==True:
            # frame = cv2.flip(frame,0)

            # write the flipped frame
            out.write(frame)

            # print(f'Processing camera Idx is {cameraIdx}')

            cv2.imshow(f'frame{cameraIdx}',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        else:
            break

    # Release everything if job is finished
    cap.release()
    out.release()
    cv2.destroyAllWindows()

    print(f'Closed camera Idx is {cameraIdx}')

if __name__ == "__main__":

    cameraIdx = [0, 2] #, 0]

    jobs = []
    for i in cameraIdx:
        p = multiprocessing.Process(target=captureVideo, args=(i,))
        jobs.append(p)
        p.start()

    for p in jobs:
        p.join()
