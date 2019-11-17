import cv2
import numpy as np

# Read matrices.xml
# projs = read_matrices(MATRICES_PATH)

def camera2(E):
    U,S,V = np.linalg.svd(E)
    m = S[:2].mean()
    E = U.dot(np.array([[m,0,0], [0,m,0], [0,0,0]])).dot(V)
    U,S,V = np.linalg.svd(E)
    W = np.array([[0,-1,0], [1,0,0], [0,0,1]])

    if np.linalg.det(U.dot(W).dot(V))<0:
        W *= -1

    M2s = np.zeros([3,4,4])
    M2s[:,:,0] = np.concatenate([U.dot(W).dot(V), U[:,2].reshape([-1, 1])/abs(U[:,2]).max()], axis=1)
    M2s[:,:,1] = np.concatenate([U.dot(W).dot(V), -U[:,2].reshape([-1, 1])/abs(U[:,2]).max()], axis=1)
    M2s[:,:,2] = np.concatenate([U.dot(W.T).dot(V), U[:,2].reshape([-1, 1])/abs(U[:,2]).max()], axis=1)
    M2s[:,:,3] = np.concatenate([U.dot(W.T).dot(V), -U[:,2].reshape([-1, 1])/abs(U[:,2]).max()], axis=1)
    return M2s

# Read projection matrices from matrices.xml into a (3,4,NUMCAMERAS) numpy array
def read_matrices(file_path):
    cv_file = cv2.FileStorage(file_path, cv2.FILE_STORAGE_READ)
    numCameras = int(cv_file.getNode("NumCameras").mat())
    projection_matrices = np.empty((numCameras, 3, 4))
    for camera in range(0, numCameras):
        matrix = cv_file.getNode("Projection" + str(camera)).mat()
        projection_matrices[camera] = matrix
    return projection_matrices

def get_projs_matrices(pts1_calib, pts2_calib, K1, K2):
    # Obtain array
    pts1_calib = np.vstack(pts1_calib)
    pts2_calib = np.vstack(pts2_calib)

    num_points = min(pts1_calib.shape[0], pts2_calib.shape[0])

    F = cv2.findFundamentalMat(pts1_calib[0:num_points], pts2_calib[0:num_points])
    F = F[0]

    print("Found F", F)

    # Obtain F
    E = K2.T @ F @ K1

    # Obtain C1 and C2
    M2_four = camera2(E)
    # M2 = M2_four[:,:,0]
    M1 = np.array([[1, 0, 0, 0],
                [0, 1, 0, 0],
                [0, 0, 1, 0]])

    # Projection matrix P = K * [R|t]
    C1 = np.dot(K1,M1)
    
    correct_M2 = np.empty((3, 4))
    for i in range(M2_four.shape[2]):
        C2 = np.dot(K2,M2_four[:,:,i])

        # Triangulate points
        # new_error, error_mat, pts3D = triangulateTwoBodies(C1, C2, pts1_calib[500], pts2_calib[500])

        # For each NEED FIX TO GET RANDOM VALUE IN A SET NOT A SET ONE LIKE 200
        pts4D = cv2.triangulatePoints(C1, C2, pts1_calib[200], pts2_calib[200]).T

        # Convert from homogeneous coordinates to 3D
        pts3D = pts4D[:, :3]/np.repeat(pts4D[:, 3], 3).reshape(-1, 3)

        # Check z coordinates in the camera frame
        point = np.array([pts3D[0, 0], pts3D[0, 1], pts3D[0, 2], 1])
        test = np.dot(M2_four[:,:,i], point)[-1]
        test2 = np.dot(M1, point)[-1]

        # If z of both projectins (M1 and M2) then it is the correct M2
        if ((test >= 0) and (test2 >= 0)):
            correct_M2 = M2_four[:,:,i]
            break;

    M2 = correct_M2

    # Test
    # M2 = M2_four[:,:,2]
    # print(M2_four[:,:,0][:,-1])
    # print(M2_four[:,:,1][:,-1])
    # print(M2_four[:,:,2][:,-1])
    # print(M2_four[:,:,3][:,-1])


    C2 = np.dot(K2,M2)

    print("Found C2")
    return C1, C2, M1, M2