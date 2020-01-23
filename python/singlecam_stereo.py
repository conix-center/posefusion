import numpy as np
import cv2

pts1 = np.array([[558.905518, 353.257507], [562.789795, 396.207245], [531.408203, 396.235535], [519.691528, 447.183441], [505.987549, 502.061829], [596.072205, 394.256287], [607.809021, 445.179199], [619.596619, 496.169098], [566.648376, 503.988556], [541.228394, 503.994476], [535.372925, 584.348511], [523.616577, 664.707275], [588.158752, 504.007996], [590.277161, 586.24646 ], [601.886169, 664.619446], [554.963562, 351.171143], [568.673828, 351.121887], [543.266174, 353.180542], [574.602478, 353.04953 ], [602.01886 , 690.102417], [615.635681, 682.327698], [598.023071, 668.559143], [521.554871, 695.979797], [507.997925, 692.056824], [525.608398, 670.55249 ]])
pts2 = np.array([[370.798889, 357.045502], [374.688568, 400.262634], [337.446198, 402.194916], [325.694885, 462.911102], [310.057495, 523.627258], [413.839111, 400.129456], [429.43457 , 457.007446], [433.45636 , 511.864838], [376.684387, 523.53302 ], [351.240387, 523.526855], [351.248962, 609.820435], [353.160583, 692.045044], [400.171906, 523.551392], [404.104401, 605.908997], [413.916016, 682.239502], [366.809021, 351.231812], [382.428772, 351.215424], [353.154785, 355.096466], [392.290924, 353.260193], [411.852173, 695.969971], [419.728699, 695.889404], [413.919739, 686.199646], [353.202942, 699.946228], [341.389709, 699.879578], [359.026001, 697.885803]])


baseline = 1
camera_ht = 1.8

# Intrinsic matrix calculated using openpose
K1 = K2 = np.array([ 
                    [753.6412388549141, 0., 627.6217759928232],
                    [0., 753.892100491527, 355.1896481269448],
                    [0., 0., 1.]
                    ], dtype='float64' 
                    )

R1 = R2 = np.array(
                    [[-1,0,0],
                        [0,-1,0],
                        [0,0,1]],
                        dtype='float64'
                    )
T1 = np.array([0, camera_ht, 0]).T
T2 = np.array([baseline, camera_ht, 0]).T

m_matrices0 = np.column_stack((R1, T1))
m_matrices1= np.column_stack((R2, T2))

#Note : Projection matrix is transformation from 3D coordinate to 2D coordinate system
cam1 = np.dot(K1, m_matrices0)
cam2 = np.dot(K2, m_matrices1)



'''
Q3.2: Triangulate a set of 2D coordinates in the image to a set of 3D points.
    Input:  C1, the 3x4 camera matrix
            pts1, the Nx2 matrix with the 2D image coordinates per row
            C2, the 3x4 camera matrix
            pts2, the Nx2 matrix with the 2D image coordinates per row
    Output: P, the Nx3 matrix with the corresponding 3D points per row
            err, the reprojection error.
'''
def triangulate(C1, C2, pts1, pts2):
    # Replace pass by your implementation
    P = []
    err = 0
   
    for [x1, y1], [x2, y2] in zip(pts1, pts2):
        A = np.concatenate([
                 [y1*C1[2,:] - C1[1,:]]
                ,[C1[0,:] - x1*C1[2,:]]
                ,[y2*C2[2,:] - C2[1,:]]  
                ,[C2[0,:] - x2*C2[2,:]] 
        ])

        u,s,v = np.linalg.svd(A)
        p = v[-1]
        #p = (p/p[-1])

        p1_e = np.matmul(C1,p)
        p2_e = np.matmul(C2,p)
        p1_e = (p1_e/p1_e[-1])[:-1]
        p2_e = (p2_e/p2_e[-1])[:-1]
        
        
        # P.append(p[:-1])
        P.append(p)
    
        err += (np.linalg.norm([x1, y1]-p1_e)**2 + np.linalg.norm([x2, y2]-p2_e)**2)
    print(f'GRG Reprojection err : {err}')   
    return np.array(P), err


pts4D, reprojection_error = triangulate(cam1, cam2, pts1, pts2)
pts3D = pts4D[:, :3]/np.repeat(pts4D[:, 3], 3).reshape(-1, 3)

print(f'pts3D is : {pts3D}')
print(f'reprojection error is : {reprojection_error}')

# #pts3D convered to array
# data = np.array([[-0.36523693,  1.79980623, -4.00613491], [-0.34458465,  2.02875364, -4.00618081], [-0.49584682,  2.02688218, -3.88472047], [-0.55483389,  2.31398274, -3.87908215], [-0.61770946,  2.60244432, -3.83558038], [-0.17298607,  2.0303896 , -4.13470764], [-0.11051006,  2.3370799 , -4.22111817], [-0.04221266,  2.59813864, -4.04204286], [-0.31897148,  2.63268702, -3.95743895], [-0.45245289,  2.63261581, -3.95711129], [-0.49671577,  3.10810661, -4.07518918], [-0.60422296,  3.68573902, -4.39723786], [-0.20815007,  2.64150264, -3.99885908], [-0.19874207,  3.09033184, -4.0372718 ], [-0.13554613,  3.48907193, -4.0006965 ], [-0.38616241,  1.77881081, -4.00543734], [-0.31650741,  1.7784174 , -4.04650313], [-0.4436982 ,  1.79447512, -3.96412863], [-0.2908168 ,  1.7888427 , -4.13380848], [-0.13448431,  3.57559662, -3.96211404], [-0.06048883,  3.50194461, -3.84216065], [-0.15932684,  3.54587495, -4.08454844], [-0.62990157,  3.83515252, -4.4760605 ], [-0.71745346,  3.8438386 , -4.52138302], [-0.6063095 ,  3.7642258 , -4.49894635]])

######## Plotting ###############
from matplotlib import pyplot as plt
# import numpy as np
import mpl_toolkits.mplot3d.axes3d as p3
from matplotlib import animation


def plotPoint(ax, data):
    ax.scatter3D(data[:,0], data[:,1], data[:,2], 'blue')


def plotline(ax, data, color):
    data = np.array(data)
    ax.plot3D(data[:,0], data[:,1], data[:,2], 'grey')


def drawSkel(ax, data):
    Nose = data[0,:]
    Neck = data[1,:]
    RShoulder = data[2,:]
    RElbow = data[3,:]
    RWrist = data[4,:]
    LShoulder = data[5,:]
    LElbow = data[6,:]
    LWrist = data[7,:]
    MidHip = data[8,:]
    RHip = data[9,:]
    RKnee = data[10,:]
    RAnkle = data[11,:]
    LHip = data[12,:]
    LKnee = data[13,:]
    LAnkle = data[14,:]
    REye = data[15,:]
    LEye = data[16,:]
    REar = data[17,:]
    LEar = data[18,:]
    LBigToe = data[19,:]
    LSmallToe = data[20,:]
    LHeel = data[21,:]
    RBigToe = data[22,:]
    RSmallToe = data[23,:]
    RHeel = data[24,:]

    plotPoint(ax, data)

    plotline(ax, [ Nose, LEye, LEar ], "#280B68")
    plotline(ax, [ Nose, REye, REar ], "#740C66" )
    plotline(ax, [ Nose, Neck, MidHip ], "#790A07" )
    plotline(ax, [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
    plotline(ax, [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
    plotline(ax, [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
    plotline(ax, [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
    plotline(ax, [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
    plotline(ax, [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"



fig = plt.figure()
ax = p3.Axes3D(fig)

N = 100

# Setting the axes properties
#ax.set_xlim3d([-1.0, 1.0])
ax.set_xlabel('X')

#ax.set_ylim3d([-1.0, 1.0])
ax.set_ylabel('Y')

#ax.set_zlim3d([0.0, 10.0])
ax.set_zlabel('Z')

drawSkel(ax, pts3D)

plt.show()