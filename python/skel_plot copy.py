
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


data = np.array([[2.09103881, 1.27165952, 3.15332506],
 [2.18702306, 1.13109862, 3.25948327],
 [2.30931267, 1.14221703, 3.20557387],
 [2.48419121, 0.86507568, 3.31679331],
 [2.32625412, 0.72966878, 3.2062737 ],
 [2.04428246, 1.1195167,  3.3167503 ],
 [2.00953879, 0.92367406, 3.49583178],
 [2.06093462, 0.78834597, 3.34316737],
 [2.21505674, 0.68533923, 3.31385834],
 [2.3192124,  0.68048281, 3.31484991],
 [2.26319131, 0.52960803, 3.09497744],
 [2.51967513, 0.1276179,  3.49810469],
 [2.12196072, 0.68820973, 3.34363122],
 [1.95521795, 0.578667,   3.40772792],
 [1.91127413, 0.305938,   3.27654601],
 [2.10640951, 1.29673972, 3.12725301],
 [2.06651414, 1.2842772,  3.15382285],
 [2.16527354, 1.29244242, 3.15376995],
 [2.04255038, 1.25879157, 3.20563477],
 [1.93262955, 0.22804152, 3.28798198],
 [0.8492351,  1.86298544, 0.58336798],
 [1.90496106, 0.32175345, 3.22358046],
 [2.45251525, 0.17222233, 3.40488677],
 [2.47008478, 0.17227538, 3.40448639],
 [2.5477294, 0.09686418, 3.56256195]])

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# ax.scatter3D(data[:,0], data[:,1], data[:,2], 'gray')
# plt.show()

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



    # refactorDraw( Person, "leftHead", [ Nose, LEye, LEar ], "#280B68" )
    # refactorDraw( Person, "rightHead", [ Nose, REye, REar ], "#740C66" )
    # refactorDraw( Person, "torso", [ Nose, Neck, MidHip ], "#790A07" )
    # refactorDraw( Person, "leftArm", [ Neck, LShoulder, LElbow, LWrist ], "#429A1D" )
    # refactorDraw( Person, "rightArm", [ Neck, RShoulder, RElbow, RWrist ], "#818525" )
    # refactorDraw( Person, "leftLeg", [ MidHip, LHip, LKnee, LAnkle ], "#113887" )
    # refactorDraw( Person, "rightLeg", [ MidHip, RHip, RKnee, RAnkle ], "#1B9E92" )
    # refactorDraw( Person, "leftFoot", [ LHeel, LAnkle, LBigToe, LSmallToe ], "#FFFFFF" ) #"#0A0769"
    # refactorDraw( Person, "rightFoot", [ RHeel, RAnkle, RBigToe, RSmallToe ], "#FFFFFF" ) #"#0A0769"

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

if __name__ == '__main__':

    fig = plt.figure()


    ax = fig.add_subplot(111, projection='3d')
    plt.show()
    
    drawSkel(ax, data)

    



