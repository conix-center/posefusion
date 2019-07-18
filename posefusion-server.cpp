// posefusion-server.cpp
// Subscribes to mqtt topic that contains pose estimation data and attempts
// to combine the data into an accurate 3D pose of people

#include "mqtt/client.h"
#include <openpose/headers.hpp>
#include <unistd.h>
#include <thread>
#include <chrono>

using namespace std;
using namespace cv;

const string FILEPATH       = "./media/matrices.xml";
const string SERVER_ADDR    = "192.168.1.191";
const string VISUAL_ADDR    = "oz.andrew.cmu.edu";
const string SUB_CLIENT1_ID = "lambda-1-conn";
const string SUB_CLIENT2_ID = "lambda-4-conn";
const string PUB_CLIENT_ID  = "3d-pose";
const string TOPIC_CAM1     = "lambda-1-pose";
const string TOPIC_CAM2     = "lambda-4-pose";
// const string TOPIC_3D       = "3d_pose";
const string TOPIC_3D       = "/topic/skeleton";
const string EXIT_MSG       = "EXIT";

const int QOS = 0;
const double ERRTHRESHOLD = 50.0;

bool debug = false;

void parseArgs(int argc, char** argv);
bool getPoseData(vector<Point2d>& poseData1, vector<Point2d>& poseData2);
void parseMessage(string message, vector<Point2d>& poseData);
void triangulate(const vector<Point2d>& poseData1, const vector<Point2d>& poseData2,
                 const vector<Mat>& cameraMatrices, vector<Point3d>& poseData3D);
double triangulatePoint(Mat& reconstructedPoint, const vector<Mat>& cameraMatrices,
                        const vector<Point2d>& pointsOnEachCamera);
void printMat(Mat mat, int prec);

int main(int argc, char** argv) {
    parseArgs(argc, argv);

    vector<Mat> cameraMatrices;
    Mat temp;
    FileStorage file;
    file.open(FILEPATH, FileStorage::READ);
    file["projection_L"] >> temp;
    cameraMatrices.push_back(temp.clone());
    file["projection_R"] >> temp;
    cameraMatrices.push_back(temp.clone());

    for (int i = 0; i < cameraMatrices.size(); i++) {
        if (cameraMatrices[i].empty()) {
            cerr << "Could not read file. Remember to run from openpose folder." << endl;
            return 1;
        }

        if (debug) {
            printf("Camera Matrix %d:\n", i);
            printMat(cameraMatrices[i], 3);
            cout << endl;
        }
    }

    cout << "Initializing MQTT Clients..." << flush;
    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    mqtt::client sub_client1(SERVER_ADDR, SUB_CLIENT1_ID);
    mqtt::client sub_client2(SERVER_ADDR, SUB_CLIENT2_ID);
    mqtt::client pub_client(VISUAL_ADDR, PUB_CLIENT_ID);
    cout << "OK" << endl;

    try {
        cout << "Connecting to MQTT Servers..." << flush;
        sub_client1.connect(connOpts);
        sub_client1.subscribe(TOPIC_CAM1, QOS);
        sub_client2.connect(connOpts);
        sub_client2.subscribe(TOPIC_CAM2, QOS);
        pub_client.connect(connOpts);
        cout << "OK\n" << endl;

        while (1) {
            vector<Point2d> poseData1, poseData2;
            vector<Point3d> poseData3D;

            shared_ptr<const mqtt::message> mqttMsg1, mqttMsg2;

            if (sub_client1.try_consume_message_for(&mqttMsg1, chrono::seconds(1))) {
                string msg = mqttMsg1->to_string();

                if(msg.compare(EXIT_MSG)){
                    parseMessage(msg, poseData1);
                }
                else
                    break;
            }
            if (sub_client2.try_consume_message_for(&mqttMsg2, chrono::seconds(1))) {
                string msg = mqttMsg2->to_string();

                if(msg.compare(EXIT_MSG)) {
                    parseMessage(msg, poseData2);
                }
                else
                    break;
            }

            if (poseData1.size() == 0 || poseData2.size() == 0) {
                if (debug) {
                    cout << "Failed to read from client." << endl;
                    printf("Client 1: %d - Client 2:  %d\n\n",
                        (int)poseData1.size(), (int)poseData2.size());
                }
                continue;
            }

            triangulate(poseData1, poseData2, cameraMatrices, poseData3D);

            // if (debug) {
            //     printf("Client 1\t\t|\tClient 2\t\t|\t3D Points\n");
            //     for (int i = 0; i < 25; i++) {
            //         printf("[%8.3f,%8.3f]\t|\t[%8.3f,%8.3f]\t|\t[%8.3f,%8.3f,%8.3f]\n",
            //             poseData1[i].x, poseData1[i].y, poseData2[i].x, poseData2[i].y,
            //             poseData3D[i].x, poseData3D[i].y, poseData3D[i].z);
            //     }
            //     cout << endl;
            // }

            // Begin to build message to send over MQTT
            string message = "Person0,";                             // Assuming only one person
            // std::string message = std::to_string(person) + " ";  // When # of people > 1    
            for (int i = 0; i < poseData3D.size(); i++) {
                message += to_string(poseData3D[i].x) + "," + 
                           to_string(poseData3D[i].y) + "," + 
                           to_string(poseData3D[i].z) + ",";
            }
	    message += "on";
            // Publish once message is fully built
            pub_client.publish(mqtt::message(TOPIC_3D, message, QOS, false));

            if (debug) {
                cout << message << endl;
            }
        }

        cout << "Disconnecting MQTT...";
        sub_client1.disconnect();
        sub_client2.disconnect();
        cout << "OK" << endl;
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    cout << "Exiting" << endl;
    return 0;
}

void parseArgs(int argc, char** argv) {
    int c;
    while ((c = getopt(argc, argv, "dh")) != -1) {
        switch(c) {
            // Prints all logs for debugging
            case 'd':
                debug = true;
                break;
            case 'h':
                cout << "\nPose Fusion Server" << endl;
                cout << "Options:" << endl;
                cout << " -h (help)    Display command line options" << endl;
                cout << " -d (debug)   Prints logging information for debugging" << endl;
                exit(0);
        }
    }
}

void parseMessage(string message, vector<Point2d>& poseData) {
    stringstream msg_stream(message);
    string temp;

    int person_number;
    msg_stream >> person_number;

    for (int i = 0; i < 25; i++) {
        double x, y;
        
        msg_stream >> x;
        msg_stream >> y;

        poseData.push_back(Point2d(x, y));
    }
}

void triangulate(const vector<Point2d>& poseData1, const vector<Point2d>& poseData2,
                 const vector<Mat>& cameraMatrices, vector<Point3d>& poseData3D) {
    vector<double> reprojErrs;
    double temp;

    for (int i = 0; i < 25; i++) {
        if (poseData1[i].x == 0 && poseData1[i].y == 0 ||
            poseData2[i].x == 0 && poseData2[i].y) {

            poseData3D.push_back(Point3d(0, 0, 0));
            reprojErrs.push_back(0.);
            continue;
        }

        vector<Point2d> pointsOnEachCamera;
        Mat reconstructedPoint;
        pointsOnEachCamera.push_back(poseData1[i]);
        pointsOnEachCamera.push_back(poseData2[i]);

        if (temp = triangulatePoint(reconstructedPoint, cameraMatrices, pointsOnEachCamera) < ERRTHRESHOLD) {
            poseData3D.push_back(Point3d(reconstructedPoint.at<double>(0),
                                         reconstructedPoint.at<double>(1),
                                         reconstructedPoint.at<double>(2)));
        }
        else
            poseData3D.push_back(Point3d(0, 0, 0));

        reprojErrs.push_back(temp);
    }

// printf("Client 1\t\t|\tClient 2\t\t|\t3D Points\n");
// for (int i = 0; i < 25; i++) {
//     printf("[%8.3f,%8.3f]\t|\t[%8.3f,%8.3f]\t|\t[%8.3f,%8.3f,%8.3f]\t%8.3f\n",
//         poseData1[i].x, poseData1[i].y, poseData2[i].x, poseData2[i].y,
//         poseData3D[i].x, poseData3D[i].y, poseData3D[i].z,
//         reprojErrs[i]);
// }
// cout << endl;

}

double triangulatePoint(Mat& reconstructedPoint, const vector<Mat>& cameraMatrices,
                        const vector<Point2d>& pointsOnEachCamera) {
    if (cameraMatrices.size() != pointsOnEachCamera.size())
        cerr << "Number of cameras: " << to_string(cameraMatrices.size()) <<
                " != # of points per camera: " << to_string(pointsOnEachCamera.size()) <<
                endl;

    // Create and fill A for homogenous equation system Ax = 0
    const int numCameras = (int)cameraMatrices.size();
    Mat A(numCameras*2, 4, CV_64F);

    for (int i = 0; i < numCameras; i++) {
        A.rowRange(i*2, i*2+1) = pointsOnEachCamera[i].x*cameraMatrices[i].rowRange(2,3)
                               - cameraMatrices[i].rowRange(0,1);
        A.rowRange(i*2+1, i*2+2) = pointsOnEachCamera[i].y*cameraMatrices[i].rowRange(2,3)
                                 - cameraMatrices[i].rowRange(1,2);
    }

    // Solve x for Ax = 0 --> SVD on A
    SVD svd{A};
    svd.solveZ(A,reconstructedPoint);
    reconstructedPoint /= reconstructedPoint.at<double>(3);


    // Calculate reprojection error
    double average = 0.;

    for (int i = 0; i < cameraMatrices.size(); i++) {
        Mat reprojectedPoint = cameraMatrices[i] * reconstructedPoint;
        reprojectedPoint /= reprojectedPoint.at<double>(2,0);
        
        average = sqrt(pow(reprojectedPoint.at<double>(0,0) - pointsOnEachCamera[i].x,2)
                          + pow(reprojectedPoint.at<double>(1,0) - pointsOnEachCamera[i].y,2));
    }

    return average / numCameras;
}

void printMat(Mat mat, int prec) {      
    for(int i = 0; i < mat.size().height; i++) {
        cout << "[";
        for(int j = 0; j < mat.size().width; j++) {
            printf("%10.*f", prec, mat.at<double>(i, j));
            // cout << setprecision(prec) << mat.at<double>(i, j);
            if(j != mat.size().width - 1)
                cout << ", ";
            else
                cout << "]" << endl; 
        }
    }
}
