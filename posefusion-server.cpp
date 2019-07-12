// posefusion-server.cpp
// Subscribes to mqtt topic that contains pose estimation data and attempts
// to combine the data into an accurate 3D pose of people

#include "mqtt/client.h"
#include <openpose/headers.hpp>

using namespace std;
using namespace cv;

const string FILEPATH    = "./media/matrices.xml";
const string SERVER_ADDR = "128.237.154.149:1883";
const string CLIENT1_ID  = "lambda-1-conn";
const string CLIENT2_ID  = "lambda-4-conn";
const string TOPIC1      = "lambda-1-pose";
const string TOPIC2      = "lambda-4-pose";
const string EXIT_MSG    = "EXIT";

const int QOS = 0;

void parseMessage(string message, vector<Point2d>& poseData);
double triangulate(Mat& reconstructedPoint, const vector<Mat>& cameraMatrices,
                   const vector<Point2d>& pointsOnEachCamera);
void printMat(Mat mat, int prec);

int main(int argc, char *argv[])
{
    Mat projection_L, projection_R;

    FileStorage file;
    file.open(FILEPATH, FileStorage::READ);

    file["projection_L"] >> projection_L;
    file["projection_R"] >> projection_R;

    if (projection_L.empty() || projection_R.empty()) {
        cerr << "Could not read file. Remember to run from openpose folder." << endl;
        return(0);
    }

    // printMat(projection_L, 5);
    // printMat(projection_R, 5);

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);

    cout << "Initializing MQTT Clients";
    mqtt::client client1(SERVER_ADDR, CLIENT1_ID);
    mqtt::client client2(SERVER_ADDR, CLIENT2_ID);
    cout << "OK" << endl;

    try {
        cout << "Connecting to MQTT Servers..." << flush;
        client1.connect(connOpts);
        client1.subscribe(TOPIC1, QOS);
        client2.connect(connOpts);
        client2.subscribe(TOPIC2, QOS);
        cout << "OK" << endl;

        while (1) {
            shared_ptr<const mqtt::message> mqttMsg;
        
            if (client1.try_consume_message(&mqttMsg)) {
                string msg = mqttMsg->to_string();
                vector<Point2d> poseData;

                if(msg.compare(EXIT_MSG)){
                    parseMessage(msg, poseData);

                    cout << "Client 1" << endl;
                    for (Point2d pt2d : poseData)
                        cout << pt2d << endl;
                    cout << endl;
                }
                else
                    break;
            }
            else {
                cerr << "No reponse from client 1" << endl;
                return 1;
            }

            if (client2.try_consume_message(&mqttMsg)) {
                string msg = mqttMsg->to_string();
                vector<Point2d> poseData;

                if(msg.compare(EXIT_MSG)){
                    parseMessage(msg, poseData);

                    cout << "Client 2" << endl;
                    for (Point2d pt2d : poseData)
                        cout << pt2d << endl;
                    cout << endl;
                }
                else
                    break;
            }
            else {
                cerr << "No reponse from client 2" << endl;
                return 1;
            }

        }

        cout << "Disconnecting MQTT...";
        client1.disconnect();
        client2.disconnect();
        cout << "OK" << endl;
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    cout << "Exiting" << endl;
    return 0;
}

void parseMessage(string message, vector<Point2d>& poseData) {
    // cout << message << endl;

    stringstream msg_stream(message);
    string temp;

    int person_number;
    msg_stream >> person_number;

    if (person_number != 0) {
        cout << "WARNING! More than 1 person on screen, ignoring person " << person_number << endl;
        return;
    }

    for (int i = 0; i < 25; i++) {
        double x, y;
        
        msg_stream >> x;
        msg_stream >> y;

        poseData.push_back(Point2d(x, y));
    }
}

double triangulate(Mat& reconstructedPoint, const vector<Mat>& cameraMatrices,
                   const vector<Point2d>& pointsOnEachCamera)
{
    if (cameraMatrices.size() != pointsOnEachCamera.size())
        cerr << "Number of cameras: " << to_string(cameraMatrices.size()) <<
                " != # of points per camera: " << to_string(pointsOnEachCamera.size()) <<
                endl;

    // Create and fill A for homogenous equation system Ax = 0
    const int numCameras = (int)cameraMatrices.size();
    Mat A(numCameras*2, 4, CV_64F);

    for (int i = 0; i < numCameras; i++)
    {
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

    for (int i = 0; i < cameraMatrices.size(); i++)
    {
        Mat reprojectedPoint = cameraMatrices[i] * reconstructedPoint;
        reprojectedPoint /= reprojectedPoint.at<double>(2,0);
        
        average = sqrt(pow(reprojectedPoint.at<double>(0,0) - pointsOnEachCamera[i].x,2)
                          + pow(reprojectedPoint.at<double>(1,0) - pointsOnEachCamera[i].y,2));
    }

    return average / numCameras;
}

void printMat(Mat mat, int prec)
{      
    for(int i=0; i<mat.size().height; i++)
    {
        cout << "[";
        for(int j=0; j<mat.size().width; j++)
        {
            printf("%10f", mat.at<double>(i,j));
            // cout << setprecision(prec) << mat.at<double>(i,j);
            if(j != mat.size().width-1)
                cout << ", ";
            else
                cout << "]" << endl; 
        }
    }
}