// posefusion-server.cpp
// Subscribes to mqtt topic that contains pose estimation data and attempts
// to combine the data into an accurate 3D pose of people

#include "mqtt/client.h"
#include "mqtt/async_client.h"
#include <openpose/headers.hpp>
#include <unistd.h>
#include <chrono>

using namespace std;
using namespace cv;

const string FILEPATH       = "./media/matrices.xml";
const string SERVER_ADDR    = "oz.andrew.cmu.edu";
const string CLIENT_ID      = "posefusion";
const string TOPIC_3D       = "/topic/skeleton";
const string TOPIC_POSE     = "/lambda/+/pose";
const string EXIT_MSG       = "EXIT";

const int QOS = 0;
const double ERRTHRESHOLD = 50.0;

bool debug = false;

vector<Mat> cameraMatrices;
vector<vector<Point2d>> posePoints;

void parseArgs(int argc, char** argv);
int loadProjections();
void parseMessage(string message, vector<Point2d>& poseData);
void triangulate(vector<Point3d>& poseData3D);
double triangulatePoint(Point3d& reconstructedPoint, const vector<Point2d>& pointsOnEachCamera);
void printMat(Mat mat, int prec);

// Do nothing
class action_listener : public virtual mqtt::iaction_listener
{
    std::string name_;

    void on_failure(const mqtt::token& tok) override {}
    void on_success(const mqtt::token& tok) override {}

public:
    action_listener(const std::string& name) : name_(name) {}
};

// Callback for when message arrives.
class callback : public virtual mqtt::callback,
                 public virtual mqtt::iaction_listener {
    // Counter for the number of connection retries
    int nretry_;
    // The MQTT client
    mqtt::async_client& cli_;
    // Options to use if we need to reconnect
    mqtt::connect_options& connOpts_;
    // An action listener to display the result of actions.
    action_listener subListener_;

    void reconnect() {
        std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        try {
            cli_.connect(connOpts_, nullptr, *this);
        }
        catch (const mqtt::exception& exc) {
            std::cerr << "Error: " << exc.what() << std::endl;
            exit(1);
        }
    }

    void on_failure(const mqtt::token& tok) override {}
    void on_success(const mqtt::token& tok) override {}
    void connection_lost(const std::string& cause) override {
        std::cout << "\nConnection lost" << std::endl;
        if (!cause.empty())
            std::cout << "\tcause: " << cause << std::endl;

        std::cout << "Reconnecting..." << std::endl;
        nretry_ = 0;
        reconnect();
    }
    void delivery_complete(mqtt::delivery_token_ptr token) override {}

    void connected(const std::string& cause) override {
        cli_.subscribe(TOPIC_POSE, QOS);
    }

    // Callback for when a message arrives.
    void message_arrived(mqtt::const_message_ptr mqttMessage) override {
        int source = mqttMessage->get_topic()[8] - '1';     // Get lambda # (starting from 0)

        if (posePoints[source].empty()) {
            string msg = mqttMessage->to_string();

            parseMessage(msg, posePoints[source]);

            bool ready = true;
            for (vector<Point2d> pts : posePoints) {
                if (pts.empty())
                    ready = false;
            }

            if (ready) {
                vector<Point3d> poseData3D;
                triangulate(poseData3D);
                
                // Begin to build message to send over MQTT
                msg = "Person0,";                             // Assuming only one person
                // std::string message = std::to_string(person) + " ";  // When # of people > 1    
                for (int i = 0; i < poseData3D.size(); i++) {
                    msg += to_string(poseData3D[i].x) + "," + 
                           to_string(poseData3D[i].y) + "," + 
                           to_string(poseData3D[i].z) + ",";
                }
                msg += "on";

                // Publish once message is fully built
                cli_.publish(TOPIC_3D, msg, QOS, false);

                // Clear vectors to be ready to be filled
                for (vector<Point2d> pts : posePoints)
                    pts.clear();
            }

// for (vector<Point2d> vpts : posePoints) {
//     cout << "size: " << vpts.size() << endl;
//     for (Point2d pt : vpts) {
//         cout << pt << " ";
//     }
//     cout << "\n" << endl;
// }

        }
    }

public:
    callback(mqtt::async_client& cli, mqtt::connect_options& connOpts)
             : nretry_(0), cli_(cli), connOpts_(connOpts), subListener_("Subscription"){}
};


int main(int argc, char** argv) {
    parseArgs(argc, argv);

    int numCameras = loadProjections();
    for (int i = 0; i < numCameras; i++) {
        posePoints.push_back(vector<Point2d>(0));
    }

    mqtt::connect_options connOpts;
    connOpts.set_keep_alive_interval(20);
    connOpts.set_clean_session(true);
    mqtt::async_client client(SERVER_ADDR, CLIENT_ID);
    callback cb(client, connOpts);
    client.set_callback(cb);

    try {
        cout << "Connecting to Pose MQTT Server..." << flush;
        client.connect(connOpts, nullptr, cb);
        cout << "OK" << endl;
    }
    catch (const mqtt::exception&) {
        cerr << "\nERROR: Unable to connect to '" << SERVER_ADDR << "'" << endl;
        return 1;
    }

    cout << "\nPress q<Enter> to quit: " << endl;
    while (std::tolower(std::cin.get()) != 'q');

    try {
        cout << "\nDisconnecting from MQTT servers..." << flush;
        client.disconnect();
        cout << "OK" << endl;
    }
    catch (const mqtt::exception& exc) {
        cerr << exc.what() << endl;
        return 1;
    }

    cout << "Exiting." << endl;
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

int loadProjections() {
    Mat cameraNumber;
    FileStorage file;
    file.open(FILEPATH, FileStorage::READ);
    file["NumCameras"] >> cameraNumber;
    int numCameras = cameraNumber.at<int>(0);

    if (numCameras < 2) {
        cerr << "Not enough cameras" << endl;
        return -1;
    }

    for (int i = 0; i < numCameras; i++) {
        string header = "Projection" + to_string(i);
        Mat tempProjection;
        file[header] >> tempProjection;
        cameraMatrices.push_back(tempProjection.clone());
    }

    for (int i = 0; i < numCameras; i++) {
        if (cameraMatrices[i].empty()) {
            printf("Could not read projection %d. Remember to run from openpose folder.\n", i);
            return -1;
        }

        if (debug) {
            printf("Camera Matrix %d:\n", i);
            printMat(cameraMatrices[i], 5);
            cout << endl;
        }
    }

    return numCameras;
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

void triangulate(vector<Point3d>& poseData3D) {
    vector<double> reprojErrors;

    for (int i = 0; i < 25; i++) {
        vector<Point2d> pointsOnEachCamera;
        Point3d reconstructedPoint;
        double reprojectionError;
        int numEmpty = 0;

        for (vector<Point2d> points : posePoints) {
            if (points[i].x == 0 && points[i].y == 0)           // Check if point is visible on that camera
                numEmpty++;
            pointsOnEachCamera.push_back(points[i]);         // Push back point even if it isn't visible
        }

        if (numEmpty >= posePoints.size() - 1) {                // If only 1 or 0 cameras can see point,  
            poseData3D.push_back(Point3d(0, 0, 0));             // push back (0,0,0) point and continue
            reprojErrors.push_back(0.);
            continue;
        }

        reprojectionError = triangulatePoint(reconstructedPoint, pointsOnEachCamera);
        reprojErrors.push_back(reprojectionError);
        
        // if (reprojectionError < ERRTHRESHOLD)
            poseData3D.push_back(reconstructedPoint);
        // else
        //     poseData3D.push_back(Point3d(0, 0, 0));
    }

    if (debug) {
        for (int i = 0; i < posePoints.size(); i++) {
            printf("Client %d points:\n", i);
            for (Point2f pt : posePoints[i]) {
                printf("[%6.2f, %6.2f] ", pt.x, pt.y);
            }
            cout << endl;
        }

        cout << "Reprojection Error:" << endl;
        for (double err : reprojErrors) {
            printf("%12f ", err);
        }
        cout << endl;

        // printf("Client 1\t\t\tClient 2\t\t\t3D Points\t\t\tReproj Err\n");
        // for (int i = 0; i < 25; i++) {
        //     printf("[%8.3f,%8.3f]\t\t[%8.3f,%8.3f]\t\t[%8.3f,%8.3f,%8.3f]\t%8.3f\n",
        //         poseData1[i].x, poseData1[i].y, poseData2[i].x, poseData2[i].y,
        //         poseData3D[i].x, poseData3D[i].y, poseData3D[i].z,
        //         reprojErrs[i]);
        // }
        // cout << endl;
    }

}

double triangulatePoint(Point3d& reconstructedPoint, const vector<Point2d>& pointsOnEachCamera) {
    if (cameraMatrices.size() != pointsOnEachCamera.size())
        cerr << "Number of cameras: " << to_string(cameraMatrices.size()) <<
                " != # of points per camera: " << to_string(pointsOnEachCamera.size()) <<
                endl;

    // Create and fill A for homogenous equation system Ax = 0
    const int numCameras = (int)cameraMatrices.size();
    Mat A(numCameras*2, 4, CV_64F);

    for (int i = 0; i < numCameras; i++) {
        A.rowRange(i*2, i*2+1)   = pointsOnEachCamera[i].x * cameraMatrices[i].rowRange(2,3)
                                 - cameraMatrices[i].rowRange(0,1);
        A.rowRange(i*2+1, i*2+2) = pointsOnEachCamera[i].y * cameraMatrices[i].rowRange(2,3)
                                 - cameraMatrices[i].rowRange(1,2);
    }

    // Solve x for Ax = 0 --> SVD on A
    Mat reconstructedMat;
    SVD svd{A};
    svd.solveZ(A, reconstructedMat);
    reconstructedMat /= reconstructedMat.at<double>(3);
    reconstructedPoint = Point3d(reconstructedMat.at<double>(0),
                                 reconstructedMat.at<double>(1),
                                 reconstructedMat.at<double>(2));

    // Calculate reprojection error
    double average = 0.;

    for (int i = 0; i < numCameras; i++) {
        Mat reprojectedPoint = cameraMatrices[i] * reconstructedMat;
        reprojectedPoint /= reprojectedPoint.at<double>(2,0);
        
        average += sqrt(pow(reprojectedPoint.at<double>(0,0) - pointsOnEachCamera[i].x,2)
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
