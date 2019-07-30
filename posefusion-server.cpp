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
double triangulatePoint(Point3d& reconstructedPoint, const vector<Point2d>& pointsOnEachCamera,
    const vector<Mat>& cameraMats);
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

    // Callback for when a message arrives. Processes message then waits for queue to fill up
    // then performs triangulation once all camera frames have been received.
    void message_arrived(mqtt::const_message_ptr mqttMessage) override {
        int source = mqttMessage->get_topic()[8] - '1';     // Get lambda # (starting from 0)

        if (posePoints[source].empty()) {
            string msg = mqttMessage->to_string();

            parseMessage(msg, posePoints[source]);


            bool ready = true;
            // Checks to make sure all points are filled, so will not work if one or more
            // KNOWN cameras are completely covered 
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
                for (int i = 0; i < posePoints.size(); i++)
                    posePoints[i].clear();
            }
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

    // Spins until user presses 'q' to stop
    cout << "\nPress q<Enter> to quit: \n" << endl;
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

// Function to parse command line arguments
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

// Loads the camera projection matrices from a matrices.xml file that 
// should be filled previously from the camera calibration step.
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

// Parses the mqtt message string and stores the points in the 
// corresponding Point2d vector
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

// Performs the triangulation of the pose points and returns the points
// in 3D. Currently does not do any filtering or checking to see if the points
// are too far from the expected human pose length. Nor does it check if 
// the reconstructed point's reprojection error is too high. In TODO list
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

        // Perform initial triangulation of single point
        reprojectionError = triangulatePoint(reconstructedPoint, pointsOnEachCamera, cameraMatrices);

        // If more than 2 cameras, perform basic RANSAC by removing a single camera and performing
        // triangulation on the subset of cameras and points. Use the reconstructed point with the
        // least reprojection error. Currently only removes 1 camera. TODO: Continue iterations such
        // that more cameras are removed if it improves reprojection error.
        if (cameraMatrices.size() > 2) {
            bool removedCamera = false;
            double bestReprojectionError = reprojectionError;
            Point3d bestReconstructedPoint;

            for (int i = 0; i < cameraMatrices.size(); i++) {
                vector<Mat> newCameraMatrices = cameraMatrices;
                vector<Point2d> newPointsOnEachCamera = pointsOnEachCamera;
                Point3d newReconstructedPoint;

                // Remove camera i to create new subset of camera matrices and pose points
                newCameraMatrices.erase(newCameraMatrices.begin() + i);
                newPointsOnEachCamera.erase(newPointsOnEachCamera.begin() + i);

                double newReprojectionError = triangulatePoint(newReconstructedPoint,
                    newPointsOnEachCamera, newCameraMatrices);

                // New reprojection error should be significantly better than previous error
                if (newReprojectionError < bestReprojectionError 
                    && newReprojectionError < 0.9 * reprojectionError)
                {
                    bestReprojectionError = newReprojectionError;
                    bestReconstructedPoint = newReconstructedPoint;
                    removedCamera = true;
                }
            }

            if (removedCamera)
            {
                reprojectionError = bestReprojectionError;
                reconstructedPoint = bestReconstructedPoint;
            }
        }

        reprojErrors.push_back(reprojectionError);
        
        // TODO: Filtering. Get distances of certain point to point correspondences and 
        // check to see if length is acceptable/realistic for a human. For instance if the
        // neck is a meter long, you know something is wrong.

        // TODO: Establish an error threshold in order to remove points that are too 
        // far from the expected reprojection error 
        // if (reprojectionError < ERRTHRESHOLD)
            poseData3D.push_back(reconstructedPoint);
        // else
        //     poseData3D.push_back(Point3d(0, 0, 0));
    }

    if (debug) {
        for (int i = 0; i < posePoints.size(); i++) {
            printf("Client %d points:\n", i);
            for (Point2f pt : posePoints[i]) {
                printf("[% 8.2f,% 8.2f] ", pt.x, pt.y);
            }
            cout << endl;
        }

        cout << "3D points:" << endl;
        for (Point3d pt3d : poseData3D)
            printf("[% 4.2f,% 4.2f,% 4.2f] ", pt3d.x, pt3d.y, pt3d.z);
        cout << endl;

        cout << "Reprojection Error:" << endl;
        for (double err : reprojErrors) {
            printf(" % -18.4f ", err);
        }
        cout << "\n" << endl;
    }

}

// Function that performs the reconstruction of a singular point from
// multiple 2D points into one 3D point, given the camera matrices.
double triangulatePoint(Point3d& reconstructedPoint, const vector<Point2d>& pointsOnEachCamera,
                        const vector<Mat>& cameraMats) {
    if (cameraMats.size() != pointsOnEachCamera.size())
        cerr << "Number of cameras: " << to_string(cameraMats.size()) <<
                " != # of points per camera: " << to_string(pointsOnEachCamera.size()) <<
                endl;

    // Create and fill A for homogenous equation system Ax = 0
    const int numCameras = (int)cameraMats.size();
    Mat A(numCameras*2, 4, CV_64F);


    // This method combines the perspective models to get an overdetermined homogenous 
    // system of linear equations that can be solved with SVD.
    // TODO: Current method minimizes algebraic error, if we implement a 
    // non-linear triangulation method to minimize geometric (reprojection) error, can
    // be more accurate. For John??
    for (int i = 0; i < numCameras; i++) {
        A.rowRange(i*2, i*2+1)   = pointsOnEachCamera[i].x * cameraMats[i].rowRange(2,3)
                                 - cameraMats[i].rowRange(0,1);
        A.rowRange(i*2+1, i*2+2) = pointsOnEachCamera[i].y * cameraMats[i].rowRange(2,3)
                                 - cameraMats[i].rowRange(1,2);
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
        Mat reprojectedPoint = cameraMats[i] * reconstructedMat;
        reprojectedPoint /= reprojectedPoint.at<double>(2,0);
        
        average += sqrt(pow(reprojectedPoint.at<double>(0,0) - pointsOnEachCamera[i].x,2)
                      + pow(reprojectedPoint.at<double>(1,0) - pointsOnEachCamera[i].y,2));
    }

    return average / numCameras;
}

// Helper funtction to print cv::Mat matrices 
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
