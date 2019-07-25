#include <openpose/headers.hpp>
#include <unistd.h>

using namespace std;
using namespace cv;

const string FILEPATH = "./media/matrices.xml";
const int BOARD_WIDTH = 8;
const int BOARD_HEIGHT = 6;
const float SQUARE_SIZE = 0.115;
const bool VERBOSE = true;
const bool CHECKCORNERS = false;

void getCameraProjections(vector<Mat>& projectionMatrices, const vector<Mat>& calib_images,
							const vector<vector<Point3f>>& objectPoints);
void getCameraMatrices(const Mat& image_L, const Mat& image_R,
					   Mat& projection_L, Mat& projection_R);
double getCameraIntrinsics(const Mat im, const vector<vector<Point3f>>& objectPoints,
						   vector<vector<Point2f>>& imagePoints,
						   Mat& cameraMatrix, Mat& distCoeff);
double computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
								 const vector<vector<Point2f>>& imagePoints,
                                 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs);
void printMat(Mat mat, int prec);

int main(int argc, char *argv[])
{
	if (argc < 3) {
		cout << "\nUsage: camera-calibration [cam1 path] [cam2 path] ... [camN path]" << endl;
		cout << "Number of cameras must be greater than 1." << endl;
		return 0;
	}

	// Read in all calibration images
	vector<Mat> calib_images;
	for (int i = 1; i < argc; i++) {
		Mat image = imread(argv[i]);

		// Return error if image can't be read
		if (image.empty()) {
			printf("Could not load image from path: %s\n", argv[i]);
			return -1;
		}

		calib_images.push_back(image.clone());
	}

	int numCameras = calib_images.size();

	// If you decide to use a different calibration board, uncomment this
	/*
	cout << "     Board width: ";
	cin >> BOARD_WIDTH;
	cout << "    Board height: ";
	cin >> BOARD_HEIGHT;
	cout << "Square size (in): ";
	cin >> SQUARE_SIZE;
	*/

	vector<Mat> projectionMatrices;
	vector<vector<Point3f>> objectPoints;
	vector<Point3f> temp_obj;
	for (int i = 0; i < BOARD_HEIGHT; i++)
		for (int j = 0; j < BOARD_WIDTH; j++)
			temp_obj.push_back(Point3f((float)j * SQUARE_SIZE,
									  -(float)i * SQUARE_SIZE,
									   0));
	objectPoints.push_back(temp_obj);
	getCameraProjections(projectionMatrices, calib_images, objectPoints);

	if (VERBOSE) {
		for (int i = 0; i < projectionMatrices.size(); i++) {
			printf("Projection %d\n", i);
			printMat(projectionMatrices[i], 5);
			cout << endl;
		}
	}

	FileStorage file(FILEPATH, FileStorage::WRITE);
	file << "NumCameras" << (Mat_<int>(1,1) << projectionMatrices.size());
	for (int i = 0; i < projectionMatrices.size(); i++) {
		string header = "Projection" + to_string(i);
		file << header << projectionMatrices[i];
	}
	file.release();

	cout << "\nSaved projection matrices to " << FILEPATH << ".\n" << endl;
    return 0;
}

void getCameraProjections(vector<Mat>& projectionMatrices, const vector<Mat>& calib_images,
							const vector<vector<Point3f>>& objectPoints) {
	double bestReprojError = 1;
	Mat bestIntrinsic, bestDistortion;
	Mat base = (Mat_<double>(1,4) << 0, 0, 0, 1);
	vector<vector<vector<Point2f>>> imagePoints;
	vector<Mat> K, d, cameraExtrinsics;

	// Solve each camera intrinsic and use the one with the lowest
	// reprojection error (we assume that the camera models are the same)
	for (int i = 0; i < calib_images.size(); i++) {
		vector<vector<Point2f>> imPoints;
		Mat cameraMatrix, distCoeff;
		double reprojError = getCameraIntrinsics(calib_images[i], objectPoints, imPoints,
							 					 cameraMatrix, distCoeff);

		imagePoints.push_back(imPoints);
		K.push_back(cameraMatrix);
		d.push_back(distCoeff);

		if (reprojError < bestReprojError) {
			bestReprojError = reprojError;
			bestIntrinsic = cameraMatrix;
			bestDistortion = distCoeff;
		}

		if (VERBOSE) {
			cout << "Camera " << i << ":" << endl;
			printMat(cameraMatrix, 5);
			printMat(distCoeff, 5);
			cout << reprojError << "\n" << endl;
		}
	}

	// Initialize left most camera as origin
	cameraExtrinsics.push_back(Mat::eye(4, 4, CV_64F));

	for (int i = 0; i < calib_images.size() - 1; i++) {
		Mat R, T, E, F;

		// Get Rotation (R) and Translation (T) w/ respect to prev camera
		stereoCalibrate(objectPoints, imagePoints[i], imagePoints[i+1],			
						K[i], d[i], K[i+1], d[i+1], calib_images[i].size(),
						R, T, E, F);
		Mat tempExtrinsic;
		hconcat(R, T, tempExtrinsic);									// Extrinsic = [R|t]
		tempExtrinsic.push_back(base);
		// Calculate camera extrinsic with respect to origin (left most camera)
		tempExtrinsic = (tempExtrinsic * cameraExtrinsics[i]); 
		cameraExtrinsics.push_back(tempExtrinsic);
	}

	for (Mat extrinsic : cameraExtrinsics) {
		extrinsic.pop_back(1);
		projectionMatrices.push_back(bestIntrinsic * extrinsic);		// Projection = K[R|t]
	}
}

double getCameraIntrinsics(const Mat im, const vector<vector<Point3f>>& objectPoints,
						   vector<vector<Point2f>>& imagePoints,
						   Mat& cameraMatrix, Mat& distCoeff)
{
	vector<Point2f> corners;
    vector<Mat> rvecs, tvecs;

	if (findChessboardCorners(im, Size(BOARD_WIDTH, BOARD_HEIGHT), corners, 
    						  CV_CALIB_CB_ADAPTIVE_THRESH | 
    						  CV_CALIB_CB_FILTER_QUADS))
    {
    	Mat gray_im;
    	cvtColor(im, gray_im, CV_BGR2GRAY);
    	cornerSubPix(gray_im, corners, Size(5, 5), Size(-1,-1),
    			 	 TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1 ));
    	
    	if (CHECKCORNERS) {
			Mat temp_im = im.clone();
			drawChessboardCorners(temp_im, Size(BOARD_WIDTH, BOARD_HEIGHT), corners, true);
			imshow("", temp_im);
			waitKey(0);
		}

		imagePoints.push_back(corners);
    }

    // calibrateCamera(objectPoints, imagePoints, im.size(), cameraMatrix,
    // 				distCoeff, rvecs, tvecs);
    calibrateCamera(objectPoints, imagePoints, im.size(), cameraMatrix,
    				distCoeff, rvecs, tvecs, CV_CALIB_FIX_PRINCIPAL_POINT);

    Mat temp_im;
	undistort(im, temp_im, cameraMatrix, distCoeff);
	imshow("", temp_im);
	waitKey(0);

    return computeReprojectionErrors(objectPoints, imagePoints, rvecs, tvecs, cameraMatrix, distCoeff);
}

double computeReprojectionErrors(const vector<vector<Point3f>>& objectPoints,
								 const vector<vector<Point2f>>& imagePoints,
                                 const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs)
{
	vector<Point2f> imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector<float> perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
		              distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float) std::sqrt(err*err/n);
		totalErr += err*err;
		totalPoints += n;
	}

	return std::sqrt(totalErr/totalPoints);
}

void printMat(Mat mat, int prec)
{      
    for(int i=0; i<mat.size().height; i++)
    {
        cout << "[";
        for(int j=0; j<mat.size().width; j++)
        {
        	printf("%10.*f", prec, mat.at<double>(i, j));
            if(j != mat.size().width-1)
                cout << ", ";
            else
                cout << "]" << endl; 
        }
    }
}