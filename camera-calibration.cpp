#include <openpose/headers.hpp>

using namespace std;
using namespace cv;

const string FILEPATH = "./media/matrices.xml";

int BOARD_WIDTH;
int BOARD_HEIGHT;

float SQUARE_SIZE;

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
		cout << "\nUsage: camera-calibration [left camera path] [right camera path]" << endl;
		return 0;
	}

	Mat image_L = imread(argv[1]);
	Mat image_R = imread(argv[2]);

	if (image_L.empty() || image_R.empty()) {
		cerr << "Could not load images." << endl;
		exit(0);
	}

	cout << "     Board width: ";
	cin >> BOARD_WIDTH;
	cout << "    Board height: ";
	cin >> BOARD_HEIGHT;
	cout << "Square size (in): ";
	cin >> SQUARE_SIZE;

	vector<Mat> cameraMatrices;
	Mat projection_L, projection_R;

	getCameraMatrices(image_L, image_R, projection_L, projection_R);

	FileStorage file(FILEPATH, FileStorage::WRITE);
	file << "projection_L" << projection_L;
	file << "projection_R" << projection_R;
	file.release();

	cout << "\nSaved projection matrices to files.\n" << endl;
    return 0;
}

void getCameraMatrices(const Mat& image_L, const Mat& image_R,
					   Mat& projection_L, Mat& projection_R)
{
	Mat K_L, K_R, d_L, d_R, R, T, E, F, extrinsic_L, extrinsic_R;
	vector<vector<Point3f>> objectPoints;
	vector<vector<Point2f>> imagePoints_L, imagePoints_R;
	vector<Point3f> temp_obj;
	double reprojError;

	for (int i = 0; i < BOARD_HEIGHT; i++)
		for (int j = 0; j < BOARD_WIDTH; j++)
			temp_obj.push_back(Point3f((float)j * SQUARE_SIZE,
									   (float)i * SQUARE_SIZE,
									   0));
	objectPoints.push_back(temp_obj);

	reprojError = getCameraIntrinsics(image_L, objectPoints, imagePoints_L, K_L, d_L);
	cout << "\nLeft Camera Calibration Error: " << reprojError << endl;
	reprojError = getCameraIntrinsics(image_R, objectPoints, imagePoints_R, K_R, d_R);
	cout << "Right Camera Calibration Error: " << reprojError << endl;

	stereoCalibrate(objectPoints, imagePoints_L, imagePoints_R,
					K_L, d_L, K_R, d_R, image_L.size(), R, T, E, F);

	extrinsic_L = Mat::eye(3, 4, CV_64F);	// Left extrinsic = origin
	hconcat(R, T, extrinsic_R);				// Right extrinsic = [R|t]

	projection_L = K_L * extrinsic_L;		// Projection = K[R|t]
	projection_R = K_R * extrinsic_R;

	cout << "\nLeft projection:" << endl;
	printMat(projection_L, 5);
	cout << "Right projection:" << endl;
	printMat(projection_R, 5);
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
    	
		// Mat temp_im = im.clone();
		// drawChessboardCorners(temp_im, Size(BOARD_WIDTH, BOARD_HEIGHT), corners, true);
		// imshow("", temp_im);
		// waitKey(0);

		imagePoints.push_back(corners);
    }

    calibrateCamera(objectPoints, imagePoints, im.size(),
    				cameraMatrix, distCoeff, rvecs, tvecs,
    				CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5);

    // cout << "K: " << endl;
    // printMat(cameraMatrix, 5);
    // cout << "D: " << endl;
    // printMat(distCoeff);

	// undistort(im, temp_im, cameraMatrix, distCoeff);
	// imshow("", temp_im);
	// waitKey(0);

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
        	printf("%10f", mat.at<double>(i,j));
            // cout << setprecision(prec) << mat.at<double>(i,j);
            if(j != mat.size().width-1)
                cout << ", ";
            else
                cout << "]" << endl; 
        }
    }
}