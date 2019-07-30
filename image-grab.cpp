#include <openpose/headers.hpp>

using namespace cv;
using namespace std;

const string LAMBDA_NUMBER = "1";
const string FILEPATH = "./media/calib" + LAMBDA_NUMBER + ".jpg";

int main(int argc, char const *argv[])
{
	VideoCapture cam(0);
	if (!cam.isOpened())
	{
		cerr << "No video cam found." << endl;
		return 1;
	}

	Mat im;

	// Wait till 'q' key is pressed then saves image
	while (waitKey(1) != 'q')
	{ 	
		cam >> im;
		imshow("image", im);
	}

	imwrite(FILEPATH, im);

	return 0;
}