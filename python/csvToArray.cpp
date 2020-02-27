#include <fstream>
#include <sstream>
#include <iostream>

using namespace std;

int loadOCamCalibMatrix(double data[720][1280], string path){

    std::ifstream file(path);

    for(int row = 0; row < 720; ++row)
    {
        std::string line;
        std::getline(file, line);
        if ( !file.good() )
            break;

        std::stringstream iss(line);

        for (int col = 0; col < 1280; ++col)
        {
            std::string val;
            std::getline(iss, val, ',');

            if ( !iss.good() )
                break;

            std::stringstream convertor(val);
            convertor >> data[row][col];
        }
    }

    return 0;
}

int main()
{
    cout << "Hello Cpp Worlds... GRG is finally here!\n";

    double data[720][1280];
    loadOCamCalibMatrix(data, "../mapy_persp_32.csv");

    cout.precision(25);
    cout << data[0][0];
    cout << '\n';
    cout << '\n';


    cout.precision(25);
    cout << data[0][638];
    cout << '\n';
    cout << '\n';
    
    cout.precision(25);
    cout << data[0][1279];
    cout << '\n';
    cout << '\n';

    cout.precision(25);
    cout << data[719][1279];
    cout << '\n';

    return 0;
}