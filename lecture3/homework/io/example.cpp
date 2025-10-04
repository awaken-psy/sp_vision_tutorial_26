#include "my_camera.hpp"
#include <iostream>

int main()
{
    myCamera camera;
    
    cv::Mat frame;
    if (camera.read(frame)) {
        cv::imshow("Camera Frame", frame);
        cv::waitKey(0);
    } else {
        std::cout << "Failed to read frame!" << std::endl;
    }
    
    return 0;
}