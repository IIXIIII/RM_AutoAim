#include <opencv2/opencv.hpp>
#include <iostream>
int main() {
    // Load an image from the current directory
    cv::Mat image = cv::imread("C:\\Desktop\\C\\opencv_test\\198.jpg");

    // Check if the image was loaded successfully
    if (image.empty()) {
        std::cerr << "Error: Could not load image!" << std::endl;
        return 1;
    }

    // Display the image in a window
    cv::imshow("OpenCV Test - Press any key to exit", image);

    // Wait for a key press
    cv::waitKey(0);

    return 0;
}