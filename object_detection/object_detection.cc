#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>

using namespace cv;
using namespace std;
int main(int argc, char** argv)
{

  cout << "Read ball tracker app started" << endl; 

  /* Open the camera stream */
  VideoCapture cameraStream(0, CAP_V4L2);
 
  // Exit if video is not opened
  if(!cameraStream.isOpened())
  {
    cout << "Unable to load the video stream" << endl; 
    return 1; 
  }
  
  else 
  {
    cout  << "Video Stream loaded" << endl;
  }

    /* Define helper variable */
    Mat frame;
    Mat hsvFrame;
    Mat thresholdFrame;

    /* While */
    while(cameraStream.isOpened())
    {
        /* Get frame from the video */ 
        cameraStream >> frame;
        
        // Convert input image to HSV
        cv::Mat hsv_image;
        cv::cvtColor(frame, hsv_image, cv::COLOR_BGR2HSV);

        // Threshold the HSV image, keep only the red pixels
        cv::Mat lower_red_hue_range;
        cv::Mat upper_red_hue_range;
        cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(10, 255, 255), lower_red_hue_range);
        cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(179, 255, 255), upper_red_hue_range);

        // Combine the above two images
        cv::Mat red_hue_image;
        cv::addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0, red_hue_image);

        cv::GaussianBlur(red_hue_image, red_hue_image, cv::Size(9, 9), 2, 2);

        // Use the Hough transform to detect circles in the combined threshold image
        std::vector<cv::Vec3f> circles;
        cv::HoughCircles(red_hue_image, circles, HOUGH_GRADIENT, 1, red_hue_image.rows/8, 100, 20, 0, 0);

        // Loop over all detected circles and outline them on the original image
        if(circles.size() == 0) std::exit(-1);
        for(size_t current_circle = 0; current_circle < circles.size(); ++current_circle) {
            cv::Point center(std::round(circles[current_circle][0]), std::round(circles[current_circle][1]));
            int radius = std::round(circles[current_circle][2]);

            cv::circle(frame, center, radius, cv::Scalar(0, 255, 0), 5);
        }

        // Show images
        cv::namedWindow("Threshold lower image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold lower image", lower_red_hue_range);
        cv::namedWindow("Threshold upper image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Threshold upper image", upper_red_hue_range);
        cv::namedWindow("Combined threshold images", cv::WINDOW_AUTOSIZE);
        cv::imshow("Combined threshold images", red_hue_image);
        cv::namedWindow("Detected red circles on the input image", cv::WINDOW_AUTOSIZE);
        cv::imshow("Detected red circles on the input image", frame);
    }
    waitKey();
    return EXIT_SUCCESS;
}