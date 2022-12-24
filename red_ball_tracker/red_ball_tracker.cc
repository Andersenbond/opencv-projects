#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>

#include "samples_utility.hpp"

using namespace cv;
using namespace std;
 
Mat myFilterBlobs(Mat frame, double minValidArea = 0.0);
Mat findBiggestItem(Mat frame, double minValidArea = 0.0);
void seColorDetectionRange(Mat* frame, const Scalar lower, const Scalar upper);

/* Define the Ball threshold colors, in this program it is 
 * expected a red ball. Here we define the lower and upper 
 * boundaried of the red ball in the HSV color space */
Scalar lowerRedRGB = Scalar(160,50,50);
Scalar upperRedRGB = Scalar(180,255,255);

Scalar lowerRedRGB1Ba = Scalar(0 ,70, 50);
Scalar upperRedRGB1Ba = Scalar(10,255, 255);
Scalar lowerRedRGB1Bb = Scalar(170 ,70, 50);
Scalar upperRedRGB1Bb = Scalar(180,255, 255);
const Scalar orangeLowerHSVRange = Scalar(5, 50, 50);
const Scalar orangeUpperHSVRange = Scalar(13, 255, 255);

int main( int argc, char** argv )
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

    /* Convert the frame to HSV colormap and apply 
     * Gaussain blur */
    cvtColor(frame, hsvFrame, COLOR_BGR2HSV);
    // imshow("hsvFrame2",hsvFrame);

    /* Applying a 1x1 Gaussian blur will help reduce the 
     * noise in the image */
    //blur(hsvFrame, hsvFrame, cv::Size(1,1));
    
    Mat1b mask1, mask2;
    // inRange(hsvFrame, lowerRedRGB1Ba, upperRedRGB1Ba, mask1);
    // inRange(hsvFrame, lowerRedRGB1Bb, upperRedRGB1Bb, mask2);
    inRange(hsvFrame, orangeLowerHSVRange, orangeUpperHSVRange, mask2);
        // imshow("mask2",mask2);
    thresholdFrame = /* mask1 |*/ mask2;
    imshow("thresholdFrame1",thresholdFrame);

    erode(thresholdFrame, thresholdFrame, 0, Point(-1,-1), 2);
    dilate(thresholdFrame, thresholdFrame, 0, Point(-1,-1), 2);

    /* Threshold the image with the color range defined above. 
     * The result should be a black and white image.  */
    // inRange(hsvFrame, lowerRedRGB, upperRedRGB, thresholdFrame);
 
    /* Show the image - test only */

    // imshow("thresholdFrame2",findBiggestItem(thresholdFrame));

    /* Calculate the center of the ball if detected */
    Moments m = moments((thresholdFrame), false);
    Point com(m.m10 / m.m00, m.m01 / m.m00);

    /* Draw the marker around the ball */
    Scalar color = Scalar(0, 0, 255);
    drawMarker(frame, com, color, MARKER_CROSS, 50, 5);

    /* Show the frame containing the marker on the ball */
    imshow("Final", frame);

// show image with the tracked object
  
      //quit on ESC button
    if(waitKey(1)==27)break;
  }

}


Mat myFilterBlobs(Mat frame, double minValidArea)
{
  Mat ret = frame;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  int largestContoursIndex = -1;
  double maxArea = 0.0;

  findContours(frame, contours, hierarchy, RETR_EXTERNAL, 
  CHAIN_APPROX_SIMPLE, Point());

  for(int i = 0; i < contours.size(); i++)
  {
    double area = contourArea(contours[i]);
    if(area > maxArea)
    {
            maxArea = area;
      largestContoursIndex = i;
    }
  }
  if(maxArea > minValidArea)
  {     

    drawContours(ret, contours, 
                  largestContoursIndex, 
                    Scalar(255), FILLED, 8);
  } 
  return ret;
}

Mat findBiggestItem(Mat frame, double minValidArea)
{
  Mat ret = frame.clone();
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  int largestContoursIndex = -1;
  double maxArea = 0.0;

  findContours(frame.clone(), contours, hierarchy, RETR_EXTERNAL, 
  CHAIN_APPROX_SIMPLE, Point());

  if(contours.size() > 2)
  { 
    for(int i = 0; i < contours.size(); i++)
    {
      double area = contourArea(contours[i]);
      if(area > maxArea)
      {
          maxArea = area;
          largestContoursIndex = i;
      }
    }
        
    drawContours(ret, contours, 
                    largestContoursIndex, 
                      Scalar(255), FILLED, 8);
    

 /* Calculate the center of the ball if detected */
    Moments m = moments(contours[largestContoursIndex], false);
    Point com(m.m10 / m.m00, m.m01 / m.m00);

    /* Draw the marker around the ball */
    Scalar color = Scalar(0, 255, 0);
    drawMarker(frame, com, color, MARKER_SQUARE, 50, 5);
    
    ret &= frame;

  } 
  else
  {
    ret = frame;
  } 
  return ret;
}

void seColorDetectionRange(Mat* frame, const Scalar lower, const Scalar upper)
{
  inRange(*frame, lower, upper, *frame);
}