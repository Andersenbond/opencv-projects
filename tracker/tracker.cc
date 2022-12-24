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
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstring>
#include "samples_utility.hpp"

using namespace cv;
using namespace std;
 
 

int main( int argc, char** argv ){
  // show help
 
   // printf("Creating Tracker");
   cout << "primeiro" << endl; 
  // create the tracker
  Ptr<Tracker> tracker = TrackerKCF::create();
   
   cout << "segundo" << endl; 

 // Read video  
 // Read video
    VideoCapture cameraStream(0, CAP_V4L2);
    //VideoCapture cameraStream;//(0, CAP_V4L2);
    //cameraStream.open("nvarguscamerasrc ! video/x-raw(memory:NVMM),height=640,width=480,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! queue ! appsink drop=1 ", cv::CAP_GSTREAMER);
    //cameraStream.open(0, CAP_V4L2);
    
        cout << "terceiro" << endl; 

    // Exit if video is not opened
    if(!cameraStream.isOpened())
    {
        cout << "Could not read video file" << endl; 
        return 1; 
    } 
    else 
    {
        cout  << "Webcam opened !!!";
    }

  Mat frame;

  // get bounding box
  cameraStream >> frame;
          cout << "quarto " << endl; 

  Rect roi = selectROI("tracker", frame, true, false);
 
  //quit if ROI was not selected
  if(roi.width==0 || roi.height==0)
    return 0;

  // initialize the tracker
  tracker->init(frame,roi);

  // do the tracking
  printf("Start the tracking process, press ESC to quit.\n");
  for ( ;; ){
    // get frame from the video
    cameraStream >> frame;
    
    //cout << frame.rows==0 << frame.cols << endl;

    // stop the program if no more images
    if(frame.rows==0 || frame.cols==0)
    {
      break;
            
    }

    // update the tracking result
    bool isfound = tracker->update(frame,roi);
    if(!isfound)
    {
        cout << "The target has been lost...\n";
        waitKey(0);
        return 0;
    }

    // draw the tracked object
    rectangle( frame, roi, Scalar( 255, 0, 0 ), 2, 1 );

    // show image with the tracked object
    imshow("tracker",frame);

    //quit on ESC button
    if(waitKey(1)==27)break;
  }

}
