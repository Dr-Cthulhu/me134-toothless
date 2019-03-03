/**
 * Dice Recogniction
 *
 * @author Glenn De Backer >
 *
 * @license Creative Commons Attribution-NonCommercial 3.0 / CC BY-NC 3.0
 *
 */

 // std lib
 #include <iostream>
 #include <sstream>
#include "opencv2/objdetect/objdetect.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
using namespace std;
using namespace cv;
#include "opencv2/opencv.hpp"
 // OpenCV
 #include <opencv2/core.hpp>
 #include "opencv2/objdetect.hpp"
 #include <opencv2/imgproc.hpp>
 //#include <opencv2/highgui.hpp>
 #include <opencv2/features2d.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class DiceDetector {
	// Constraints

public:
	ros::NodeHandle nh_;

	boost::mutex connect_mutex_;
	bool use_rbgd_;

	// Subscription topics
	string image_image_;

	// RBG-D
	string camera_;
	string camera_topic_;
	string depth_image_;
	string depth_topic_;
	string camera_info_topic;
	string depth_info_topic;
	string rbg_ns_;
	string depth_ns_;

	image 


	int countPips(cv::Mat dice){

	  // resize
		cv::resize(dice, dice, cv::Size(150, 150));

	  // convert to grayscale
	  cvtColor(dice, dice, CV_BGR2GRAY);

	  // threshold
	  cv::threshold(dice, dice, 150, 255, cv::THRESH_BINARY | CV_THRESH_OTSU );

	  // show
	  cv::namedWindow("processed", true);
	  cv::imshow("processed", dice);


	  // floodfill
	  cv::floodFill(dice, cv::Point(0,0), cv::Scalar(255));
	  cv::floodFill(dice, cv::Point(0,149), cv::Scalar(255));
	  cv::floodFill(dice, cv::Point(149,0), cv::Scalar(255));
	  cv::floodFill(dice, cv::Point(149,149), cv::Scalar(255));

	  // search for blobs
	  cv::SimpleBlobDetector::Params params;

	  // filter by interia defines how elongated a shape is.
	  params.filterByInertia = true;
	  params.minInertiaRatio = 0.5;

	  // will hold our keyponts
	  std::vector<cv::KeyPoint> keypoints;

	  // create new blob detector with our parameters
	  cv::Ptr<cv::SimpleBlobDetector> blobDetector = cv::SimpleBlobDetector::create(params);

	  // detect blobs
	  blobDetector->detect(dice, keypoints);


	  // return number of pips
	  return keypoints.size();
	}

	bool isSquare(vector<Point> contour) {
	  int peri = cv::arcLength(contour, true);
	  vector<Point> approx;
	  cv::approxPolyDP(contour, approx, 0.04 * peri, true);
	  if (approx.size() == 4) {
	    cv::RotatedRect bound = cv::minAreaRect(contour);
	    int ar = bound.size.width / bound.size.height;
	    if (ar > 0.9 && ar < 1.1)
	      return true;
	  }
	  return false;
	}

	int main( int argc, char** argv )
	{

	  cout <<"start";
	  // open window frame
	  cv::namedWindow("frame", true);

	  while(true){
	    // open the default camera
	    cv::VideoCapture cap(0);
	    

	    // check if we succesfully opened the camera stream
	    if(!cap.isOpened()){
	      cout <<"coouldnt open camera \n";
	      return -1;
	    }

	    // set camera properties
	    cap.set(CV_CAP_PROP_FRAME_WIDTH,1920);
	    cap.set(CV_CAP_PROP_FRAME_HEIGHT,1080);

	    //  take single frame (without dices) and convert it to grayscale
	    //  to use when removing the background
	    cv::Mat backgroundFrame;
	    cap >> backgroundFrame;
	    cvtColor(backgroundFrame, backgroundFrame, CV_BGR2GRAY);

	    // will hold our frame
	    //cv::Mat frame;
	    //cv::Mat unprocessFrame;
	    Mat frame;
	    Mat unprocessFrame;
	    int totalDotCount = 0;
	    Scalar markingColor = Scalar(0, 200, 0);

	    cap >> frame;
	    //cap.read(frame);

	    
	    unprocessFrame = frame.clone();
	    cv::imshow("frame", unprocessFrame);
	    waitKey(33);
	    // if (waitKey(30) >= 0)
	    // {
	    //   continue;
	    //   }

	    inRange(unprocessFrame, Scalar(200, 200, 200), Scalar(255, 255, 255), frame);
	    //double diceContourArea = cv::contourArea(diceContours[0]);
	    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	    erode(frame, frame, element);
	    dilate(frame, frame, element);

	    vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
	    findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	    int numberOfDice = 0;
	    int dotCount = 0;
	    int last_square_index = 0;

	    for(int i = 0; i < contours.size(); i++){
	      if (hierarchy[i][3] == -1) {
	        if (!isSquare(contours[i]))
	          continue;
	        numberOfDice += 1;
	        dotCount = 0;
	        last_square_index = i;
	      } else {
	        dotCount += 1;

	        if ((i == contours.size() - 1) || (hierarchy[i+1][3] == -1)) {
	          totalDotCount += dotCount;

	          stringstream stream;
	          stream << dotCount;
	          Moments m = moments(contours[last_square_index]);
	          putText(unprocessFrame, stream.str(), Point(m.m10 / m.m00 + 100, m.m01 / m.m00), FONT_HERSHEY_PLAIN, 2, markingColor, 2);
	        }
	      }
	      drawContours(unprocessFrame, contours, i, markingColor, 2, 8, hierarchy, 0, Point());
	    }
	    
	    cv::imshow("frame", unprocessFrame);
	    waitKey(33);
	}
	
}
