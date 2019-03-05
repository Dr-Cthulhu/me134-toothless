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



int countPips(cv::Mat dice){

  // resize
  cv::resize(dice, dice, cv::Size(150, 150));

  // convert to grayscale
  cvtColor(dice, dice, CV_BGR2GRAY);

  // threshold
  cv::threshold(dice, dice, 150, 255, cv::THRESH_BINARY | CV_THRESH_OTSU );

  // show
  // cv::namedWindow("processed", true);
  // cv::imshow("processed", dice);


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

float isSquare(vector<Point> contour) {
  int peri = cv::arcLength(contour, true);
  vector<Point> approx;
  cv::approxPolyDP(contour, approx, 0.04 * peri, true);
  if (approx.size() == 4) {
    cv::RotatedRect bound = cv::minAreaRect(contour);
    int ar = bound.size.width / bound.size.height;
    if (ar > 0.9 && ar < 1.1)
      return cv::contourArea(contour);
  }
  return -1;
}

bool isRightSize(vector<Point> contour) {
	float area = cv::contourArea(contour);
	return (area > 1500 && area < 2000);
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
        	if (!isRightSize(contours[i]))
          		continue;
        	numberOfDice += 1;

	     	cv::RotatedRect rect = cv::minAreaRect(contours[i]);
	     	float angle = rect.angle;
	     	Size rect_size = rect.size;
	     	// following may be unnecessary but to be on the safe side
	    	if (rect.angle < -45.0) {
	    		angle += 90.0;
	    		swap(rect_size.width, rect_size.height);
	    	}

	    	Mat M, rot, crop;
	    	M = getRotationMatrix2D(rect.center, angle, 1.0);
	    	warpAffine(unprocessFrame, rot, M, unprocessFrame.size(), INTER_CUBIC);
	    	getRectSubPix(rot, rect_size, rect.center, crop);
	    	dotCount = countPips(crop);

	    	stringstream stream;
	        stream << dotCount;
	        Moments m = moments(contours[i]);
	        putText(unprocessFrame, stream.str(), Point(m.m10 / m.m00 + 100, m.m01 / m.m00), FONT_HERSHEY_PLAIN, 2, markingColor, 2);
	    }
      drawContours(unprocessFrame, contours, i, markingColor, 2, 8, hierarchy, 0, Point());
    }
    
    cv::imshow("frame", unprocessFrame);
    waitKey(33);
  }
}