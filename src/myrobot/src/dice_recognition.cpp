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
// ROS + CVBridge
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

static const std::string OPENCV_WINDOW = "Image window";

class DiceDetector {

	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
 	image_transport::Subscriber image_sub_;
 	image_transport::Publisher image_pub_;

 public:
 	DiceDetector()
  		: it_(nh_)
 	{
 		// Subscribe to input video feed and publish pointcloud
  		image_sub_ = it_.subscribe("/camera/image_raw", 1, &DiceDetector::findDice, this);
		point_pub_ = nh_.advertise<sensor_msgs::PointCloud>("dice_points", 1);
	}

	~DiceDetector() {
	}

	int countPips(cv::Mat dice) {
	  	// resize
		cv::resize(dice, dice, cv::Size(150, 150));

	  	// convert to grayscale
	  	cvtColor(dice, dice, CV_BGR2GRAY);

	  	// threshold
	  	cv::threshold(dice, dice, 150, 255, cv::THRESH_BINARY | CV_THRESH_OTSU );

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

	bool isRightSize(vector<Point> contour) {
		float area = cv::contourArea(contour);
		return (area > 1500 && area < 2000);
	}

	void findDice(const sensor_msgs::ImageConstPtr& msg) {

		// Initialize point-cloud
		sensor_msgs::PointCloud all;
		ros::Time scan_time = ros::Time::now();
		all.header.frame_id = "world_frame";

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	    Mat frame;
	    int totalDotCount = 0;
	    Scalar markingColor = Scalar(0, 200, 0);

	    frame = cv_ptr->image;
	    
	    inRange(unprocessFrame, Scalar(200, 200, 200), Scalar(255, 255, 255), frame);
	    //double diceContourArea = cv::contourArea(diceContours[0]);
	    Mat element = getStructuringElement(MORPH_RECT, Size(3, 3), Point(1, 1));
	    erode(frame, frame, element);
	    dilate(frame, frame, element);

	    vector<vector<Point>> contours;
	    vector<Vec4i> hierarchy;
	    findContours(frame, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	    int numberOfDice = 0;
	    vector<Point> real;

	    for(int i = 0; i < contours.size(); i++){
	      	if (hierarchy[i][3] == -1) {
		        if (!isRightSize(contours[i]))
		          	continue;
		        numberOfDice += 1;
		        for (int j = 0; j < contours[i].size(); j++) {
		        	Point realPoint;
		        	cv::RotatedRect bound = cv::minAreaRect(contours[i]);
		        	realPoint.x = bound.x + bound.size.width / 2;
		        	realPoint.y = bound.y + bound.size.width / 2;
		        }
		        real.append(realPoint);
		    }
	    }

	    all.points.resize(numberOfDice);
	    for (int i = 0; i < numberOfDice; i++) {
	    	sensor_msgs::Point32 point;
	    	point.x = real[i].x;
	    	point.y = real[i].y;
	    	all.points[i] = point;
	    }
	    point_pub_.publish(all);
	}
}

int main( int argc, char** argv )
{
	ros::init(argc, argv, "dice_publisher");
	DiceDetector dd;
	ros::spin();
	return 0;
}
	

