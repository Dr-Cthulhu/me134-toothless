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
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>

static const std::string OPENCV_WINDOW = "Image window";

class DiceDetector {

	ros::NodeHandle nh_;
  	image_transport::ImageTransport it_;
 	image_transport::Subscriber image_sub_;
 	ros::Publisher point_pub_;

 public:
 	DiceDetector()
  		: it_(nh_)
 	{
		ros::Rate loop_rate(0.1);
 		// Subscribe to input video feed and publish pointcloud
  		image_sub_ = it_.subscribe("/cam/image_rect_color", 1, &DiceDetector::findDice, this);
		point_pub_ = nh_.advertise<visualization_msgs::Marker>("dice_points", 1);

		cv::namedWindow(OPENCV_WINDOW);
		ROS_INFO_STREAM("Created new dice detector");
	}

	~DiceDetector() {
		cv::destroyWindow(OPENCV_WINDOW);
	}

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

	bool isRightSize(vector<Point> contour) {
		float area = cv::contourArea(contour);
		return (area > 500 && area < 700);
	}

	float getX(int x) {
		float x_adj =  45.8 - ((float(x) - 40) * (45.8 / 695) - ((x - 400) / 40000));
		return (x_adj) / 100;
	}

	float getY(int y) {
		float y_adj = (float(y) - 15) * (38.0 / 580) - ((y - 300) / 30000);
		return (y_adj) / 100;
	}

	void findDice(const sensor_msgs::ImageConstPtr& msg) {
		// ROS_INFO_STREAM("finding dice...");

		// Initialize point-cloud
		visualization_msgs::Marker all;
		all.header.frame_id = "world_frame";
		all.header.stamp = ros::Time::now();
		all.type = visualization_msgs::Marker::POINTS;

		cv_bridge::CvImagePtr cv_ptr;
		try {
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}

	    Mat frame;
	    Mat unprocessFrame;
	    int totalDotCount = 0;
	    Scalar markingColor = Scalar(0, 200, 0);

	    frame = cv_ptr->image;
	    unprocessFrame = frame.clone();
	    
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
	    vector<Point> real;

	    for(int i = 0; i < contours.size(); i++){
	      	if (hierarchy[i][3] == -1) {
		        if (!isRightSize(contours[i]))
		          	continue;
		        numberOfDice += 1;
		        
		        Point realPoint = Point(0, 0);
		        cv::RotatedRect bound = cv::minAreaRect(contours[i]);
		        realPoint.x = bound.center.x;
		        realPoint.y = bound.center.y;
		        
		        real.push_back(realPoint);

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
		    	
		    	drawContours(unprocessFrame, contours, i, markingColor, 2, 8, hierarchy, 0, Point());
		    }
	    }
	    // ROS_INFO_STREAM("number of dice: " << numberOfDice);

	    all.points.resize(numberOfDice);
	    for (int i = 0; i < numberOfDice; i++) {
	    	geometry_msgs::Point point;
	    	point.x = getX(real[i].x);
	    	point.y = getY(real[i].y);
	    	all.points[i] = point;
	    }

	    // cv::imshow(OPENCV_WINDOW, unprocessFrame);
	    // cv::waitKey(3);
	    point_pub_.publish(all);
	}
};

int main( int argc, char** argv )
{
	ros::init(argc, argv, "dice_publisher");
	ROS_INFO_STREAM("dice_detector running");
	DiceDetector dd;
	ros::spin();
	return 0;
}
	

