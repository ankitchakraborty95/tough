#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <perception_common/MultisenseImage.h>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include "opencv2/nonfree/features2d.hpp"


// ros::Publisher panelDetected;
// using namespace cv;

void getPanel(cv::Mat sampleImage, cv::Mat sceneImage){

//  cv::cvtColor(sampleImage, sampleImage, CV_BGR2GRAY);
//  cv::cvtColor(sceneImage, sceneImage, CV_BGR2GRAY);

// Detecting the features in the sample image and Task1 scene
  int minHessian = 700;
  cv::SurfFeatureDetector detector(minHessian);

  std::vector<cv::KeyPoint> keypoints_sample, keypoints_scene;
  detector.detect( sampleImage, keypoints_sample);
  detector.detect( sceneImage, keypoints_scene);

  cv::Mat sampleImage_kp, sceneImage_kp;

  cv::drawKeypoints(sampleImage, keypoints_sample, sampleImage_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);
  cv::drawKeypoints(sceneImage, keypoints_scene, sceneImage_kp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DEFAULT);

  cv::imshow("sampleImage", sampleImage_kp);
  cv::imshow("sceneImage", sceneImage_kp);

  // Calculating the decriptors in the scene (Feature Vectors)
  cv::SurfDescriptorExtractor extractor;

  cv::Mat descriptors_sample, descriptors_scene;

  extractor.compute(sampleImage, keypoints_sample, descriptors_sample);
  extractor.compute(sceneImage, keypoints_scene, descriptors_scene);

  // Matching feature vectors using FLANN based matcher
  cv::FlannBasedMatcher matcher;
  std::vector<cv::DMatch> matches;
  matcher.match(descriptors_sample, descriptors_scene, matches);

  double dist_min = 0;
  double dist_max = 100;

  for (unsigned int i = 0; i < descriptors_sample.rows; i++){
    double dist = matches[i].distance;
    ROS_INFO("Distance = %f", dist);
        // if(dist < dist_min) dist_min = dist;
    // if(dist > dist_max) dist_max = dist;
  }

  ROS_INFO("Maximum Distance = %f", dist_max);
  ROS_INFO("Minimum Distance = %f", dist_min);
  cv::Mat finalImage;

  cv::drawMatches(sampleImage, keypoints_sample, sceneImage, keypoints_scene, matches, finalImage, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

  cv::imshow("Final Image", finalImage);

  // cv::GaussianBlur(image, image, cv::Size(3,3), 2, 2);
  // cv::Canny(image, image, 40, 120, 3);
  // std::vector<cv::Vec3f> circles;
  // cv::HoughCircles(image, circles, CV_HOUGH_GRADIENT, 1, image.rows/8, 40, 120, 0, 0);
  // if(circles.size()!=0){
  //   for (unsigned int i = 0; i < circles.size(); i++){
  //     cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
  //     int radius = cvRound(circles[i][2]);
  //     cv::circle(image, center, 3, cv::Scalar(0, 255, 0), -1, 8, 0);
  //     cv::circle(image, center, radius, cv::Scalar(0, 0, 255), -1, 8, 0);
  //   }
  // }
  // else{
  // ROS_ERROR("No Circle found in the Image. Please Check the Parameters");
  // }
  //
  cv::namedWindow("Image", 0);
  cv::imshow("Image", sceneImage);
  cv::waitKey(1);
}


int main(int argc, char** argv){

  ros::init(argc, argv, "panelDetection");
  ros::NodeHandle nh;
  cv::Mat sampleImage = cv::imread("/home/sathya/indigo_ws/src/space_robotics_challenge/val_perception/val_panelDetection/samples/panel.png", 1);
  cv::imshow("Sample Image", sampleImage);
  // panelDetected = nh.advertise<std_msgs::Bool>("panelDetected", 1);
  ros::Rate rate(10);

  src_perception::MultisenseImage mi(nh);
  cv::Mat image;

  while(ros::ok()){

    if(mi.giveImage(image)){
      cv::flip(image, image, -1);
      getPanel(sampleImage, image);
      ROS_INFO("Got Image");
    }
    else{
      ROS_WARN("Cannot Read Image");
    }
    ros::spinOnce();
    rate.sleep();

  }

return 0;

}
