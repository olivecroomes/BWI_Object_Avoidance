#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>

#include <stdio.h>
#include <string.h>

//#include <boost/program_options.hpp>

#define NODE "opencv_example"

namespace {
  int qDepth_ = 1;               // ROS topic queue size

  enum Method {
    SOBEL,
    CANNY,
    DILATE,
    ERODE,
    FLOOD,
    THRESHOLD
  } method_;

  bool useStaticImage_;
  std::string staticImageFile_;

}

/* http://opencv.jp/opencv-2.2_org/cpp/imgproc_miscellaneous_image_transformations.html#cv-floodfill */
void getFloodFillImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  inputImage.copyTo(outputImage);

  cv::Point seed(inputImage.cols / 2, inputImage.rows / 2);
  cv::Scalar newVal(255, 0, 0);
  cv::Rect* rect = NULL;
  cv::Scalar loDiff(10, 10, 10);
  cv::Scalar upDiff(10, 10, 10);
  int flags = 8 | CV_FLOODFILL_FIXED_RANGE;

  floodFill(outputImage, seed, newVal, rect, loDiff, upDiff, flags);
}

/* http://opencv.jp/opencv-2.2_org/cpp/imgproc_miscellaneous_image_transformations.html#cv-threshold */
void getThresholdImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  // Get a gray image - quite a bit of vision processing is done on grayscale images
  cv::Mat grayImage;
  cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);

  int thresh = 128;
  double maxVal = 255;
  int thresholdType = CV_THRESH_BINARY;

  cv::threshold(grayImage, outputImage, thresh, maxVal, thresholdType);
}

/* http://opencv.jp/opencv-2.2_org/cpp/imgproc_image_filtering.html#cv-sobel */
void getErodeImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;
  cv::erode(inputImage, outputImage, element, anchor, iterations);
}

/* http://opencv.jp/opencv-2.2_org/cpp/imgproc_image_filtering.html#cv-sobel */
void getDilateImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  cv::Mat element(7, 7, CV_8U);
  cv::Point anchor(-1,-1);
  int iterations = 3;
  cv::dilate(inputImage, outputImage, element, anchor, iterations);
}

/* http://opencv.jp/opencv-2.2_org/cpp/imgproc_image_filtering.html#cv-sobel */
void getSobelImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  // Get a gray image - quite a bit of vision processing is done on grayscale images
  cv::Mat grayImage;
  cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);

  int ddepth = CV_8U; // output image is 8 bit unsigned
  int xorder = 1; // 1st derivative in x
  int yorder = 1; // 1st derivative in y
  int ksize = 5; // filter size is 5

  cv::Sobel(grayImage, outputImage, ddepth, xorder, yorder, ksize);
}

void getCannyImage(const cv::Mat& inputImage, cv::Mat& outputImage) {
  // Get a gray image - quite a bit of vision processing is done on grayscale images
  cv::Mat grayImage;
  cv::cvtColor(inputImage, grayImage, CV_RGB2GRAY);

  // Get an edge image - here we use the canny edge detection algorithm to get the edges
  double threshold1 = 20;
  double threshold2 = 50;
  int apertureSize = 3;

  // The smallest of threshold1 and threshold2 is used for edge linking, 
  // the largest - to find initial segments of strong edges.  Play around 
  // with these numbers to get desired result, and/or pre-process the 
  // image, e.g. clean up, sharpen, blur).
  cv::Canny(grayImage, outputImage, threshold1, threshold2, apertureSize);
}

void callFilter(const cv::Mat& inputImage, cv::Mat& outputImage) {
  switch (method_) {
    case CANNY: 
      getCannyImage(inputImage, outputImage); 
      break;
    case SOBEL: 
      getSobelImage(inputImage, outputImage); 
      break;
    case DILATE: 
      getDilateImage(inputImage, outputImage); 
      break;
    case ERODE: 
      getErodeImage(inputImage, outputImage);
      break;
    case FLOOD:
      getFloodFillImage(inputImage, outputImage);
      break;
    case THRESHOLD: 
      getThresholdImage(inputImage, outputImage);
      break;
  }
}

void processImage(const sensor_msgs::ImageConstPtr &msg) {
  // Get a reference to the image from the image message pointer
  cv_bridge::CvImageConstPtr imageMsgPtr = cv_bridge::toCvShare(msg, "bgr8");
  cv::Mat outputImage;
  callFilter(imageMsgPtr->image, outputImage);
  cv::imshow("Output", outputImage);
  cv::imshow("Input", imageMsgPtr->image);
}

/** 
 * get command line parameters using boost::program_options
 * http://www.boost.org/doc/libs/1_41_0/doc/html/program_options/tutorial.html#id1404217
 *
 * \returns 0 if successful
 */
/*
int getParameters(int argc, char *argv[]) {

  boost::program_options::options_description desc("Allowed options");
  desc.add_options()
      ("help", "produce help message")
      ("image", boost::program_options::value<std::string>(), "provide input image instead of subscribing to topic")
      ("method", boost::program_options::value<std::string>(), "choose filter/image manipulation ['canny','sobel','erode','dilate','flood','threshold']")
  ;

  boost::program_options::variables_map vm;
  boost::program_options::store(boost::program_options::parse_command_line(argc, argv, desc), vm);
  boost::program_options::notify(vm);   

  if (vm.count("help")) {
    std::cout << desc << std::endl;
    return 1;
  }

  if (!vm.count("method")) {
    std::cerr << "Method needs to be specified!!" << std::endl;
    std::cout << desc << std::endl;
    return 1;
  } else {
    std::string method(vm["method"].as<std::string>());
    if (method == "canny") {
      method_ = CANNY;
    } else if (method == "sobel") {
      method_ = SOBEL;
    } else if (method == "erode") {
      method_ = ERODE;
    } else if (method == "dilate") {
      method_ = DILATE;
    } else if (method == "flood") {
      method_ = FLOOD;
    } else if (method == "threshold") {
      method_ = THRESHOLD;
    } else {
      std::cerr << "Unknown method '" << method_ << "' specified!!" << std::endl;
      std::cout << desc << std::endl;
      return 1;
    }
  }

  if (vm.count("image")) {
    useStaticImage_ = true;
    staticImageFile_ = vm["image"].as<std::string>();
  }

  return 0;
}
*/

int main(int argc, char *argv[]) {
  ros::init(argc, argv, NODE);

  int mType = std::atoi(argv[1]);
  if (mType == 1)
    method_ = SOBEL;
  else if (mType == 2)
    method_ = DILATE;
  else if (mType == 3)
    method_ = ERODE;
  else if (mType == 4)
    method_ = FLOOD;
  else if (mType == 5)
    method_ = THRESHOLD;
  else
    method_ = CANNY;

  useStaticImage_ = false;
  staticImageFile_ = "data/apple.jpg";

  cv::namedWindow("Input");
  cvResizeWindow("Input", 320, 240);

  cv::namedWindow("Output");
  cvResizeWindow("Output", 320, 240);

  cvStartWindowThread();

  if (useStaticImage_) {

    cv::Mat inputImage(cv::imread(staticImageFile_));

    if (inputImage.data == NULL) {
      std::cerr << "Invalid input file '" << staticImageFile_ << "' provided!!" << std::endl;
      return -1;
    }

    cv::Mat outputImage;
    callFilter(inputImage, outputImage);

    cv::imshow("Input", inputImage);
    cv::imshow("Output", outputImage);

    cv::waitKey(0);
    return 0;
  }
  
  ros::NodeHandle node;
  image_transport::ImageTransport it(node);
  
  std::string image_topic = node.resolveName("usb_cam/image_raw");
  image_transport::Subscriber center_camera =	it.subscribe(image_topic, qDepth_, &processImage);

  ROS_INFO(NODE ": starting main loop");

  ros::spin();                          // handle incoming data

  ROS_INFO(NODE ": exiting main loop");

  return 0;
}
