#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

public:
  ImageConverter() : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
   //I'm guessing this is where my errors are
//    image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);

    image_sub_ = it_.subscribe("camera/rgb/image_color", 1, &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise("/image_converter/output_video", 1);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Draw an example circle on the video stream
    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);

    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
   ros::init(argc, argv, "image_converter");
   ROS_INFO_STREAM("test to see if node is running");
  ImageConverter ic;
  ros::spin();
  return 0;
}









//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";

//class ImageConverter
//{
//    ros::NodeHandle nh;
//    image_transport::ImageTransport it;
//    image_transport::Subscriber image_sub;
//    image_transport::Publisher image_pub;

//public:
//    ImageConverter() : it(nh)
//    {
//        // subscribe to input video feed and publish
//        // output video feed
//        image_sub = it.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
//        image_pub = it.advertise("/image_converter/output_video", 1);
//        cv::namedWindow(OPENCV_WINDOW);
//    }
//    ~ImageConverter()
//    {
//        cv::destroyWindow(OPENCV_WINDOW);
//    }

//    void imageCb(const sensor_msgs::ImageConstPtr& msg)
//    {
//        cv_bridge::CvImage cv_ptr;
//        try
//        {
//            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
////            cv::Mat image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
//        } catch (cv_bridge::Exception& exception) {
//            ROS_ERROR("cv_bridge exception: %s", exception.what());
//            return;
//        }

//        // Draw an example circle on the video stream
//        if(cv_ptr.image.rows > 60 && cv_ptr.image.cols > 60)
//            cv::circle(cv_ptr.image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));

//        // Update GUI Window
//        cv::imshow(OPENCV_WINDOW, cv_ptr.image);
//        cv::waitKey(3);

//        // Output modified video stream
//        image_pub.publish(cv_ptr.toImageMsg());
//    }
//};

//int main(int argc, char **argv)
//{

//  ros::init(argc, argv, "image_converter");
//  ros::NodeHandle n;
//  ImageConverter ic;

//  //Create a publisher for the image
//  ros::Publisher image_pub = n.advertise<sensor_msgs::Image>("image", 1);
//  //Publish-rate [Hz]
//  ros::Rate loop_rate(1);
//  //Create ROS subscriber for the input images
//  ros::Subscriber image_sub;
//  image_sub = it.subscribe("camera/rgb/image_color", 1, &ImageConverter::imageCb, this);
//  int count = 0;
//   while (ros::ok())
//   {
//     if (cv_ptr)
//     {
//         //Here is usually your stitching-routine.
//        // Error won't occur here:
//        image1 = cv_ptr->image;     //Load the cv_bridge image into an OpenCV Matrix

//        //Convert cv::Mat stitched to ROS image
//        image_pub.publish(stitching_output);
//        // after you have processed the image your might want to release them
//        // so your node does not process the same images multiple times
//        // reset the ptrs and wait for the callbacks to fill them again
//        cv_ptr.reset();
//     }
//     else
//     {
//        // nothing can be done here; we have to spin and wait for images to arrive
//     }
//     ros::spinOnce();
//     loop_rate.sleep();
//     ++count;
//   }
//  return 0;
//}

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "image_converter");
//    ImageConverter ic;
//    ros::spin();
//    return 0;
//}


//#include <ros/ros.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>

//static const std::string OPENCV_WINDOW = "Image window";

//class ImageConverter
//{
//  ros::NodeHandle nh_;
//  image_transport::ImageTransport it_;
//  image_transport::Subscriber image_sub_;
//  image_transport::Publisher image_pub_;

//public:
//  ImageConverter()
//    : it_(nh_)
//  {
//    // Subscrive to input video feed and publish output video feed
//    image_sub_ = it_.subscribe("/camera/image_raw", 1,
//      &ImageConverter::imageCb, this);
//    image_pub_ = it_.advertise("/image_converter/output_video", 1);

//    cv::namedWindow(OPENCV_WINDOW);
//  }

//  ~ImageConverter()
//  {
//    cv::destroyWindow(OPENCV_WINDOW);
//  }

//  void imageCb(const sensor_msgs::ImageConstPtr& msg)
//  {
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//      ROS_ERROR("cv_bridge exception: %s", e.what());
//      return;
//    }

//    // Draw an example circle on the video stream
//    if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)
//      cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255,0,0));

//    // Update GUI Window
//    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
//    cv::waitKey(3);

//    // Output modified video stream
//    image_pub_.publish(cv_ptr->toImageMsg());
//  }
//};

//int main(int argc, char** argv)
//{
//  ros::init(argc, argv, "image_converter");
//  ImageConverter ic;
//  ros::spin();
//  return 0;
//}




















