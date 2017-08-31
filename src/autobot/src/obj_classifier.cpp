/*
 * http://github.com/dusty-nv/jetson-inference
 */

// #include "gstCamera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <autobot/detected_img.h>
#include <autobot/detected_object.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <jetson-inference/cudaMappedMemory.h>
#include <jetson-inference/cudaNormalize.h>
#include <jetson-inference/cudaFont.h>

#include <jetson-inference/imageNet.h>


#define DEFAULT_CAMERA -1   // -1 for onboard camera, or change to index of /dev/video V4L2 camera (>=0)

using namespace std;

bool signal_recieved = false;

/*
void sig_handler(int signo)
{
    if( signo == SIGINT )
    {
        //printf("received SIGINT\n");
        //signal_recieved = true;
    }
}
*/

void depthToCV8UC1(const cv::Mat& float_img, cv::Mat& mono8_img){
  //Process images
  if(mono8_img.rows != float_img.rows || mono8_img.cols != float_img.cols){
    mono8_img = cv::Mat(float_img.size(), CV_8UC1);
  }
  cv::convertScaleAbs(float_img, mono8_img, 100, 0.0);
}

static const std::string OPENCV_WINDOW = "Image post bridge conversion";
static const std::string OPENCV_WINDOW2 = "Image post bit depth conversion";
static const std::string OPENCV_WINDOW3 = "Image post color conversion";
static const std::string OPENCV_WINDOW4 = "Image window";

class ObjectClassifier
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    ros::Publisher class_pub_;
    cv::Mat cv_im;
    cv_bridge::CvImagePtr cv_ptr;
    std::chrono::steady_clock::time_point prev;
    cv::Mat cv_depth_im;
    cv::Mat cv_depth;
    float confidence = 0.0f;

    //float* bbCPU    = NULL;
    //float* bbCUDA   = NULL;
    //float* confCPU  = NULL;
    //float* confCUDA = NULL;
    
    imageNet* net = NULL;
    //detectNet* net = NULL;
    //uint32_t maxBoxes = 0;
    //uint32_t classes = 0;

    float4* gpu_data = NULL;

    uint32_t imgWidth;
    uint32_t imgHeight;
    size_t imgSize;

    bool displayToScreen = false;

public:
    ObjectClassifier(int argc, char** argv, bool display) : it_(nh_)
    {
        cout << "start constructor" << endl;
        // Subscrive to input video feed and publish output video feed
        //image_sub_ = it_.subscribe("/left/image_rect_color", 2,
          //&ObjectClassifier::imageCb, this);
        image_sub_ = nh_.subscribe("/detected_image", 2,
          &ObjectClassifier::imageCb, this);
        class_pub_ = nh_.advertise<autobot::detected_object>("/detected_object", 2);

        displayToScreen = display;

        if (displayToScreen) {
            cv::namedWindow(OPENCV_WINDOW);
            cout << "Named a window" << endl;
            prev = std::chrono::steady_clock::now();
        }

         // create imageNet
        net = imageNet::Create(argc, argv );


        if( !net )
        {
            printf("obj_detect:   failed to initialize imageNet\n");
        }
    }

    ~ObjectClassifier()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const autobot::detected_img& detect_msg)
    {
        // Conversion is now offloaded to navigator.py node
        // cv_bridge::CvImagePtr cv_depth_ptr;
        // cv_depth_ptr = cv_bridge::toCvCopy(detect_msg.depthImg);
        // cv_depth = cv_depth_ptr->image;
        // depthToCV8UC1(cv_depth, cv_depth_im);
        // cv::imshow("depth",cv_depth_im);
        int width;
        int height;
         
        try
        {
            
            cv_ptr = cv_bridge::toCvCopy(detect_msg.img, sensor_msgs::image_encodings::BGR8);
            cv_im = cv_ptr->image;
            width = cv_im.cols;
            height = cv_im.rows;
            cv::resize(cv_im,cv_im,cv::Size(224,224));
            cv_im.convertTo(cv_im,CV_32FC3);

            // convert color
            cv::cvtColor(cv_im,cv_im,CV_BGR2RGBA);

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }


        // allocate GPU data if necessary
        if(!gpu_data){
            ROS_INFO("first allocation");
            CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
        } else if(imgHeight != cv_im.rows || imgWidth != cv_im.cols){
            ROS_INFO("re allocation");
            // reallocate for a new image size if necessary
            CUDA(cudaFree(gpu_data));
            CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
        }

        imgHeight = cv_im.rows;
        imgWidth = cv_im.cols;
        imgSize = cv_im.rows*cv_im.cols * sizeof(float4);
        float4* cpu_data = (float4*)(cv_im.data);

        // copy to device
        CUDA(cudaMemcpy(gpu_data, cpu_data, imgSize, cudaMemcpyHostToDevice));

        void* imgCUDA = NULL;

        // classify image
        const int img_class = net->Classify((float*)gpu_data, imgWidth, imgHeight, &confidence);
        
        // find class string
        std::string class_name;
        class_name = net->GetClassDesc(img_class);
        
        
        if (displayToScreen) {
            std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
            float fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count() ;

            prev = now;
            
            char str[256];
            sprintf(str, "TensorRT build %x | %s | %04.1f FPS", NV_GIE_VERSION, net->HasFP16() ? "FP16" : "FP32", fps);
            cv::setWindowTitle(OPENCV_WINDOW, str);
        }

        sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cv_depth_im).toImageMsg();

        autobot::detected_object detected_object;
        detected_object.className = class_name;
        detected_object.depthImg = detect_msg.depthImg;
        detected_object.box = detect_msg.box;

        class_pub_.publish<autobot::detected_object>(detected_object);
        // update image back to original

        if (displayToScreen) {
            cv_im.convertTo(cv_im,CV_8UC3);
            cv::cvtColor(cv_im,cv_im,CV_RGBA2BGR);
            cv::resize(cv_im,cv_im,cv::Size(width,height));
            // draw class string
            cv::putText(cv_im, class_name, cv::Point(60,60), cv::FONT_HERSHEY_PLAIN, 3.0, cv::Scalar(0,0,255),3);
            
            // Update GUI Window
            cv::imshow(OPENCV_WINDOW, cv_im);
            cv::waitKey(1);
        }
    }
};

int main( int argc, char** argv ) {
    cout << "starting node" << endl;
    printf("obj_classifier\n  args (%i):  ", argc);

    ros::init(argc, argv, "obj_detector");
    ros::NodeHandle nh;

    bool display = false;
    for( int i=0; i < argc; i++ ) {
        printf("%i [%s]  ", i, argv[i]);
        if (strcmp(argv[i], "--DISPLAY") == 0) {
            display = true;
        }
    }

    printf("\n\n");

    ObjectClassifier ic(argc, argv, display);

    ros::spin();

    return 0;
}

