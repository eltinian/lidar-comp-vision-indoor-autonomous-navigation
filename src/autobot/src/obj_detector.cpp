/*
 * http://github.com/dusty-nv/jetson-inference
 */

// #include "gstCamera.h"
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <autobot/compound_img.h>
#include <autobot/detected_img.h>
#include <autobot/bounding_box.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <boost/make_shared.hpp>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <jetson-inference/cudaMappedMemory.h>
#include <jetson-inference/cudaNormalize.h>
#include <jetson-inference/cudaFont.h>

#include <jetson-inference/detectNet.h>


#define DEFAULT_CAMERA -1   // -1 for onboard camera, or change to index of /dev/video V4L2 camera (>=0)

using namespace std;

bool signal_recieved = false;

//void sig_handler(int signo)
//{
    //if( signo == SIGINT )
    //{
        //printf("received SIGINT\n");
        //signal_recieved = true;
    //}
//}

static const std::string OPENCV_WINDOW = "Image post bridge conversion";
static const std::string OPENCV_WINDOW2 = "Image post bit depth conversion";
static const std::string OPENCV_WINDOW3 = "Image post color conversion";
static const std::string OPENCV_WINDOW4 = "Image window";

class ObjectDetector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber image_sub_;
    ros::Publisher detect_img_pub;
    cv::Mat cv_im;
    cv_bridge::CvImagePtr cv_ptr;
    std::chrono::steady_clock::time_point prev;
    image_transport::Subscriber dummysub;
    float confidence = 0.0f;


    float* bbCPU    = NULL;
    float* bbCUDA   = NULL;
    float* confCPU  = NULL;
    float* confCUDA = NULL;

    detectNet* net = NULL;
    uint32_t maxBoxes = 0;
    uint32_t classes = 0;

    float4* gpu_data = NULL;

    uint32_t imgWidth;
    uint32_t imgHeight;
    size_t imgSize;
    bool displayToScreen = false;

public:
    ObjectDetector(int argc, char** argv, bool display) : it_(nh_)
    {
        // Subscrive to input video feed and publish output video feed
        // image_sub_ = it_.subscribe("/left/image_rect_color", 2,
        // &ObjectDetector::imageCb, this);
        image_sub_ = nh_.subscribe("/compound_img", 2,
          &ObjectDetector::imageCb, this);
        detect_img_pub = nh_.advertise<autobot::detected_img>("/detected_image", 2);
        dummysub = it_.subscribe("depth/depth_registered", 1, &ObjectDetector::dummy, this);
        displayToScreen = display;

        if (displayToScreen) {
            cv::namedWindow(OPENCV_WINDOW);
            cout << "Named a window" << endl;
            prev = std::chrono::steady_clock::now();
        }


        /*
         * create detectNet
         */
        net = detectNet::Create(argc, argv);
        cout << "Created DetectNet" << endl;

        if( !net )
        {
            printf("obj_detect:   failed to initialize imageNet\n");

        }
        
        maxBoxes = net->GetMaxBoundingBoxes();
        printf("maximum bounding boxes:  %u\n", maxBoxes);
        classes  = net->GetNumClasses();


        /*
         * allocate memory for output bounding boxes and class confidence
         */

        if( !cudaAllocMapped((void**)&bbCPU, (void**)&bbCUDA, maxBoxes * sizeof(float4)) ||
            !cudaAllocMapped((void**)&confCPU, (void**)&confCUDA, maxBoxes * classes * sizeof(float)) )
        {
            printf("detectnet-console:  failed to alloc output memory\n");

        }
        cout << "Allocated CUDA mem" << endl;


        maxBoxes = net->GetMaxBoundingBoxes();
        printf("maximum bounding boxes:  %u\n", maxBoxes);
        classes  = net->GetNumClasses();
        cout << "Constructor operations complete" << endl;
    }

    ~ObjectDetector()
    {
        if (displayToScreen) {
            cv::destroyWindow(OPENCV_WINDOW);
        }

    }
    
    void dummy(const sensor_msgs::ImageConstPtr& msg) {
        
    }

    void imageCb(const autobot::compound_img& cp_msg)
    {
        //ROS_INFO("callback called");
        //boost::shared_ptr<sensor_msgs::Image> cp_shared(&cp_msg.depthImg);
        //sensor_msgs::ImageConstPtr msg = cp_shared;

        try
        {
            cv_ptr = cv_bridge::toCvCopy(cp_msg.img, sensor_msgs::image_encodings::BGR8);
            cv_im = cv_ptr->image;
            
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
        if (!gpu_data) {
            ROS_INFO("first allocation");
            CUDA(cudaMalloc(&gpu_data, cv_im.rows*cv_im.cols * sizeof(float4)));
        } else if (imgHeight != cv_im.rows || imgWidth != cv_im.cols) {
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


        // find object with detectNet
        int numBoundingBoxes = maxBoxes;

        if (net->Detect((float*)gpu_data, imgWidth, imgHeight, bbCPU, &numBoundingBoxes, confCPU))
        {
            //printf("%i bounding boxes detected\n", numBoundingBoxes);

            int lastClass = 0;
            int lastStart = 0;

            for( int n=0; n < numBoundingBoxes; n++ )
            {
                const int nc = confCPU[n*2+1];
                float* bb = bbCPU + (n * 4);

                printf("bounding box %i   (%f, %f)  (%f, %f)  w=%f  h=%f\n", n, bb[0], bb[1], bb[2], bb[3], bb[2] - bb[0], bb[3] - bb[1]);
                //cv::rectangle( cv_im, cv::Point( bb[0], bb[1] ), cv::Point( bb[2], bb[3]), cv::Scalar( 255, 55, 0 ), +1, 4 );

            }

            
            if (displayToScreen) {
                std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
                float fps = 1000.0 / std::chrono::duration_cast<std::chrono::milliseconds>(now - prev).count() ;

                prev = now;
                
                char str[256];
                sprintf(str, "TensorRT build %x | %s | %04.1f FPS", NV_GIE_VERSION, net->HasFP16() ? "FP16" : "FP32", fps);
                cv::setWindowTitle(OPENCV_WINDOW, str);
            }

        }


        // update image back to original

        cv_im.convertTo(cv_im,CV_8UC3);
        cv::cvtColor(cv_im,cv_im,CV_RGBA2BGR);



        for( int n=0; n < numBoundingBoxes; n++ ) {
            
            const int nc = confCPU[n*2+1];
            float* bb = bbCPU + (n * 4);
            
            sensor_msgs::ImagePtr msg;
            
            float crop_width = bb[2] - bb[0];
            float crop_height = bb[3] - bb[1];
            float origin_x = bb[0];
            float origin_y = bb[1];
            
            printf("imgWidth: %u imgHeight: %u\n",imgWidth,imgHeight);
            printf("BEFORE: origin_x: %f origin_y: %f crop_width: %f crop_height: %f\n",origin_x, origin_y, crop_width, crop_height);
            
            if (crop_width < crop_height) {
                float diff = crop_height - crop_width;
                printf("diff: %f\n",diff);
                origin_x = origin_x - (diff / 2.0);
                crop_width = crop_height;

            } else if (crop_width > crop_height) {
                float diff = crop_width - crop_height;
                printf("diff: %f\n",diff);
                origin_y = origin_y - (diff / 2.0);
                crop_height = crop_width;
            }
            printf("MIDDLE: origin_x: %f origin_y: %f crop_width: %f crop_height: %f\n",origin_x, origin_y, crop_width, crop_height);

            
            if (origin_x < 0.0) {
                origin_x = 0.0;
            }
            
            if (origin_y < 0.0) {
                origin_y = 0.0;
            }
            
            if (origin_x + crop_width >= (float)imgWidth ) {
                crop_width = (float)imgWidth - origin_x - 1.0;
            }
            
            if (origin_y + crop_height >= (float)imgHeight) {
                crop_height = (float)imgHeight - origin_y - 1.0;
            }

            printf("AFTER : origin_x: %f origin_y: %f crop_width: %f crop_height: %f\n",origin_x, origin_y, crop_width, crop_height);
            cv::Mat croppedImage = cv_im(cv::Rect(origin_x, origin_y, crop_width, crop_height)) ;
            sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", croppedImage).toImageMsg();
            boost::shared_ptr<autobot::detected_img> detected_img = boost::make_shared<autobot::detected_img>();
            boost::shared_ptr<autobot::bounding_box> bbox = boost::make_shared<autobot::bounding_box>();
            
            bbox->origin_x = origin_x;
            bbox->origin_y = origin_y;
            bbox->height = crop_height;
            bbox->width = crop_width;
            
            detected_img->img = *img_msg.get();
            detected_img->depthImg = cp_msg.depthImg;
            detected_img->box = *bbox.get();

            detect_img_pub.publish<autobot::detected_img>(detected_img);

            if (displayToScreen) {
                //cv::imshow("crop", croppedImage);
                cv::waitKey(1);
            }
            
        }
        
        for( int n=0; n < numBoundingBoxes; n++ ) {
            float* bb = bbCPU + (n * 4);
            cv::rectangle( cv_im, cv::Point( bb[0], bb[1] ), cv::Point( bb[2], bb[3]), cv::Scalar( 255, 55, 0 ), +1, 4 );
        }
        
        // Update GUI Window
        if (displayToScreen) {
            cv::imshow(OPENCV_WINDOW, cv_im);
            cv::waitKey(1);
        }


    }
};

int main( int argc, char** argv ) {
    cout << "starting node" << endl;
    printf("obj_detect\n  args (%i):  ", argc);

    ros::init(argc, argv, "obj_detector");
    ros::NodeHandle nh;

    bool displayToScreen = false;

    for( int i=0; i < argc; i++ ) {
        printf("%i [%s]  ", i, argv[i]);
        if (strcmp(argv[i], "--DISPLAY") == 0) { 
            displayToScreen = true;
        }
    }

    printf("\n\n");

    ObjectDetector ic(argc, argv, displayToScreen);

    ros::spin();

    return 0;
}

