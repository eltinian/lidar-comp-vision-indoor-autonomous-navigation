#include <opencv2/dnn.hpp>
#include <fstream>
using namespace cv::dnn;

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>
#include <iostream>
#include <unistd.h>
#include <zed/Camera.hpp>;

using namespace std;
using namespace cv;

static void help()
{
    cout << "\nThis program can test the performance of a trained caffe model on object recognition with the built in camera.\n"
            "Usage:\n"
            "./stopdetect [--model-bin=<trained_model_path> path of trained caffe model.]\n"
               "   [--modeltxt=<model_definition_path>] Path of prototxt file used to define the model.\n"
               "   [--scale=<image scale greater or equal to 1, try 1.3 for example>]\n"
               // "   [--try-flip]\n"
               "   [filename|camera_index]\n\n"
            "see facedetect.cmd for one call:\n"
            "\tUsing OpenCV version " << CV_VERSION << "\n" << endl;
}

/* Find best class for the blob (i. e. class with maximal probability) */
void getMaxClass(dnn::Blob &probBlob, int *classId, double *classProb);

std::vector<String> readClassNames(const char *filename = "autobot_words.txt");
// std::vector<String> readClassNames(const char *filename = "synset_words.txt");
void detectAndDraw( Mat& img, double scale);

String camType;
int Resolution;

//! [Initialize network]
dnn::Net net;

int main( int argc, const char** argv )
{


    cv::ocl::setUseOpenCL(false);

    VideoCapture capture;
    Mat frame, image;
    string inputName;
    bool tryflip;
    CascadeClassifier stopCascade, crossCascade;
    double scale;

    cv::CommandLineParser parser(argc, argv,
        "{help h||}"
        "{cameraType|CV}"
        "{Resolution|256|}"
    );

    if (parser.has("help"))
    {
        help();
        return 0;
    }

    // grab CL arguments
    camType = parser.get<string>("cameraType");
    Resolution = atoi(parser.get<string>("Resolution").cstr());



    //! [Create the importer of Caffe model]
    Ptr<dnn::Importer> importer;
    try                                     //Try to import Caffe GoogleNet model
    {
        importer = dnn::createCaffeImporter(modelTxt, modelBin);
    }
    catch (const cv::Exception &err)        //Importer can throw errors, we will catch them
    {
        std::cerr << err.msg << std::endl;
        cout << "What?" << endl;
    }

    // set the frame size
    capture.set(CV_CAP_PROP_FRAME_WIDTH,800);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,600);

    int c = inputName.empty() ? 0 : inputName[0] - '0';
    if(!capture.open(c))
        cout << "Capture from camera #" <<  c << " didn't work" << endl;


    if( capture.isOpened() )
    {
        cout << "Video capturing has been started ..." << endl;

        for(;;)
        {
            // printf("start of loop\n");
            capture >> frame;
            if( frame.empty() ) {
                printf("empty frame\n");
                break;
            }

            Mat frame1 = frame.clone();
            detectAndDraw( frame1, scale);
            imshow( "result", frame1 );

            // printf("end of loop before break\n");
            int c = waitKey(10);
            if( c == 27 || c == 'q' || c == 'Q' )
                break;
            // printf("end of loop\n");
            // usleep(200000);
        }
    }
    return 0;
}

void detectAndDraw( Mat& img, double scale)
{

    // DNN CAFE ADDITONS
    // ------------------------------------------------

    //! [Prepare blob]
    if (img.empty())
    {
        std::cerr << "Can't read image" << std::endl;
        exit(-1);
    }

    // resize(img, img, Size(256, 256));       //GoogLeNet accepts only 224x224 RGB-images
    resize(img, img, Size(224, 224));       //GoogLeNet accepts only 224x224 RGB-images
    static double t = 0;

    t = (double)getTickCount();

    dnn::Blob inputBlob = dnn::Blob::fromImages(img);   //Convert Mat to dnn::Blob image batch
    //! [Prepare blob]


    //! [Set input blob]
    net.setBlob(".data", inputBlob);        //set the network input
    //! [Set input blob]

    //! [Make forward pass]
    net.forward();                          //compute output
    //! [Make forward pass]

    //! [Gather output]
    dnn::Blob prob = net.getBlob("prob");   //gather output of "<output layer name>" layer

    int classId;
    double classProb;
    getMaxClass(prob, &classId, &classProb);//find the best class
    //! [Gather output]

    // get time for classification
    t = (double)getTickCount() - t;

    //! [Print results]
    std::vector<String> classNames = readClassNames();
    std::cout << "Best class: #" << classId << " '" << classNames.at(classId) << "'" << std::endl;
    std::cout << "Probability: " << classProb * 100 << "%" << std::endl;
    //! [Print results]

    // ----------------

    // vector<Rect> stopSigns;
    static Scalar classColor = Scalar(0, 0, 255);
    Mat gray, smallImg;

    // cvtColor( img, gray, COLOR_BGR2GRAY );
    // double fx = 1 / scale;
    // resize( gray, smallImg, Size(), fx, fx, INTER_LINEAR );
    // equalizeHist( smallImg, smallImg );

    // stopCascade.detectMultiScale( smallImg, stopSigns,
    //     1.1, 2, 0
    //     //|CASCADE_FIND_BIGGEST_OBJECT
    //     //|CASCADE_DO_ROUGH_SEARCH
    //     |CASCADE_SCALE_IMAGE,
    //     Size(30, 30) );
    printf( "detection time = %g ms\n", t*1000/getTickFrequency());

    // mark stop signs in the frame
    // for ( size_t i = 0; i < stopSigns.size(); i++ )
    // {
    //     Rect r = stopSigns[i];
    //     Point center;
    //     int radius;

    //     Point textOrg;
    //     textOrg.x = r.x;
    //     textOrg.y = r.y;
    //     center.x = cvRound((r.x + r.width*0.5)*scale);
    //     center.y = cvRound((r.y + r.height*0.5)*scale);
    //     radius = cvRound((r.width + r.height)*0.25*scale);
    //     circle( img, center, radius, stopColor, 3, 8, 0 );
    // }
    putText( img, (classProb > 0.6)? classNames.at(classId) : String("No Match"), Point(20,20), FONT_HERSHEY_SIMPLEX, 1.0, classColor, 2);

    imshow( "result", img );
}

/* Find best class for the blob (i. e. class with maximal probability) */
void getMaxClass(dnn::Blob &probBlob, int *classId, double *classProb)
{
    Mat probMat = probBlob.matRefConst().reshape(1, 1); //reshape the blob to 1x1000 matrix
    Point classNumber;

    minMaxLoc(probMat, NULL, classProb, NULL, &classNumber);
    *classId = classNumber.x;
}

std::vector<String> readClassNames(const char *filename)
{
    std::vector<String> classNames;

    std::ifstream fp(filename);
    if (!fp.is_open())
    {
        std::cerr << "File with classes labels not found: " << filename << std::endl;
        exit(-1);
    }

    std::string name;
    while (!fp.eof())
    {
        std::getline(fp, name);
        if (name.length())
            classNames.push_back( name.substr(name.find(' ')+1) );
    }

    fp.close();
    return classNames;
}