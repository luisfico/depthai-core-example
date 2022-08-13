/*BUILD:    cmake --build build --parallel -j$(nproc)
RUN:        ./build/myapp Q.xml 

TODO: it needs  getCameraExtrinsics() because pointcloud is with respect to right camera frame
    search extrensics  color - right

    move cloud from left to center

OR TO CALIBRATE EXTRENSICS ???

*/



// USAGE:   ./build/myapp Q.xml     generates   continous color clouds  cloudDepth_X.pcd

// todo:  save  image   and see cloud in real time    ,    use  ros,  use   rgbs pipeline2

//  example from  /depthai-core-example/depthai-core/examples/StereoDepth/rgb_depth_aligned.cpp

#include <cstdio>
#include <iostream>

//#include "utility.hpp"
//#include "/home/lc/Dev/depthai-core-example/depthai-core/examples/utility/utility.hpp"
//#include "/home/lc/env/oakd/codeCpp/depthai-core-example/depthai-core/examples/utility/utility.hpp" //ok
#include "../depthai-core/examples/utility/utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

#include <fstream>

//---------- //PCL_cloud_viewer  -----------------------ini

#include <pcl/io/pcd_io.h>

// for cloud viewer
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <thread>

#define CUSTOM_REPROJECT

using namespace std::chrono_literals;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ PointT;

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));

// std::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    // std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem(0.30, "global");
    viewer->initCameraParameters();
    // return (viewer);

    PCL_INFO("Press q to begin the registration.\n");
    viewer->spin();
}

// TO ADD COLOR
std::shared_ptr<pcl::visualization::PCLVisualizer>
Vis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud, pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudB)
{
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0); // rgb red

    PointCloudColorHandlerCustom<PointT> tgt_h(cloud, 0, 255, 0);  // green
    PointCloudColorHandlerCustom<PointT> src_h(cloudB, 255, 0, 0); // red

    viewer->addPointCloud<pcl::PointXYZ>(cloud, tgt_h, "sample cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");

    viewer->addPointCloud<pcl::PointXYZ>(cloudB, src_h, "sample cloudB");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudB");

    viewer->addCoordinateSystem(0.3, "global");
    // viewer->setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport = 0);
    viewer->initCameraParameters();
    return (viewer);
}

//---------- //PCL_cloud_viewer  -----------------------end

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{false};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;
// static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;

/* NOTE
                                         1280×720, 1280×800, 640×400,    640×480
enum class SensorResolution : int32_t { THE_720_P, THE_800_P, THE_400_P, THE_480_P };
*/

static float rgbWeight = 0.6f;
static float depthWeight = 0.4f;

static void updateBlendWeights(int percentRgb, void *ctx)
{
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}

void printMatrix(std::vector<std::vector<float>> matrix)
{
    using namespace std;
    std::string out = "[";
    for (auto row : matrix)
    {
        out += "[";
        for (auto val : row)
            out += to_string(val) + ", ";
        out = out.substr(0, out.size() - 2) + "]\n";
    }
    out = out.substr(0, out.size() - 1) + "]\n\n";
    cout << out;
}

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
static std::atomic<bool> extended_disparity{false};
// Better accuracy for longer distance, fractional disparity 32-levels:
static std::atomic<bool> subpixel{true};
// Better handling for occlusions:
static std::atomic<bool> lr_check{true};

int main(int argc, char **argv)
{

/*
Extrinsics from left->rgb test:
[[0.999989, 0.000000, -0.004777, -3.757788]
[0.000064, 0.999910, 0.013421, -0.029268]
[0.004777, -0.013421, 0.999898, -0.183094]
[0.000000, 0.000000, 0.000000, 1.000000]]

Extrinsics from rgb->right test:
[[0.999862, -0.016537, 0.001532, -3.721128]
[0.016547, 0.999836, -0.007300, -0.103135]
[-0.001411, 0.007325, 0.999972, 0.215810]
[0.000000, 0.000000, 0.000000, 1.000000]]
*/
    // Extrinsics from left->rgb test: cm to meters
    /*
        cv::Mat TF_LeftCamToColorCam = (cv::Mat_<double>(4, 4) << .999989, 0.000000, -0.004777, -0.03757788,
                                                    0.000064, 0.999910, 0.013421, -0.00029268,
                                                    0.004777, -0.013421, 0.999898, -0.00183094,
                                 				0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000);
        cv::Mat TF_RightCamToColorCam = (cv::Mat_<double>(4, 4) << 0.999862, -0.016537, 0.001532, -0.03721128,
                                                0.016547, 0.999836, -0.007300, -0.00103135,
                                                -0.001411, 0.007325, 0.999972, 0.00215810,
                                 				0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000);

Extrinsics from left->right test:
[[0.999857, -0.016571, -0.003245, -7.479905]
[0.016591, 0.999844, 0.006121, -0.129734]
[0.003143, -0.006174, 0.999976, 0.016303]
[0.000000, 0.000000, 0.000000, 1.000000]]
*/
        cv::Mat TF_LeftToRightCam = (cv::Mat_<double>(4, 4) << 0.999857, -0.016571, -0.003245, -7.479905,
                                                0.016591, 0.999844, 0.006121, -0.129734,
                                                0.003143, -0.006174, 0.999976, 0.016303,
                                                0.000000, 0.000000, 0.000000, 1.000000,
                                 				0.000000000000, 0.000000000000, 0.000000000000, 1.000000000000);

    // Load Matrix Q
    cv::FileStorage fs(argv[1], cv::FileStorage::READ);
    cv::Mat Q;

    fs["Q"] >> Q;

    // If size of Q is not 4x4 exit
    if (Q.cols != 4 || Q.rows != 4)
    {
        std::cerr << "ERROR: Could not read matrix Q (doesn't exist or size is not 4x4)" << std::endl;
        return 1;
    }

    // Get the interesting parameters from Q
    double Q03, Q13, Q23, Q32, Q33;
    Q03 = Q.at<double>(0, 3);
    Q13 = Q.at<double>(1, 3);
    Q23 = Q.at<double>(2, 3);
    Q32 = Q.at<double>(3, 2);
    Q33 = Q.at<double>(3, 3);

    std::cout << "Q(0,3) = " << Q03 << "; Q(1,3) = " << Q13 << "; Q(2,3) = " << Q23 << "; Q(3,2) = " << Q32 << "; Q(3,3) = " << Q33 << ";" << std::endl;

    std::cout << "Read matrix in file " << argv[1] << std::endl;

    // PCL_cloud_viewer
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPCDFile("/home/lc/fprojects/cpp/pcl/detectionLibViewer/cloudOrig.pcd", *cloud);    //for Debug

    // std::ofstream file;
    // file.open("foo.csv");
    // file.open("cloudDepth.csv", std::fstream::in | std::fstream::out | std::fstream::app);
    // file << "//X;Y;Z\n";

    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::RGB);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K); // color THE_1080_P
    camRgb->setFps(fps);
    if (downscaleColor)
        camRgb->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    camRgb->initialControl.setManualFocus(135);

    left->setResolution(monoRes);
    left->setBoardSocket(dai::CameraBoardSocket::LEFT);
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setBoardSocket(dai::CameraBoardSocket::RIGHT);
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_ACCURACY); // HIGH_ACCURACY,HIGH_DENSITY

    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    // stereo->initialConfig.setMedianFilter(dai::MedianFilter::MEDIAN_OFF);
    // stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    stereo->setLeftRightCheck(lr_check);
    stereo->setExtendedDisparity(extended_disparity);
    stereo->setSubpixel(subpixel);

    stereo->setDepthAlign(dai::CameraBoardSocket::RGB);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Sets queues size and behavior
    for (const auto &name : queueNames)
    {
        device.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;

    auto rgbWindowName = "rgb";
    auto depthWindowName = "depth";
    auto blendedWindowName = "rgb-depth";
    cv::namedWindow(rgbWindowName);
    cv::namedWindow(depthWindowName);
    cv::namedWindow(blendedWindowName);
    int defaultValue = (int)(rgbWeight * 100);
    cv::createTrackbar("RGB Weight %", blendedWindowName, &defaultValue, 100, updateBlendWeights);

    // PCL_cloud_viewer
    //std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    // viewer = Vis(cloud,cloudOut); //2 clouds
    // viewer = simpleVis(cloud); //1 cloud

    // Get calib
    dai::CalibrationHandler calibData = device.readCalibration();
    // calibData.eepromToJsonFile(filename);
    std::vector<std::vector<float>> intrinsics;
    int width, height;

    cout << "Intrinsics from defaultIntrinsics function:" << endl;
    std::tie(intrinsics, width, height) = calibData.getDefaultIntrinsics(dai::CameraBoardSocket::RIGHT);
    printMatrix(intrinsics);

    //cout << "Intrinsics from getCameraIntrinsics function 1280 x 720:" << endl;
    //intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RIGHT, 1280, 720);
    //intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::LEFT, 3840,2160);
    
    cout << "Intrinsics from getCameraIntrinsics function 4K 3840x2160 :" << endl;
    intrinsics = calibData.getCameraIntrinsics(dai::CameraBoardSocket::RGB, 3840,2160);
    
    printMatrix(intrinsics);

    int cont = 0;
    while (true)
    {

        // PCL_cloud_viewer
        // viewer->spinOnce(100);
        // std::this_thread::sleep_for(100ms);

        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = device.getQueueEvents(queueNames);
        for (const auto &name : queueEvents)
        {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if (count > 0)
            {
                latestPacket[name] = packets[count - 1];
            }
        }

        for (const auto &name : queueNames)
        {
            if (latestPacket.find(name) != latestPacket.end())
            {
                if (name == depthWindowName)
                {

                    frame[name] = latestPacket[name]->getFrame();





                    //-----Generation pointcloud with respect to left stereo frame-----------ini
                    cont++;
                    std::string numb_img = "/home/lc/env/oakd/codeCpp/depthai-core-example/tmp/cloudDepth_" + to_string(cont);
                    // std::string numb_imgColor= "/home/lc/env/oakd/codeCpp/depthai-core-example/tmp/img_"+ to_string(cont)+".png" ;
                    std::string numb_imgColor = "/home/lc/env/oakd/codeCpp/depthai-core-example/tmp/img_current.png";
                    //std::string numb_imgname = numb_img + ".csv";
                    // file.open(numb_imgname, std::fstream::in | std::fstream::out | std::fstream::app);
                    // file << "//X;Y;Z\n";

                    auto disparity = frame["depth"].clone();

                    // cv::imwrite("tmp/imgRgb.png", frame["rgb"]);
                    cv::imwrite(numb_imgColor, frame["rgb"]);
                    cv::imwrite("/home/lc/env/oakd/codeCpp/depthai-core-example/tmp/imgDisparity.pgm", frame["depth"]);

                    cv::Mat img_rgb = frame["rgb"].clone();
                    cv::Mat img_disparity = frame["depth"].clone();

                    // Assuming  1280 x 720  default
                    // TODO: check this calib default!!!
                    
                    /*
                     Intrinsics from getCameraIntrinsics color  function 4K 3840x2160 :
                    [[3090.419189, 0.000000, 1953.194824]
                    [0.000000, 3090.419189, 1068.689209]
                    [0.000000, 0.000000, 1.000000]]
                    */
                    //double fx = 3090.419189, fy = fx, cx = 1953.194824, cy = 1068.689209; // 4K color = 3840x2160 is (1280×720   x3times)  
                    double fx = 2366.810547, fy = fx, cx = 1980.788452, cy = 1073.155884; // 4K right = 3840x2160 is (1280×720   x3times)  
                    //double fx = 788.936829*3, fy = 788.936829*3, cx = 660.262817*3, cy = 397.718628*3; // 4K = 3840x2160 is (1280×720   x3times)  
                    //double fx = 788.936829, fy = 788.936829, cx = 660.262817, cy = 397.718628; // default  1280 x 800
                    
                    // double fx = 857.1668, fy = 856.0823, cx = 643.9126, cy = 387.56018;// 1280 x 800 calib   rms 0.12219291207537852  file:///home/lc/Dev/calib1%20oak-d%20dataset/calib%20with%20monitor
                    // double fx = 1042.20948, fy = 1040.51395, cx = 643.9126, cy = 387.56018;// 1280 x 720 calib   rms 0.016328653730143784 file:///home/lc/env/oakd/codeCpp/depthai-core-example/depthai-core/tmp%20to%20use/select/result%20fast%20calib
                    // Problem cloud scale :  real  0.30/  generated 0.756   aprox factor  0.4 ???     disparityD value is scaled by 16bits? so   real disparityD= disparityD/16bits ?
                    
                    
                    //double scaleToShow=0.4; //for this example  for 0-2.5m is <0;1>       scale 0.4=1 max/2.5m max
        

                    double factorFix = 0.4;// 1         // 720/400; //720/400; // 1080/720      0.4; //1000; //0.4;  // upscale   THE_400_P to THE_720_P
                    double baselineStereo = 0.075; // Stereo baseline distance: 7.5 cm
                    for (int v = 0; v < disparity.rows; v++)
                    {
                        for (int u = 0; u < disparity.cols; u++)
                        {

                            // if (disparity.at<uint8_t>(v, u) <= 0.0 || disparity.at<uint8_t>(v, u) >= 96.0)    //ok
                            // if (disparity.at<uint8_t>(v, u) <= 0.0 || disparity.at<uint8_t>(v, u) >= 200.0)
                            //     continue;

                            // compute the depth from disparity
                            // double disparityD=disparity.at<float>(v, u);//ko
                            // double disparityD=disparity.at<double>(v, u); //ko
                            // double disparityD = disparity.ptr<float>(v)[u]; //ko
                            // double disparityD = disparity.ptr<unsigned short>(v)[u]; //ko

                            // unsigned int disparityD = disparity.ptr<uint8_t>(v)[u]; //ok
                            double disparityD = disparity.ptr<uchar>(v)[u]; // ok!!   disparityD value is scaled by 16bits? so   real disparityD= disparityD/16bits ?
                            // double disparityD = disparity.ptr<uint8_t>(v)[u]; //ok!!   disparityD value is scaled by 16bits? so   real disparityD= disparityD/16bits ?
                            // double disparityD = disparity.ptr<uint16_t>(v)[u];
                            // double disparityD = disparity.ptr<uint32_t>(v)[u];
                            // uchar dispValue = disparity.ptr<uchar>(v)[u];  double disparityD = static_cast<double>(dispValue);
                            if (disparityD <= 0.0)
                                continue;

                            double xNorm = (u - cx) / fx;                                // x normalizado
                            double yNorm = (v - cy) / fy;                                // y normalizado
                            double depth = fx * baselineStereo / (disparityD)*factorFix; // ok depth=z real = scala w
                            // double depth = fx * baselineStereo / (disparity.at<float>(v, u));//ko
                            // double depth = fx * baselineStereo / disparity.ptr<float>(v)[u];//ko
                            // unsigned int d2 = img_depth.ptr<uint8_t>(400)[1000];

                            if (depth > 15.0)
                                continue;              // solo rescata los puntos con profundiad inferior a 15m
                            double xP = xNorm * depth; // x normalizado se escala y se recupera x real
                            double yP = yNorm * depth;
                            double zP = depth;

                            // file << xP << ";" << yP << ";" << zP << "\n";
                            cloud->push_back(pcl::PointXYZ(xP, yP, zP));
                        }
                    }
                    // file.close();

                    if (!cloud->empty())
                    {
                        pcl::io::savePCDFileASCII(numb_img + ".pcd", *cloud); // for Debug
                        // pcl::io::savePCDFileASCII("tmp/currentCloud.pcd", *cloud); //for Debug
                        // simpleVis(cloud); //1 cloud
                        cloud->clear();
                    }
                    //-----Generation pointcloud-----------end

                    if (!img_rgb.empty() && !img_disparity.empty())
                    {


                        /*
                        Extrinsics from left->rgb test:
                        [[0.999989, 0.000000, -0.004777, -3.757788]
                        [0.000064, 0.999910, 0.013421, -0.029268]
                        [0.004777, -0.013421, 0.999898, -0.183094]
                        [0.000000, 0.000000, 0.000000, 1.000000]]
                        */
                    
                        // Create point cloud and fill it          frema left
                        std::cout << "Creating Point Cloud..." << std::endl;
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

                        double px, py, pz;
                        uchar pr, pg, pb;

                        for (int i = 0; i < img_rgb.rows; i++)
                        {
                            uchar *rgb_ptr = img_rgb.ptr<uchar>(i);
#ifdef CUSTOM_REPROJECT
                            uchar *disp_ptr = img_disparity.ptr<uchar>(i);
                            // uint8_t* disp_ptr = img_disparity.ptr<uint8_t>(i);//   /2
                            // uint16_t* disp_ptr = img_disparity.ptr<uint16_t>(i);//   /4
                            // uint32_t* disp_ptr = img_disparity.ptr<uint32_t>(i);  //  /2

#else
                            double *recons_ptr = recons3D.ptr<double>(i);
#endif
                            for (int j = 0; j < img_rgb.cols; j++)
                            {
                                // Get 3D coordinates
#ifdef CUSTOM_REPROJECT
                                uchar d = disp_ptr[j];
                                // uint8_t d = disp_ptr[j];//   /8
                                // uint16_t d = disp_ptr[j];
                                // uint32_t d = disp_ptr[j];
                                if (d == 0)
                                    continue; // Discard bad pixels
                                double dDebug = static_cast<double>(d);
                                // std::cout<<"d:"<<d<<std::endl;
                                // std::cout<<"dDebug:"<<dDebug<<std::endl;

                                double pw = (-1.0 * static_cast<double>(d) * Q32 + Q33) /factorFix; // --disparity/baseline      /2
                                px = static_cast<double>(j) + Q03;
                                py = static_cast<double>(i) + Q13;
                                pz = Q23 ; // focus 

                                px = px / pw;
                                py = py / pw;
                                pz = pz / pw; // +focus*baseline/disparity

#else
                                px = recons_ptr[3 * j];
                                py = recons_ptr[3 * j + 1];
                                pz = recons_ptr[3 * j + 2];
#endif


                                //Fix cloud from left frame to center color frame
                                cv::Mat pt3dCamStereo = (cv::Mat_<double>(4, 1) << px, py, pz, 1);

                                // Transf Pt3d
                                //cv::Mat pt3dCvTransf = TF_LeftCamToColorCam.inv() * pt3dCamStereo;
                                //cv::Mat pt3dCvTransf = TF_RightCamToColorCam*pt3dCamStereo;
                                cv::Mat pt3dCvTransf = TF_LeftToRightCam*pt3dCamStereo;
                                
                                
                                // cloud_out_transf.push_back(pt3dCvTransf);

                                // Project tranformed Pt3d  to cameraVP    find x,y    assign the depth value of double(d)
                                px = pt3dCvTransf.at<double>(0);
                                py = pt3dCvTransf.at<double>(1);
                                pz = pt3dCvTransf.at<double>(2);





                                // Get RGB info
                                pb = rgb_ptr[3 * j];
                                pg = rgb_ptr[3 * j + 1];
                                pr = rgb_ptr[3 * j + 2];

                                // Insert info into point cloud structure
                                pcl::PointXYZRGB point;
                                point.x = px;
                                point.y = py;
                                point.z = pz;
                                uint32_t rgb = (static_cast<uint32_t>(pr) << 16 |
                                                static_cast<uint32_t>(pg) << 8 | static_cast<uint32_t>(pb));
                                point.rgb = *reinterpret_cast<float *>(&rgb);
                                point_cloud_ptr->points.push_back(point);
                            }
                        }
                        point_cloud_ptr->width = (int)point_cloud_ptr->points.size();
                        point_cloud_ptr->height = 1;

                        if (!point_cloud_ptr->empty())
                            pcl::io::savePCDFileASCII(numb_img + "_color.pcd", *point_cloud_ptr); // for Debug
                        // pcl::io::savePCDFileASCII("/home/lc/env/oakd/codeCpp/depthai-core-example/depthai-core/tmp/cloudOut.pcd", *point_cloud_ptr); //for Debug
                    }

                    auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                    std::cout << "maxDisparity: " << maxDisparity << std::endl; // 95 o 190
                    // Optional, extend range 0..95 -> 0..255, for a better visualisation

                    if (1)
                        frame[name].convertTo(frame[name], CV_8UC1, 255. / maxDisparity);
                    // Optional, apply false colorization
                    if (1)
                        cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_HOT);
                }
                else
                {
                    frame[name] = latestPacket[name]->getCvFrame();
                }

                cv::imshow(name, frame[name]);
                cv::imwrite("/home/lc/env/oakd/codeCpp/depthai-core-example/tmp/" + std::to_string(cont) + name + ".png", frame[name]);
            }
        }

        // Blend when both received
        if (frame.find(rgbWindowName) != frame.end() && frame.find(depthWindowName) != frame.end())
        {

            // Need to have both frames in BGR format before blending
            if (frame[depthWindowName].channels() < 3)
            {
                cv::cvtColor(frame[depthWindowName], frame[depthWindowName], cv::COLOR_GRAY2BGR);
            }
            cv::Mat blended;
            cv::addWeighted(frame[rgbWindowName], rgbWeight, frame[depthWindowName], depthWeight, 0, blended);
            cv::imshow(blendedWindowName, blended);
            frame.clear();
        }

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q')
        {
            // file.close();
            return 0;
        }
    }

    // file.close();
    return 0;
}

/*

lc@lc-px ~/Dev/depthai-core-example/depthai-core/build/examples ((HEAD détachée sur 9710c727))$ ./calibration_reader
Intrinsics from defaultIntrinsics function:
[[788.936829, 0.000000, 660.262817]
[0.000000, 788.936829, 397.718628]
[0.000000, 0.000000, 1.000000]]

Width: 1280
Height: 800
Stereo baseline distance: 7.5 cm
Mono FOV from camera specs: 71.86, calculated FOV: 77.5185
Intrinsics from getCameraIntrinsics function full resolution:
[[788.936829, 0.000000, 660.262817]
[0.000000, 788.936829, 397.718628]
[0.000000, 0.000000, 1.000000]]

Intrinsics from getCameraIntrinsics function 1280 x 720:
[[788.936829, 0.000000, 660.262817]
[0.000000, 788.936829, 357.718628]
[0.000000, 0.000000, 1.000000]]

Intrinsics from getCameraIntrinsics function 720 x 450:
[[443.776978, 0.000000, 371.397827]
[0.000000, 443.776978, 223.716736]
[0.000000, 0.000000, 1.000000]]

Intrinsics from getCameraIntrinsics function 600 x 1280:
[[1262.298950, 0.000000, 332.420532]
[0.000000, 1262.298950, 636.349792]
[0.000000, 0.000000, 1.000000]]

Extrinsics from left->right test:
[[0.999857, -0.016571, -0.003245, -7.479905]
[0.016591, 0.999844, 0.006121, -0.129734]
[0.003143, -0.006174, 0.999976, 0.016303]
[0.000000, 0.000000, 0.000000, 1.000000]]

Extrinsics from right->left test:
[[0.999857, 0.016591, 0.003143, 7.480940]
[-0.016571, 0.999844, -0.006174, 0.005864]
[-0.003245, 0.006121, 0.999976, -0.039781]
[0.000000, 0.000000, 0.000000, 1.000000]]

Extrinsics from right->rgb test:
[[0.999862, 0.016547, -0.001411, 3.722626]
[-0.016537, 0.999836, 0.007325, 0.040002]
[0.001532, -0.007300, 0.999972, -0.210855]
[0.000000, 0.000000, 0.000000, 1.000000]]

Extrinsics from rgb->right test:
[[0.999862, -0.016537, 0.001532, -3.721128]
[0.016547, 0.999836, -0.007300, -0.103135]
[-0.001411, 0.007325, 0.999972, 0.215810]
[0.000000, 0.000000, 0.000000, 1.000000]]

Extrinsics from left->rgb test:
[[0.999989, 0.000000, -0.004777, -3.757788]
[0.000064, 0.999910, 0.013421, -0.029268]
[0.004777, -0.013421, 0.999898, -0.183094]
[0.000000, 0.000000, 0.000000, 1.000000]]


*/

                    /*
                    Intrinsics from getCameraIntrinsics function 4K 3840x2160 (LEFT):
                    [[2391.479004, 0.000000, 1924.136475]
                    [0.000000, 2391.479004, 1050.397217]
                    [0.000000, 0.000000, 1.000000]]

                    Intrinsics from getCameraIntrinsics function 4K 3840x2160 (RIGHT):
                    [[2366.810547, 0.000000, 1980.788452]
                    [0.000000, 2366.810547, 1073.155884]
                    [0.000000, 0.000000, 1.000000]]

                    Intrinsics from getCameraIntrinsics color  function 4K 3840x2160 :
                    [[3090.419189, 0.000000, 1953.194824]
                    [0.000000, 3090.419189, 1068.689209]
                    [0.000000, 0.000000, 1.000000]]

                    */