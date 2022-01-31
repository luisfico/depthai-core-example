//todo:  save  image   and see cloud in real time    ,    use  ros,  use   rgbs pipeline2

//  example from  /depthai-core-example/depthai-core/examples/StereoDepth/rgb_depth_aligned.cpp

#include <cstdio>
#include <iostream>

//#include "utility.hpp"
#include "/home/lc/Dev/depthai-core-example/depthai-core/examples/utility/utility.hpp"

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


using namespace std::chrono_literals;
using pcl::visualization::PointCloudColorHandlerCustom;
typedef pcl::PointXYZ PointT;

std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  

//std::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
void simpleVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  //std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ>(cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (0.30, "global");
  viewer->initCameraParameters();
  //return (viewer);

  PCL_INFO ("Press q to begin the registration.\n");
  viewer->spin();
}

//TO ADD COLOR 
std::shared_ptr<pcl::visualization::PCLVisualizer>
Vis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud,pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloudB)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  std::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);//rgb red
  
  PointCloudColorHandlerCustom<PointT> tgt_h (cloud, 0, 255, 0);//green
  PointCloudColorHandlerCustom<PointT> src_h (cloudB, 255, 0, 0);//red
  
  viewer->addPointCloud<pcl::PointXYZ>(cloud, tgt_h, "sample cloud");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    
  viewer->addPointCloud<pcl::PointXYZ>(cloudB, src_h,"sample cloudB");
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloudB");
  
  viewer->addCoordinateSystem (0.6, "global");
  //viewer->setCameraPosition (double pos_x, double pos_y, double pos_z, double up_x, double up_y, double up_z, int viewport = 0);
  viewer->initCameraParameters();
  return (viewer);
}


//---------- //PCL_cloud_viewer  -----------------------end


// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_400_P;

static float rgbWeight = 0.6f;
static float depthWeight = 0.4f;

static void updateBlendWeights(int percentRgb, void *ctx)
{
    rgbWeight = float(percentRgb) / 100.f;
    depthWeight = 1.f - rgbWeight;
}

int main()
{

  //PCL_cloud_viewer
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  //pcl::io::loadPCDFile("/home/lc/fprojects/cpp/pcl/detectionLibViewer/cloudOrig.pcd", *cloud);    //for Debug

    std::ofstream file;
    //file.open("foo.csv");
  	//file.open("cloudDepth.csv", std::fstream::in | std::fstream::out | std::fstream::app);
    //file << "//X;Y;Z\n";

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
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
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

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
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


    //PCL_cloud_viewer 
    std::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    //viewer = Vis(cloud,cloudOut); //2 clouds
    //viewer = simpleVis(cloud); //1 cloud


    int cont=0;
    while (true)
    {
        //PCL_cloud_viewer 
        //viewer->spinOnce(100);
        //std::this_thread::sleep_for(100ms);



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


                    //-----Generation pointcloud-----------ini
                    cont++;
                    std::string numb_img= "/home/lc/Dev/depthai-core-example/build/tmp/cloudDepth_"+ to_string(cont) ;
                    std::string numb_imgname=numb_img + ".csv";
                    //file.open(numb_imgname, std::fstream::in | std::fstream::out | std::fstream::app);
                    //file << "//X;Y;Z\n";

                    auto disparity = frame["depth"].clone();
                    // Assuming  1280 x 720  default
                    //TODO: check this calib default!!!
                    //double fx = 788.936829, fy = 788.936829, cx = 660.262817, cy = 397.718628; //default
                    double fx = 857.1668, fy = 856.0823, cx = 643.9126, cy = 387.56018;//calib   rms 0.12219291207537852  file:///home/lc/Dev/calib1%20oak-d%20dataset/calib%20with%20monitor
                    //Problem cloud scale :  real  0.30/  generated 0.756   aprox factor  0.4 ???  
                    double factorFix=1; //0.4;
                    double baselineStereo = 0.075; // Stereo baseline distance: 7.5 cm
                    for (int v = 0; v < disparity.rows; v++)
                    {
                        for (int u = 0; u < disparity.cols; u++)
                        {

                            if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 96.0)
                            //if (disparity.at<float>(v, u) <= 0.0 || disparity.at<float>(v, u) >= 200.0)
                                continue;

                            // compute the depth from disparity
                            //double disparityD=disparity.at<float>(v, u);//ko
                            //double disparityD=disparity.at<double>(v, u); //ko
                            unsigned int disparityD = disparity.ptr<uint8_t>(v)[u]; //ok
                            //unsigned int disparityD = disparity.ptr<unsigned short>(v)[u];

                            double xNorm = (u - cx) / fx*factorFix;                        // x normalizado
                            double yNorm = (v - cy) / fy*factorFix;                        // y normalizado
                            double depth = fx * baselineStereo / (disparityD)*factorFix; //ok depth=z real = scala w
                            //double depth = fx * baselineStereo / (disparity.at<float>(v, u));//ko
                            // unsigned int d2 = img_depth.ptr<uint8_t>(400)[1000];

                            if (depth > 15.0)
                                continue;              // solo rescata los puntos con profundiad inferior a 15m
                            double xP = xNorm * depth; // x normalizado se escala y se recupera x real
                            double yP = yNorm * depth;
                            double zP = depth;

                            file << xP << ";" << yP << ";" << zP << "\n";
                            cloud->push_back(pcl::PointXYZ(xP,yP,zP));

                        }
                    }
                    //file.close();

                    if(!cloud->empty())
                    {
                        //pcl::io::savePCDFileASCII(numb_img+".pcd", *cloud); //for Debug
                        pcl::io::savePCDFileASCII("tmp/currentCloud.pcd", *cloud); //for Debug
                        simpleVis(cloud); //1 cloud
                        cloud->clear();
                    }
                    //-----Generation pointcloud-----------end



                    auto maxDisparity = stereo->initialConfig.getMaxDisparity();
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
            file.close();
            return 0;
        }
    }

    //file.close();
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