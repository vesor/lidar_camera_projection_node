
// Weizhe Liu, July 2018

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//#include "../build_external/lib_camera.hpp"

#include <opencv2/opencv.hpp>

#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>


#include <algorithm>

#include <sstream>
#include <memory>
#include <chrono>
#include <thread>
#include <mutex>

static bool cmdOptionExists(char** begin, char** end, const std::string& option)
{
    return std::find(begin, end, option) != end;
}

static cv::Mat_<float> rotationByAxis(const cv::Mat_<float>& axis, float theta) {
    cv::Mat_<float> r(1, 3);
    r = axis / cv::norm(axis) * theta;

    cv::Mat_<float> m(3, 3);
    cv::Rodrigues(r, m);
    return m;
}

class LidarCameraNode
{

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_subImage;
    ros::Subscriber m_subPointCloud;
    //ros::Publisher m_pub;

    std::unique_ptr<pcl::PointCloud<pcl::PointXYZI> > m_lastPointCloud;
    std::unique_ptr<sensor_msgs::Image> m_lastImage;
    std::mutex m_mutex;

    cv::Mat_<float> mA; //camera intrinsic
    cv::Mat_<float> mDist; //camera distort
    cv::Mat_<float> mRt;

public:
    bool m_stop = false;

public:
    LidarCameraNode()
    {
        m_subImage = m_nh.subscribe("/image", 1000, &LidarCameraNode::imageCallback, this);
        m_subPointCloud = m_nh.subscribe("/velodyne_points", 100, &LidarCameraNode::pointCloudCallback, this);
    
    
        // the rotation from lidar coord (X front Y left) to camera coord (X right Y down)
        cv::Mat_<float> axis1(1, 3);
        axis1(0, 0) = 0;
        axis1(0, 1) = 0;
        axis1(0, 2) = 1;
        cv::Mat_<float> R1 = rotationByAxis(axis1, -CV_PI/2);
        cv::Mat_<float> axis2(1, 3);
        axis2(0, 0) = 0;
        axis2(0, 1) = 1;
        axis2(0, 2) = 0;
        cv::Mat_<float> R2 = rotationByAxis(axis2, CV_PI/2);
        cv::Mat_<float> Rlidar = R2*R1;
        Rlidar = Rlidar.inv(); // matrix of coord is inv of matrix of xy

        // final Rt
        cv::Mat_<float> R = cv::Mat_<float>::zeros(3, 3);
        R(0, 0) = 1;
        R(1, 1) = 1;
        R(2, 2) = 1;

        R = R*Rlidar;

        cv::Mat_<float> Rt = cv::Mat_<float>::zeros(3, 4);
        for(int i = 0; i < 3; ++i) 
            for(int j = 0; j < 3; ++j) {
                Rt(i, j) = R(i, j);
            }

        // translation
        Rt(0, 3) = 0;
        Rt(1, 3) = 0;
        Rt(2, 3) = 0;

        /*

        Rt: [0.010087344, -0.99994814, -0.0014967881, -0.0099999998;
 -0.050143667, 0.00098789344, -0.99874109, -0.38000003;
 0.99869001, 0.010174613, -0.050131064, -0.87999976]

    Auto calib:
///////////
    Average translation is: 0.00866682 -0.535491 -0.446261
    Final rotation is:
    0.0176194  -0.997378 -0.0701919
    0.0231083  0.0705903  -0.997238
    0.999578  0.0159487  0.0242914

//////////
    0.0101941   -0.996833  -0.0788668
-0.00831791   0.0787836   -0.996857
   0.999913    0.010818 -0.00748845


         0.0304534  -0.996871 -0.0729403
0.00204286  0.0730361  -0.997327
  0.999534   0.030223 0.00426066
translation is:^M
0.0629707^M
-0.360966^M
-0.382847


0.0288035  -0.992645  -0.117583
-0.0412694    0.11635   -0.99235
  0.998733  0.0334358 -0.0376146
Average translation is:
-0.0570178
-0.350666
-0.660128

        */
       /*
        Rt(0, 0) = 0.010087344;
        Rt(0, 1) = -0.99994814;
        Rt(0, 2) = -0.0014967881;
        Rt(0, 3) = 0.0099999998;
        Rt(1, 0) = -0.050143667;
        Rt(1, 1) = 0.00098789344;
        Rt(1, 2) = -0.99874109;
        Rt(1, 3) = -0.38000003;
        Rt(2, 0) = 0.99869001;
        Rt(2, 1) = 0.010174613;
        Rt(2, 2) = -0.050131064;
        Rt(2, 3) = -0.87999976;
*/
       
        Rt(0, 0) = 0.0288035;
        Rt(0, 1) = -0.992645;
        Rt(0, 2) = -0.117583;
        Rt(0, 3) = -0.0570178;
        Rt(1, 0) = -0.0412694;
        Rt(1, 1) = 0.11635;
        Rt(1, 2) = -0.99235;
        Rt(1, 3) = -0.350666;
        Rt(2, 0) = 0.998733;
        Rt(2, 1) = 0.0334358;
        Rt(2, 2) = -0.0376146;
        Rt(2, 3) = -0.660128;
        

        mRt = Rt;

        /*
        Front camear: ========================
        Camera instrinsic matrix:
[992.1620754487082, 0, 969.5583299058292;
 0, 1000.674008070566, 654.071285477028;
 0, 0, 1]

DistCoeffs:k1,k2,p1,p2,k3
Distort:
[-0.3916873617463607, 0.2142708213044636, 0.00296983348161513, 0.0004183505466134317, -0.05536865707371406]

        Side camera: ==============
[970.9985230135304, 0, 954.3524844256706;
 0, 971.1977084259171, 646.8882402685825;
 0, 0, 1]

DistCoeffs:k1,k2,p1,p2,k3
Distort:
[-0.3561908021676249, 0.1571885803354126, -0.0007415593773920534, -2.88538532614705e-05, -0.0364625720320078]


        */
        
        cv::Mat_<float> A = cv::Mat_<float>::zeros(3, 3);
        A(0, 0) = 992.1620754487082; //fx
        A(1, 1) = 1000.674008070566; //fy
        A(0, 2) = 969.5583299058292; //cx
        A(1, 2) = 654.071285477028; //cy
        A(2, 2) = 1;

        mA = A;

        cv::Mat_<float> D = cv::Mat_<float>::zeros(1, 5);
        D(0, 0) = -0.3916873617463607;
        D(0, 1) = 0.2142708213044636;
        D(0, 2) = 0.00296983348161513;
        D(0, 3) = 0.0004183505466134317;
        D(0, 4) = -0.05536865707371406;
        mDist = D;
    }

    void rotateXYZ(float x, float y, float z)
    {
        cv::Mat_<float> R = cv::Mat_<float>::zeros(3, 3);
        for(int i = 0; i < 3; ++i) 
            for(int j = 0; j < 3; ++j) {
                R(i, j) = mRt(i, j);
            }

        cv::Mat_<float> axis1(1, 3);
        axis1(0, 0) = 1;
        axis1(0, 1) = 0;
        axis1(0, 2) = 0;
        cv::Mat_<float> R1 = rotationByAxis(axis1, x);

        cv::Mat_<float> axis2(1, 3);
        axis2(0, 0) = 0;
        axis2(0, 1) = 1;
        axis2(0, 2) = 0;
        cv::Mat_<float> R2 = rotationByAxis(axis2, y);

        cv::Mat_<float> axis3(1, 3);
        axis3(0, 0) = 0;
        axis3(0, 1) = 0;
        axis3(0, 2) = 1;
        cv::Mat_<float> R3 = rotationByAxis(axis3, z);

        R = R*R3*R2*R1;

        for(int i = 0; i < 3; ++i) 
            for(int j = 0; j < 3; ++j) {
                mRt(i, j) = R(i, j);
            }
    }

    void translateXYZ(float x, float y, float z)
    {
        mRt(0, 3) += x;
        mRt(1, 3) += y;
        mRt(2, 3) += z;
    }

    void handleKey(int key)
    {
        float dr = 0.005;
        float dt = 0.01;
        switch (key) {
            case 'w':
            case 'W':
                rotateXYZ(dr, 0, 0);
                break;
            case 's':
            case 'S':
                rotateXYZ(-dr, 0, 0);
                break;
            case 'e':
            case 'E':
                rotateXYZ(0, dr, 0);
                break;
            case 'd':
            case 'D':
                rotateXYZ(0, -dr, 0);
                break;
            case 'r':
            case 'R':
                rotateXYZ(0, 0, dr);
                break;
            case 'f':
            case 'F':
                rotateXYZ(0, 0, -dr);
                break;

            case 'u':
            case 'U':
                translateXYZ(dt, 0, 0);
                break;
            case 'j':
            case 'J':
                translateXYZ(-dt, 0, 0);
                break;
            case 'i':
            case 'I':
                translateXYZ(0, dt, 0);
                break;
            case 'k':
            case 'K':
                translateXYZ(0, -dt, 0);
                break;
            case 'o':
            case 'O':
                translateXYZ(0, 0, dt);
                break;
            case 'l':
            case 'L':
                translateXYZ(0, 0, -dt);
                break;

            case 13: //CR
            case 10: //LF
                std::cout << "Rt: " << mRt << std::endl;
                break;
        }
    }

    void doCalib()
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        if(!m_lastPointCloud || !m_lastImage) {
            return;
        }

        // the default intrinsic is based on 1920x1208 resolution
        float imageScaleX = m_lastImage->width / 1920.f; 
        float imageScaleY = m_lastImage->height / 1208.f; 
        cv::Mat_<float> cA = mA.clone();
        cA(0, 0) *= imageScaleX;
        cA(1, 1) *= imageScaleY;
        cA(0, 2) *= imageScaleX;
        cA(1, 2) *= imageScaleY;

        cv::Mat matOrigin(m_lastImage->height,m_lastImage->width,CV_8UC3, (unsigned char*)m_lastImage->data.data());
        
        cv::Mat mat;
        cv::undistort(matOrigin, mat, cA, mDist);

        cv::cvtColor(mat, mat, cv::COLOR_RGB2BGR);

        //std::cout << "lidar size " << m_lastPointCloud->points.size() << std::endl;
        auto t1 = std::chrono::system_clock::now();
        const int PT_SIZE = 3;
        int rc_shift = (PT_SIZE - 1)/2;
            

        // https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html

        const int pointCount = m_lastPointCloud->points.size();
        cv::Mat_<float> M(4, pointCount);
        cv::Mat_<unsigned char> grayVec(1, pointCount);
        
        for (int i = 0; i < pointCount; ++i) 
		{
            const auto& pt = m_lastPointCloud->points[i];
            M(0, i) = pt.x;
            M(1, i) = pt.y;
            M(2, i) = pt.z;
            M(3, i) = 1;

            float dist = sqrtf(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
            grayVec.at<unsigned char>(0, i) = std::min(1.0f, dist / 5) * 255;
            //grayVec(0, i) = pt.intensity;
        }
        
        auto t2 = std::chrono::system_clock::now();
        cv::Mat_<float> Mc = mRt * M; // convert to camera world coordinate

        // filter point with z <= 0 in camera world cooridnate, 
        // avoid lidar point overlap after project to camera plane
        int valid_cols = 0;
        cv::Mat_<float> Mcc(Mc.rows, Mc.cols);
        cv::Mat_<unsigned char> grayVecTmp(grayVec.rows, grayVec.cols);
        for (int i = 0; i < Mc.cols; ++i)
        {
            if (Mc(2, i) > 0) {
                Mcc(0, valid_cols) = Mc(0, i);
                Mcc(1, valid_cols) = Mc(1, i);
                Mcc(2, valid_cols) = Mc(2, i);
                grayVecTmp(0, valid_cols) = grayVec(0, i);
                ++valid_cols; 
            }
        }
        //std::cout << "filtered z<0 points " << Mc.cols - valid_cols << std::endl;
        Mc.create(3, valid_cols);
        grayVec.create(1, valid_cols);
        for (int i = 0; i < valid_cols; ++i)
        {
            Mc(0, i) = Mcc(0, i);
            Mc(1, i) = Mcc(1, i);
            Mc(2, i) = Mcc(2, i);
            grayVec(0, i) = grayVecTmp(0, i);
        }

        cv::Mat uv = cA * Mc; // convert to camera plane coordinate
        cv::Mat colorVec;
        cv::applyColorMap(grayVec, colorVec, cv::COLORMAP_JET);

        auto t3 = std::chrono::system_clock::now();
        for (int i = 0; i < uv.cols; ++i)
        {
            float w = uv.at<float>(2, i);
            float x = uv.at<float>(0, i) / w;
            float y = uv.at<float>(1, i) / w;

            //cv::rectangle(mat, cv::Point2d(x - rc_shift,y - rc_shift), cv::Point2d(x + rc_shift,y + rc_shift), colorVec[i], -1, cv::LINE_8, 0);

            int col = x;
            int row = y;
            if(col >= 0 && col < mat.cols && row >= 0 && row < mat.rows) {
                //std::cout << "col row" << col << ", " << row << " - " << mat.cols << ", " << mat.rows << std::endl;
                auto color = colorVec.at<cv::Vec3b>(0, i);
                mat.at<cv::Vec3b>(row, col) = color;

                //enlarge point area
                if(col+1 < mat.cols && row+1 < mat.rows) {
                    mat.at<cv::Vec3b>(row, col+1) = color;
                    mat.at<cv::Vec3b>(row+1, col+1) = color;
                    mat.at<cv::Vec3b>(row+1, col) = color;
                }
            }

        }

        
		auto t2_t1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1);
        auto t3_t2 = std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2);
        //std::cout << " timecost " << t2_t1.count() << "   " << t3_t2.count() << std::endl;

        cv::namedWindow("Image", cv::WINDOW_GUI_EXPANDED); // WINDOW_GUI_EXPANDED needs -DWITH_QT when build opencv
        cv::imshow("Image",mat);
        int key = cv::waitKey(30);
        if(27 == key) {
            exit(1); //exit when ESC pressed
        } else {
            handleKey(key);
        }
    }

    void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
    {   
        //std::cout << "receive image " << msg->width << "," << msg->height << std::endl;
        std::unique_ptr<sensor_msgs::Image> inputImage(new sensor_msgs::Image());
        *inputImage = *msg;
        std::lock_guard<std::mutex> lk(m_mutex);
        m_lastImage = std::move(inputImage);
    } 

    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //	for(int i = 0; i < msg->fields.size();i++){
    //		std::cout << "field ," << msg->fields[i].name << "," << msg->fields[i].datatype << std::endl;
    //	}
    //	if(lidarTime.sec != 0){
    //		std::cout << "lidar time diff " << ros::Time().now() - lidarTime << std::endl;
    //	}
    //	lidarTime = ros::Time().now();
        pcl::PCLPointCloud2 pclPointCloud;
        pcl_conversions::toPCL(*msg, pclPointCloud);

        std::unique_ptr<pcl::PointCloud<pcl::PointXYZI>> pointCloud(new pcl::PointCloud<pcl::PointXYZI>());

        //pcl::console::setVerbosityLevel(pcl::console::L_DEBUG);
        pcl::fromPCLPointCloud2(pclPointCloud, *pointCloud);

        //std::cout << "lidar received timestamp " << msg->header.stamp << " size " << pointCloud->points.size() << std::endl;
        // for (auto& pt : pointCloud->points) 
		// {
        //     std::cout << pt.x << ", " << pt.y << "," << pt.z << "," << pt.intensity << std::endl;
        // }
        std::lock_guard<std::mutex> lk(m_mutex);
        m_lastPointCloud = std::move(pointCloud);
    } 

    void run_calib()
    {
        while (!m_stop)
        {
            doCalib();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    }
};

LidarCameraNode *myNode = nullptr;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "lidar_camera_proj");

    myNode = new LidarCameraNode();

    // std::string config_path = ros::package::getPath("lidar_camera_proj");
    // std::string rig_path = config_path + "/build_external/rig.xml";

    std::thread calib_thread(&LidarCameraNode::run_calib, myNode);

    int ret = 0;
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    myNode->m_stop = true;
    calib_thread.join();

    delete myNode;
    return ret;
}
