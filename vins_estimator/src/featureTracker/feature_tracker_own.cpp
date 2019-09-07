#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <queue>
#include <vector>
#include <thread>
#include <mutex>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <math.h>
using namespace std;
using namespace cv;


queue<sensor_msgs::ImageConstPtr> img0_buf;
queue<sensor_msgs::ImageConstPtr> img1_buf;
std::mutex m_buf;

void img0_cb(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img0_buf.push(img_msg);
    m_buf.unlock();
}

void img1_cb(const sensor_msgs::ImageConstPtr &img_msg){
    m_buf.lock();
    img1_buf.push(img_msg);
    m_buf.unlock();
}

cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg){
    cv_bridge::CvImageConstPtr ptr;
    ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    cv::Mat img = ptr->image.clone();
    return img;
}

// void setMask(const std::vector<cv::Point2f> &cur_pts){
//     vector<pair<int, pair<cv::Point2f, int>>> cnt_pts_id;
//     for (unsigned int i = 0; i < cur_pts.size(); i++)
//         cnt_pts_id.push_back(make_pair(track_cnt[i], make_pair(cur_pts[i], ids[i])));
// }
void tracking_img(const cv::Mat &img1,const cv::Mat &img2){
    // cv::Mat img1 = cv::imread("1.png");
    // cv::Mat img2 = cv::imread("2.png");
    std::vector<cv::KeyPoint> keypoints_1;
    std::vector<cv::KeyPoint> keypoints_2;
    cv::Mat descriptors_1;
    cv::Mat descriptors_2;
    
    Ptr<ORB> orb_detector_1 = ORB::create(150);
    Ptr<ORB> orb_detector_2 = ORB::create(150);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    orb_detector_1->detect(img1, keypoints_1);
    orb_detector_2->detect(img2, keypoints_2);
    orb_detector_1->compute(img1, keypoints_1, descriptors_1);
    orb_detector_2->compute(img2, keypoints_2, descriptors_2);

    // grid(keypoints_1);
    // vector<cv::Point2f> cur_pts(keypoints_1.size());
    // for(int i=0; i<keypoints_1.size(); ++i)
    //     cur_pts[i] = keypoints_1[i].pt;
    // vector<cv::Point2f> cur_pts(keypoints_2.size());
    // for(int i=0; i<keypoints_2.size(); ++i)
    //     cur_pts[i] = keypoints_2[i].pt;

    // setMask(cur_pts);
    cv::Mat img_keypoints1;
    cv::drawKeypoints(img1, keypoints_1, img_keypoints1, cv::Scalar(0,0,255), 0);

    cv::Mat img_keypoints2;
    cv::drawKeypoints(img2, keypoints_2, img_keypoints2, cv::Scalar(0,0,255), 0);
    // cv::imshow("describtor1",img_keypoints1);
    // cv::imshow("describtor2",img_keypoints2);
    //////////////////////////
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);
    ////////////////////////////
    // auto min_dist,max_dist;
    ////////////////////////////matching optimization
    /*
    auto _min_dist = *min_element(matches.begin(),matches.end());
    auto _max_dist = *max_element(matches.begin(),matches.end());

    double min_dist = (_min_dist.distance);
    double max_dist = (_max_dist.distance);
    // printf ( "-- Max dist : %f \n", max_dist );
    // printf ( "-- Min dist : %f \n", min_dist );
    */
   float min_dist = std::min_element(
       matches.begin(), matches.end(),
       [] (const cv::DMatch& m1, const cv::DMatch& m2){
           return m1.distance < m2.distance;
       }
   )->distance;

    std::vector< DMatch > good_matches;
    for ( int i = 0; i < descriptors_1.rows; i++ )
    {
        if ( matches[i].distance <= max<float> ( 2*min_dist, 40.0 ) )
        {
            good_matches.push_back ( matches[i] );
        }
    }
    std::cout << keypoints_1.size() <<std::endl;
    std::cout << keypoints_2.size() <<std::endl;
    Mat img_match;
    Mat img_goodmatch;
    cv::drawMatches ( img1, keypoints_1, img2, keypoints_2, good_matches, img_match );
    cv::imshow ( "match", img_match );
    cv::waitKey(2);
}

void sync_process(){
    while(1){
        if(img0_buf.empty()||img1_buf.empty()){
            std::chrono::milliseconds dura(2);
            std::this_thread::sleep_for(dura);
            continue;
        }
        cv::Mat img0;
        cv::Mat img1;
        double time0;
        double time1;
        double time;
        m_buf.lock();

        time0 = img0_buf.front()->header.stamp.toSec();
        time1 = img1_buf.front()->header.stamp.toSec();
        if(time0 > time1)   {img1_buf.pop();}
        else if(time0 < time1)   {img0_buf.pop();}
        else if(time0 == time1){
            time = img0_buf.front()->header.stamp.toSec();
            img0 = getImageFromMsg(img0_buf.front());
            img1 = getImageFromMsg(img1_buf.front());
            img0_buf.pop();
            img1_buf.pop();
        }
        m_buf.unlock();
        tracking_img(img0, img1);
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc,char** argv){
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle nh("~");
    
    ros::Subscriber sub_img0 = nh.subscribe("/camera_ns/stereo/left/image_raw", 2, img0_cb);
    ros::Subscriber sub_img1 = nh.subscribe("/camera_ns/stereo/right/image_raw", 2, img1_cb);
    // cv::Mat img0 = cv::imread("1.png");
    // cv::Mat img1 = cv::imread("2.png");
    // tracking_img(img0, img1);
    std::thread sync_thread{sync_process};
    ros::spin();

    return 0;
}

// void grid(const std::vector<cv::KeyPoint> &features){
//     // static int grid_height = img1.rows / grid_row;
//     // static int grid_ = img1.rows / grid_row;
//     // vector<KeyPoint> features(0);
//     vector<cv::Point2f> cam0_points(features.size());
//     for(int i=0; i<features.size(); ++i)
//         cam0_points[i] = features[i].pt;

//     vector<cv::Point2f> cam1_points(0);
//     vector<unsigned char> inlier_markers(0);
//     stereoMatch(cam0_points, cam1_points, inlier_markers);

//     GridFeatures grid_new_features;
//     for (int code = 0; code <
//         grid_row * grid_col; ++code)
//         grid_new_features[code] = vector<FeatureMetaData>(0);

//     for (int i = 0; i < cam0_inliers.size(); ++i) {
//         const cv::Point2f& cam0_point = cam0_inliers[i];
//         const cv::Point2f& cam1_point = cam1_inliers[i];
//         const float& response = response_inliers[i];

//         int row = static_cast<int>(cam0_point.y / grid_height);
//         int col = static_cast<int>(cam0_point.x / grid_width);
//         int code = row*processor_config.grid_col + col;

//         FeatureMetaData new_feature;
//         new_feature.response = response;
//         new_feature.cam0_point = cam0_point;
//         new_feature.cam1_point = cam1_point;
//         grid_new_features[code].push_back(new_feature);
//     }
// }

/*
    //1.使用FAST函数进行特征检测
int main(int argc, char** argv){
    cv::Mat img = cv::imread("1.png");
    std::vector<cv::KeyPoint> keypoints;

    // cv::FastFeatureDetector fast;
    // fast.create(10);
    // fast.detect(img, keypoints);
    cv::FAST(img, keypoints, 50);
    //1.输入图像 2.输出检测到的特征点 3.中心像素值与周围像素之间的阈值(越小，检测出来的特征点越多) 
    //4.是否采用极大值抑制(默认为1) 5.type像素邻域圆的三种种类

    cv::Mat img_keypoints;
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), 0);

    cv::imshow("test_1",img_keypoints);
    cv::waitKey(0);
    return 0;
}

*/

/*
    //2.使用FastFeatureDetector函数进行特征检测
int main(int argc, char** argv){
    cv::Mat img = cv::imread("1.png");
    std::vector<cv::KeyPoint> keypoints;

    Ptr<FastFeatureDetector> fast = FastFeatureDetector::create(20);
    fast->detect(img, keypoints)

    cv::Mat img_keypoints;
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), 0);

    cv::imshow("test_1",img_keypoints);
    cv::waitKey(0);
    return 0;
}

*/
/*
    //3.orb特征检测(升级版的fast--有了描述子)
int main(int argc, char** argv){
    cv::Mat img = cv::imread("1.png");
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;

    Ptr<ORB> orb_detector = ORB::create();
    // Ptr<ORB> orb_descriptor = ORB::create();
    orb_detector->detect(img, keypoints);
    // orb_descriptor->compute(img, keypoints, descriptors);
    orb_detector->compute(img, keypoints, descriptors);

    cv::Mat img_keypoints;
    cv::drawKeypoints(img, keypoints, img_keypoints, cv::Scalar::all(-1), 0);

    cv::imshow("describtor",img_keypoints);
    cv::waitKey(0);
    return 0;
}
*/
/*
    //4.orb特征检测+描述子匹配(升级版的fast--有了描述子)
int main(int argc, char** argv){
    cv::Mat img1 = cv::imread("1.png");
    cv::Mat img2 = cv::imread("2.png");
    std::vector<cv::KeyPoint> keypoints_1;
    std::vector<cv::KeyPoint> keypoints_2;
    cv::Mat descriptors_1;
    cv::Mat descriptors_2;

    Ptr<ORB> orb_detector_1 = ORB::create(30);
    Ptr<ORB> orb_detector_2 = ORB::create(30);
    Ptr<DescriptorMatcher> matcher  = DescriptorMatcher::create ( "BruteForce-Hamming" );
    orb_detector_1->detect(img1, keypoints_1);
    orb_detector_2->detect(img2, keypoints_2);
    orb_detector_1->compute(img1, keypoints_1, descriptors_1);
    orb_detector_2->compute(img2, keypoints_2, descriptors_2);

    cv::Mat img_keypoints1;
    cv::drawKeypoints(img1, keypoints_1, img_keypoints1, cv::Scalar::all(-1), 0);

    cv::Mat img_keypoints2;
    cv::drawKeypoints(img2, keypoints_2, img_keypoints2, cv::Scalar::all(-1), 0);
    cv::imshow("describtor1",img_keypoints1);
    cv::imshow("describtor2",img_keypoints2);
    //////////////////////////
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors_1, descriptors_2, matches);




    Mat img_match;
    Mat img_goodmatch;
    drawMatches ( img1, keypoints_1, img2, keypoints_2, matches, img_match );
    // drawMatches ( img_1, keypoints_1, img_2, keypoints_2, good_matches, img_goodmatch );
    imshow ( "所有匹配点对", img_match );
    // imshow ( "优化后匹配点对", img_goodmatch );

    cv::waitKey(0);
    return 0;
}
*/

/*
void get_q_from_dcm(Eigen::Quaterniond &q, const Eigen::Matrix3d &dcm) {
    // Eigen::Matrix3d mat = Eigen::Matrix3d::Zero() ;
    // mat << 0,0,1,1,0,0,0,1,0;
    // Eigen::Quaterniond q;
    // get_q_from_dcm(q,mat);
    // cout << q.w() << endl;
    // cout << q.x() << endl;
    // cout << q.y() << endl;
    // cout << q.z() << endl;
    float t = dcm.trace();
    if ( t > 0.0f ) {
        t = sqrt(1.0f + t);
        q.w() = 0.5f * t;
        t = 0.5f / t;
        q.x() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(0,2) - dcm(2,0)) * t;
        q.z() = (dcm(1,0) - dcm(0,1)) * t;
    } else if (dcm(0,0) > dcm(1,1) && dcm(0,0) > dcm(2,2)) {
        t = sqrt(1.0f + dcm(0,0) - dcm(1,1) - dcm(2,2));
        q.x() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(2,1) - dcm(1,2)) * t;
        q.y() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(0,2) + dcm(2,0)) * t;
    } else if (dcm(1,1) > dcm(2,2)) {
        t = sqrt(1.0f - dcm(0,0) + dcm(1,1) - dcm(2,2));
        q.y() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(0,2) - dcm(2,0)) * t;
        q.x() = (dcm(1,0) + dcm(0,1)) * t;
        q.z() = (dcm(2,1) + dcm(1,2)) * t;
    } else {
        t = sqrt(1.0f - dcm(0,0) - dcm(1,1) + dcm(2,2));
        q.z() = 0.5f * t;
        t = 0.5f / t;
        q.w() = (dcm(1,0) - dcm(0,1)) * t;
        q.x() = (dcm(0,2) + dcm(2,0)) * t;
        q.y() = (dcm(2,1) + dcm(1,2)) * t;
    }
}
*/