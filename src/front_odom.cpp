#include<ros/ros.h>
#include<sensor_msgs/LaserScan.h>
#include<opencv2/opencv.hpp>
#include<ceres/ceres.h>
#include<sophus/se3.hpp>
using namespace std;
using namespace cv;
using namespace Sophus;
typedef vector<Vector3d, Eigen::aligned_allocator<Vector3d>> VecVector3d;
typedef vector<Vector2d, Eigen::aligned_allocator<Vector3d>> VecVector2d;
typedef Matrix<double, 6, 1> Vector6d;

class LaserOdom{
    public:
        LaserOdom(ros::NodeHandle& n);
        void laserodomcallback(const sensor_msgs::LaserScanConstPtr& laser_msg);
        void line(cv::Point2i pix1,cv::Point2i pix2);
    private:
        ros::Subscriber sub_;
        vector<float> a_cos_;
        vector<float> a_sin_;
        const int SCALE = 30;//激光比例尺
        const float p_hit = 0.9;//击中概率
        const float p_miss = 0.1;//未击中概率
        const float p_init = 0.5;//概率初始化
        cv::Mat *framemap;
        cv::Mat last_framemap;
        bool init = false;
};

LaserOdom::LaserOdom(ros::NodeHandle& n){
    sub_ = n.subscribe<sensor_msgs::LaserScan>
    ("/scan",1000,&LaserOdom::laserodomcallback,this);
}
void LaserOdom::laserodomcallback(const sensor_msgs::LaserScanConstPtr& laser_msg){
    
    vector<float> laser_range(laser_msg->ranges);
    vector<float> laser_angle;
    const int POINTSIZE = laser_msg->ranges.size();
    const int MAPSIZE = laser_msg->range_max * 80;
    a_cos_.clear();
    a_sin_.clear();
    for(int i = 0;i < POINTSIZE;i++){
        float langle = laser_msg->angle_min + i * laser_msg->angle_increment;
        laser_angle.push_back(langle);
        a_sin_.push_back(sinf(langle));
        a_cos_.push_back(cosf(langle));
    }
    /*把range与angle转化为坐标*/
    vector<cv::Point2i> pix_vec;
    framemap = new Mat(Size(MAPSIZE,MAPSIZE),CV_8UC1,Scalar(127));
    if(!init){
        last_framemap = cv::Mat(Size(MAPSIZE,MAPSIZE),CV_8UC1,Scalar(127));
        init = true;    
    }
    ceres::Problem problem;
    /*概率栅格化*/
    int iterations = 100;//迭代次数
    double cost = 0, lastCost = 0;
    Sophus::SE3d T_esti;
    for(int iter = 0;iter < iterations;iter++){
        Sophus::Matrix<double, 6, 6> H = Sophus::Matrix<double, 6, 6>::Zero();
        Vector6d b = Vector6d::Zero();//b=J转置×f(x)，为６×2×2×１＝６＊１
        cost = 0;
        for(int i = 0;i < POINTSIZE;i++){
            if(laser_range[i] != INFINITY){
                cv::Point2f pix_laser(laser_range[i]*a_cos_[i]*SCALE,laser_range[i]*a_sin_[i]*SCALE);//激光坐标系下
                cv::Point2i pix_win;//窗口坐标系下
                cv::Point2i pix_origin;
                Vector3d pix_lastlaser=T_esti.rotationMatrix() *Vector3d(pix_laser.x,pix_laser.y,1)+T_esti.translation();
            
                pix_origin.x = MAPSIZE >> 1;
                pix_origin.y = MAPSIZE >> 1;
                pix_win.x = (MAPSIZE>>1) + pix_laser.x;
                pix_win.y = (MAPSIZE>>1) + pix_laser.y;
                pix_vec.push_back(pix_win);
                float hit_odds = log(p_hit/(1-p_hit)) + (127 - last_framemap.at<uchar>(pix_win.y,pix_win.x));//赔率
                hit_odds = hit_odds > 127 ? 127:hit_odds;
                framemap->at<uchar>(pix_win.y,pix_win.x) = 127 - hit_odds;
                line(pix_origin,pix_win);

            }
        }   
    }
    last_framemap = framemap->clone();

    /*显示地图*/
    imshow("framemap",*framemap);
    cv::waitKey();
    delete(framemap);
    
    
}
void LaserOdom::line(cv::Point2i pix1,cv::Point2i pix2) {
  int dx = abs(pix2.x-pix1.x), sx = pix1.x<pix2.x ? 1 : -1;
  int dy = abs(pix2.y-pix1.y), sy = pix1.y<pix2.y ? 1 : -1; 
  int err = (dx>dy ? dx : -dy)>>1, e2;
  while(!(pix1.x==pix2.x && pix1.y==pix2.y)){
    float miss_odds = log(p_miss/(1-p_miss)) + (127 - last_framemap.at<uchar>(pix1.y,pix1.x));//赔率
    miss_odds = miss_odds < -128 ? -128:miss_odds;
    framemap->at<uchar>(pix1.y,pix1.x) = 127 - miss_odds;
    e2 = err;
    if (e2 >-dx) { err -= dy; pix1.x += sx; }
    if (e2 < dy) { err += dx; pix1.y += sy; }
  }
}

int main(int argc, char *argv[])
{
    ros::init(argc,argv,"laser_odom");
    ros::NodeHandle nh;
    LaserOdom m(nh);
    ros::spin();
    return 0;
}
