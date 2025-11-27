#include "occupany_mapping.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point32.h"
#include "nav_msgs/Path.h"
#include <nav_msgs/MapMetaData.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

//Eigen
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>

#define disTH 0.01

/**
 * Increments all the grid cells from (x0, y0) to (x1, y1);
 * //不包含(x1,y1)
 * 2D画线算法　来进行计算两个点之间的grid cell
 * @param x0
 * @param y0
 * @param x1
 * @param y1
 */
std::vector<GridIndex> TraceLine(int x0, int y0, int x1, int y1)
{
  GridIndex tmpIndex;
  std::vector<GridIndex> gridIndexVector;

  bool steep = fabs(y1 - y0) > fabs(x1 - x0);
  if (steep)
  {
    std::swap(x0, y0);
    std::swap(x1, y1);
  }
  if (x0 > x1)
  {
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int deltaX = x1 - x0;
  int deltaY = fabs(y1 - y0);
  int error = 0;
  int ystep;
  int y = y0;

  if (y0 < y1)
  {
    ystep = 1;
  }
  else
  {
    ystep = -1;
  }

  int pointX;
  int pointY;
  for (int x = x0; x <= x1; x++)
  {
    if (steep)
    {
      pointX = y;
      pointY = x;
    }
    else
    {
      pointX = x;
      pointY = y;
    }

    error += deltaY;

    if (2 * error >= deltaX)
    {
      y += ystep;
      error -= deltaX;
    }

    //不包含最后一个点．
    if(pointX == x1 && pointY == y1) continue;

    //保存所有的点
    tmpIndex.SetIndex(pointX,pointY);

    gridIndexVector.push_back(tmpIndex);
  }

  return gridIndexVector;
}


//判断index是否有效
bool isValidGridIndex(GridIndex index, int width, int height)
{
    if(index.x >= 0 && index.x < width && index.y >= 0 && index.y < height)
        return true;

    return false;
}



class ScanProcess{
public:
    ScanProcess();

    sensor_msgs::LaserScan pre_scan;

    Eigen::Vector3d scan_pos_cal; //使用激光匹配算出来的新位姿
    std::string odom_frame;
    std::string base_frame;
    std::string map_frame;
    std::string laser_frame;

    ros::NodeHandle nodeHandler;
    tf::TransformListener tf_;
    //进行时间同步
    message_filters::Subscriber<sensor_msgs::LaserScan>* scan_filter_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan>* scan_filter_;

    ros::Publisher scanodomPub;
    ros::Publisher mapPub;

    nav_msgs::Path path_scan;
    nav_msgs::OccupancyGrid laser_map;
    Eigen::Vector3d now_pos, last_pos; //储存临时pos

    ros::Time current_time;


    void LaserCallback(const sensor_msgs::LaserScan::ConstPtr& _laserScanMsg);
    void pub_path( Eigen::Vector3d& pose,nav_msgs::Path &path, ros::Publisher &pub);
    void pub_Map();

    //进行激光匹配
    void GetRelativePose(sensor_msgs::LaserScan scan, Eigen::Vector3d delta_pose, Eigen::Vector3d &pose);


    //tf树查询里程计位姿
    bool getOdomPose(Eigen::Vector3d& pose, const ros::Time& t);
    Eigen::Vector3d cal_delta_distence();

    //地图相关
    map_params mapParams;
    unsigned char* pMap;
    nav_msgs::OccupancyGrid rosMap;
    void SetMapParams();
    GridIndex ConvertWorld2GridIndex(double x,double y);
    bool firstregister;
    int GridIndexToLinearIndex(GridIndex index);
    tf::Transform map_to_odom_;
    boost::mutex map_to_odom_mutex_;
    boost::thread* transform_thread_;
    void publishLoop(double transform_publish_period);
    tf::TransformBroadcaster* tfB_;

};


ScanProcess::ScanProcess(){
    firstregister = true;

//    laserSub = nodeHandler.subscribe("/scan",1000, &ScanProcess::LaserCallback, this); //订阅激光
    scanodomPub = nodeHandler.advertise<nav_msgs::Path>("laser_path",1,true); //发布估计的位姿
    mapPub = nodeHandler.advertise<nav_msgs::OccupancyGrid>("map",1,true); //发布地图

    //坐标系设定
    odom_frame = "odom";
    base_frame = "base_link";
    map_frame = "map";
    laser_frame = "laser_link";

    current_time = ros::Time::now();
    path_scan.header.stamp=current_time;
    path_scan.header.frame_id=map_frame;

    laser_map.header.stamp=current_time;
    laser_map.header.frame_id=map_frame;

    last_pos << 0., 0., 0.;
    now_pos << 0., 0., 0.;
    scan_pos_cal.setZero();

    // 初始化地图
    SetMapParams();
    rosMap.info.resolution = mapParams.resolution;
    rosMap.info.origin.position.x = 0.0;
    rosMap.info.origin.position.y = 0.0;
    rosMap.info.origin.position.z = 0.0;
    rosMap.info.origin.orientation.x = 0.0;
    rosMap.info.origin.orientation.y = 0.0;
    rosMap.info.origin.orientation.z = 0.0;
    rosMap.info.origin.orientation.w = 1.0;

    
    rosMap.info.origin.position.x = mapParams.origin_x;
    rosMap.info.origin.position.y = mapParams.origin_y;
    rosMap.info.width = mapParams.width;
    rosMap.info.height = mapParams.height;
    rosMap.data.resize(rosMap.info.width * rosMap.info.height);
    rosMap.header.frame_id = map_frame;

    //进行里程计和激光雷达数据的同步
    scan_filter_sub_ = new message_filters::Subscriber<sensor_msgs::LaserScan>(nodeHandler, "/scan", 10);
    scan_filter_ = new tf::MessageFilter<sensor_msgs::LaserScan>(*scan_filter_sub_, tf_, odom_frame, 10); //由tf树得到
    scan_filter_->registerCallback(boost::bind(&ScanProcess::LaserCallback, this, _1));
    tfB_ = new tf::TransformBroadcaster();
    transform_thread_ = new boost::thread(boost::bind(&ScanProcess::publishLoop, this, 0.05)); // 用于发布map与odom之间的坐标转换
}

void ScanProcess::publishLoop(double transform_publish_period){
    if(transform_publish_period == 0)
        return;

    ros::Rate r(1.0 / transform_publish_period);
    while(ros::ok()){
        map_to_odom_mutex_.lock();
        ros::Time tf_expiration = ros::Time::now() + ros::Duration(transform_publish_period);
        tfB_->sendTransform( tf::StampedTransform (map_to_odom_, tf_expiration, map_frame, odom_frame));
        map_to_odom_mutex_.unlock();
        r.sleep();
    }
}

/*
 * TODO
 * 通过两帧激光数据的匹配求出相对位姿
 * 输入: pre_scan && scan && 由底盘得到的相对运动
 * 输出: pose, 格式为(dx, dy, dtheta)
 */
void ScanProcess::GetRelativePose(sensor_msgs::LaserScan scan, Eigen::Vector3d delta_pose, Eigen::Vector3d &pose){
    // pre_scan is the previous laser scan data
    // need to match scan and pre_scan and use kalman filter? to get pose with delta_pose
    std::cout << pose << std::endl;
    pose = pose + delta_pose;
    // pose << 0., 0., 0.;
}

void ScanProcess::LaserCallback(const sensor_msgs::LaserScan::ConstPtr& _laserScanMsg){
    //读激光数据 匹配 得到相对位姿
    static long int dataCnt = 0;
    Eigen::Vector3d pose; //激光的scanmatch计算的pose: x,y,theta

    // 得到机器人的当前时刻轮式编码器得到的坐标(发布的/odom话题里得到)
    // now_pose: odom-->base_link: base_link在odom坐标系下的坐标
    if(!getOdomPose(now_pos, _laserScanMsg->header.stamp))
        return;

    if(firstregister) {
        pre_scan = *_laserScanMsg;
        last_pos = now_pos;
        firstregister = false;
        pub_Map();
        return;
    }

    //如果运动的距离太短，则不进行处理．
    Eigen::Vector3d delta_pose = cal_delta_distence(); //由底盘里程计得到数据
    if(fabs(delta_pose(0)) < disTH && fabs(delta_pose(1)) < disTH && fabs(delta_pose(2)) < tfRadians(5.0))
    {
        return ;
    }
    last_pos = now_pos;
    sensor_msgs::LaserScan scan = *_laserScanMsg;

    GetRelativePose(scan, delta_pose, pose);

    // 更新位姿, 在上一帧的基础上叠加相对位姿
    double c,s;
    c = cos(scan_pos_cal(2));
    s = sin(scan_pos_cal(2));
    Eigen::Matrix3d transform_matrix;
    transform_matrix<<c,-s,0,
            s, c,0,
            0, 0,1;
    scan_pos_cal+=(transform_matrix*pose);

    //计算机器人现在在map里的位置
    /*
     * now_pos是机器人在里程计坐标系下的位置, 由于轮子可能打滑等原因,其值不一定为真实位置,存在累计误差,看做baselink在odom系下的坐标
     * scan_pos_cal是根据激光数据得到的机器人在地图坐标系下的位置.可看做baselink在map系下的坐标
     * 为了弥补累计误差,即需要对map->odom进行坐标变换补偿,并实时地发布该坐标变换进
     */
    tf::Transform base_to_map = tf::Transform(tf::createQuaternionFromRPY(0, 0, scan_pos_cal(2)), tf::Vector3(scan_pos_cal(0), scan_pos_cal(1), 0.0)).inverse();
    tf::Transform odom_to_base = tf::Transform(tf::createQuaternionFromRPY(0, 0, now_pos(2)), tf::Vector3(now_pos(0), now_pos(1), 0.0));
    map_to_odom_mutex_.lock();
    map_to_odom_ = (odom_to_base * base_to_map).inverse();
    map_to_odom_mutex_.unlock();

    // 位姿发布
    pub_path(scan_pos_cal, path_scan, scanodomPub);

    // 是否更新地图
    if(fabs(delta_pose(0)) > disTH || fabs(delta_pose(1) < disTH) || fabs(delta_pose(2) > tfRadians(5.0))){
        //计算机器人当前位置处于map的什么位置
        GridIndex robotIndex = ConvertWorld2GridIndex(scan_pos_cal(0), scan_pos_cal(1));
        if(isValidGridIndex(robotIndex, mapParams.width, mapParams.height) == false)
        {
            std::cout <<"Error,This should not happen"<<std::endl;
            return; //扩充地图
        }
        for(int id = 0; id < scan.ranges.size();id++) {
            double dist = scan.ranges[id];
            double angle = id * scan.angle_increment;

            if (std::isinf(dist) || std::isnan(dist)) continue;

            double theta = scan_pos_cal(2);
            //雷达坐标系下的坐标
            double laser_x = dist * cos(theta + angle);
            double laser_y = dist * sin(theta + angle);

            //世界坐标系下的坐标--激光机器人进行转换
            double world_x = laser_x + scan_pos_cal(0);
            double world_y = laser_y + scan_pos_cal(1);

            //将激光点位置转换到地图坐标系
            GridIndex mapIndex = ConvertWorld2GridIndex(world_x, world_y);
            if (isValidGridIndex(mapIndex, mapParams.width, mapParams.height) == false) {
                continue;
            }

            //得到所有的被激光通过的index，并且更新栅格
            std::vector<GridIndex> freeIndex = TraceLine(robotIndex.x,robotIndex.y,mapIndex.x,mapIndex.y);

            for(int k = 0; k < freeIndex.size();k++)
            {
                GridIndex tmpIndex = freeIndex[k];
                int linearIndex = GridIndexToLinearIndex(tmpIndex);

                int data = pMap[linearIndex];
                data += mapParams.log_free;
                if(data < 0)
                    data = 0;
                pMap[linearIndex] = data;

            }

            //更新被击中的点
            int tmpIndex = GridIndexToLinearIndex(mapIndex);
            int data = pMap[tmpIndex];
            data += mapParams.log_occ; //计算栅格的概率
            if(data > 100)
                data = 100;
            pMap[tmpIndex] = data;
        }
        pub_Map();
    }
    std::cout <<"Data Cnt:"<<dataCnt++<<std::endl;
}

void ScanProcess::pub_Map() {
    rosMap.header.stamp = ros::Time::now();
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
    {
        rosMap.data[i] = pMap[i];
    }
    mapPub.publish(rosMap);
}

void ScanProcess::pub_path( Eigen::Vector3d& pose,nav_msgs::Path &path,ros::Publisher &pub){
    current_time = ros::Time::now();
    geometry_msgs::PoseStamped this_pose_stamped;
    this_pose_stamped.pose.position.x = pose(0);
    this_pose_stamped.pose.position.y = pose(1);

    geometry_msgs::Quaternion goal_quat = tf::createQuaternionMsgFromYaw(pose(2));
    this_pose_stamped.pose.orientation.x = goal_quat.x;
    this_pose_stamped.pose.orientation.y = goal_quat.y;
    this_pose_stamped.pose.orientation.z = goal_quat.z;
    this_pose_stamped.pose.orientation.w = goal_quat.w;

    this_pose_stamped.header.stamp=current_time;
    this_pose_stamped.header.frame_id=map_frame;
    path.poses.push_back(this_pose_stamped);
    pub.publish(path);

    tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin( tf::Vector3(pose(0), pose(1), 0.0) );
    tf::Quaternion q (goal_quat.x, goal_quat.y, goal_quat.z, goal_quat.w);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "odom"));

}

Eigen::Vector3d ScanProcess::cal_delta_distence()
{

    Eigen::Vector3d d_pos;  //return value

    Eigen::Matrix3d Tnow,Tprev;
    double theta = last_pos(2);
    double x = last_pos(0);
    double y = last_pos(1);

    //前一次的位姿
    Tprev << cos(theta),-sin(theta),x,
            sin(theta), cos(theta),y,
            0,          0,       1;

    //当前的位姿
    x = now_pos(0);
    y = now_pos(1);
    theta = now_pos(2);
    Tnow << cos(theta),-sin(theta),x,
            sin(theta), cos(theta),y,
            0,          0,       1;

    //相对位姿
    Eigen::Matrix3d T = Tprev.inverse() * Tnow;
    d_pos(0) = T(0,2);
    d_pos(1) = T(1,2);
    d_pos(2) = atan2(T(1,0),T(0,0));
    return d_pos;
}

bool ScanProcess::getOdomPose(Eigen::Vector3d& pose, const ros::Time& t)
{

    // Get the robot's pose
    tf::Stamped<tf::Pose> ident (tf::Transform(tf::createQuaternionFromRPY(0,0,0),
                                               tf::Vector3(0,0,0)), t, base_frame);
    tf::Stamped<tf::Transform> odom_pose;
    try
    {
        tf_.transformPose(odom_frame, ident, odom_pose);
    }
    catch(tf::TransformException e)
    {
        ROS_WARN("Failed to compute odom pose, skipping scan (%s)", e.what());
        return false;
    }
    double yaw = tf::getYaw(odom_pose.getRotation());

    pose << odom_pose.getOrigin().x(),
            odom_pose.getOrigin().y(),
            yaw;
    return true;
}

void ScanProcess::SetMapParams(){
    mapParams.width = 100; //尺寸为 width * resolution
    mapParams.height = 100;
    mapParams.resolution = 0.1; //0.1m为一个栅格

    mapParams.log_free = -1;
    mapParams.log_occ = 2;

    //地图的原点，在地图的正中间
    mapParams.origin_x = -mapParams.width/2*mapParams.resolution;
    mapParams.origin_y = -mapParams.height/2*mapParams.resolution;

    mapParams.offset_x = 0;
    mapParams.offset_y = 0;

    pMap = new unsigned char[mapParams.width*mapParams.height];
    // 值代表该栅格内的状态

    //初始化为50
    for(int i = 0; i < mapParams.width * mapParams.height;i++)
        pMap[i] = 50;
}

GridIndex ScanProcess::ConvertWorld2GridIndex(double x,double y)
{
    GridIndex index;

    index.x = std::ceil((x - mapParams.origin_x) / mapParams.resolution) + mapParams.offset_x;
    index.y = std::ceil((y - mapParams.origin_y) / mapParams.resolution) + mapParams.offset_y;
    return index;
}
int ScanProcess::GridIndexToLinearIndex(GridIndex index)
{
    int linear_index;
    linear_index = index.x + index.y * mapParams.width;
    return linear_index;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "OccupanyMapping");
    ros::Time::init();
    std::cout << "Start" << std::endl;
    ScanProcess process;
    ros::spin();

    if(pMap != NULL)
        delete pMap;
}

