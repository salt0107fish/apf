#include<iostream>
#include<ros/ros.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include<nav_msgs/Path.h>
#include<geometry_msgs/PoseStamped.h>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <vector>
#include <queue>
#include<time.h>
#include <nav_msgs/Odometry.h>
#include<cmath>


using namespace std;
using namespace cv;

struct node//单个节点（节点就是栅格地图的一个栅格），包括xy，代价，是否找到最短路径，父节点
{
    int x,y;//x,y；
    // float costG;
    // float costH;
    // float costF;
    float U_att ;  //引力负
    float U_rep ;  //斥力正
    float U_sum ;
    bool visit=0;//表征是否进入关闭列表   visit=1,在关闭列表之中
    bool flag=0;//表征是否进入开启列表   flag=1,在开启列表之中

    node* parent_node;//用于指向父节点

    bool operator<(const node& other) const//与优先队列相关，将cost设置为排序标准  （自带的排序功能）
    {
        return U_sum<other.U_sum; 
    }
};

struct dian                                                                 //用于存储平滑后的路径
{
    float x,y;
};

class apf//定义类
{
    public://公有
    apf(ros::NodeHandle& nh);
    //----------------------------------函数均在core中定义------------------------------------
    bool start(void);//布尔值



    void apfextend(void);

    void simu(void);

    void findPath(void);

    void reset(void);

    void count_U(vector<vector<node>>& maplist, node* temp_node, node* goal_node, Mat image_map);  //image_map = 0黑点，image_map = 255，白点，计算这个点的总场

    void count_rep(vector<vector<node>>& maplist, node* temp_node, node* goal_node, node* end_node);  //计算两个点之间的斥力场

    void count_att(vector<vector<node>>& maplist, node* temp_node, node* goal_node);  //计算两个点之间的引力场



//-------------------------------------------------
    bool pubPath(void);

    float getH( node* path);
//---------------------------------布尔值均在core中使用--------------------------------------
    //发布路径
    bool path_flag;
    bool map_flag;

     ros::Publisher pubThePath;                                                                                                                                      //路径的publisher
     ros::Publisher pubThePath0;    
     
    ros::Publisher pubMap;                                                                                                                                             //地图的publisher

    nav_msgs::OccupancyGrid map;                                                                                                                         //ROS消息的数据结构

    private:
//shichang
    float k_att, k_rep, limit_att, limit_rep;
    int limit_obs;

    ros::NodeHandle nh;                                                                                                                                                 //内部访问的ros句柄

    ros::Subscriber subInitialpose;                                                                                                                             //起点订阅者回调
    void subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg);

    ros::Subscriber subEndpose;                                                                                                                                 //终点订阅者回调
    void subendPoseCallback(const geometry_msgs::PoseStamped& msg);

    ros::Subscriber subMap;                                                                                                                                            //地图订阅者回调
    void subMapCallback(const nav_msgs::OccupancyGrid& msg);

    geometry_msgs::PoseWithCovarianceStamped initial_pose;                                                                     //起点
    geometry_msgs::PoseStamped end_pose;                                                                                                         //终点

    node* start_node;                                                                                                                                                           //起点的指针
    node* end_node;                                                                                                                                                             //终点的指针
    node* finall_path;                                                                                                                                                            //路径指针

    vector<vector<node>> maplist;
    priority_queue<node> node_queue;                                                                                                                        //优先队列，会将代价最小的节点放到最前面。

      vector<dian> badpath;                                                                                                                                                 //储存原路经
      vector<dian> goodpath;                                                                                                                                              //储存平滑后的路径
    
    bool initial_pose_flag;
    bool end_pose_flag;

    //地图
    int col;                                                                                                                                                                                         //列
    int row;                                                                                                                                                                                       //行
    vector<vector<int>> grid_map;                                                                                                                                      //地图的二维容器----opencv里边只处理int类型
    float dis2wall;                                                                                                                                                                         //与障碍物的距离
    float origin_x;                                                                                                                                                                         //原点x
    float origin_y;                                                                                                                                                                         //原点y
    float map_resolution;                                                                                                                                                          //地图分辨率
    cv::Mat image_dis;                                                                                                                                                                 //二值化地图
    std::string image_path;
    Mat img;
    cv::Mat image_map;    
    
    //apf
    node tmp_node;
    node tmp_nodes;
    //寻路
    node* parent_path;
    nav_msgs::Path path_msg;
     nav_msgs::Path path_msg0;
    geometry_msgs::PoseStamped POSE_msg;
};
