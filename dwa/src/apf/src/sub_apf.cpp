 #include"a_star_path/a_star.h"

apf::apf(ros::NodeHandle& nh)//构造函数，创建对象时运行
{
    subInitialpose = nh.subscribe("/initialpose", 1000, &apf::subPoseCallback,this);     //创建订阅者subscribe（topic，buffer，&回调函数，this//当前类指针）

    subEndpose = nh.subscribe("/move_base_simple/goal", 1000, &apf::subendPoseCallback,this);

    subMap = nh.subscribe("/map", 1000, &apf::subMapCallback,this);                                //bag文件的通信订阅

    pubThePath = nh.advertise<nav_msgs::Path>("/path", 1000);                                 //创建发布path的发布者<数据结构>（topic，buffer）

    pubThePath0 = nh.advertise<nav_msgs::Path>("/path_ago", 1000);                                 //创建发布path的发布者<数据结构>（topic，buffer）
    
    pubMap = nh.advertise<nav_msgs::OccupancyGrid>("/rastermap",2);

    nh.getParam("k_att", k_att);
    nh.getParam("k_rep", k_rep);
    nh.getParam("limit_att", limit_att);
    nh.getParam("limit_rep", limit_rep);
    nh.getParam("limit_obs", limit_obs);

    map_flag = false;
    initial_pose_flag = false;
    end_pose_flag = false;
    path_flag = false;

    map_resolution = 0.2;                                                                                                                                 //地图分辨率

    dis2wall = 1.35/ map_resolution;                                                                                                             //生成路径与障碍物距离，碰撞检测

}

void apf::subPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
    initial_pose = msg; 

    if(!map_flag) //如果没有得到栅格地图，则不进行
    return;

    int x=(initial_pose.pose.pose.position.x - origin_x) / map_resolution;

    int y=(initial_pose.pose.pose.position.y - origin_y) / map_resolution;//获得起点的栅格位置

    if(grid_map[x][y] == 100 || image_dis.at<float>(y, x) < dis2wall)//如果将起点设置在障碍物上或离障碍物太近，返回
    {
        cout << "please set the right initial position" << endl;

        return;
    }

    if((initial_pose.pose.pose.position.x != 0||initial_pose.pose.pose.position.y != 0))
    {

        start_node = &maplist[x][y];//设置开始点的指针

        initial_pose_flag = true;   

        cout << "get start point   " << endl;

        cout << "start x=" << start_node->x << "  start y=" << start_node-> y << endl;

        return;
    }
}

void apf::subendPoseCallback(const geometry_msgs::PoseStamped& msg)
{
    end_pose = msg;

    if(!map_flag)
    return;

    int x = (end_pose.pose.position.x - origin_x) / map_resolution;

    int y = (end_pose.pose.position.y - origin_y) / map_resolution;//ROS坐标与栅格地图坐标进行坐标变换

    if(grid_map[x][y] == 100||image_dis.at<float>(y, x) < dis2wall)
    {
        cout << "please set the right end position" << endl;

        return;
    }

    if((end_pose.pose.position.x != 0||end_pose.pose.position.y != 0))
    {

        end_node = &maplist[x][y];

        end_pose_flag = true; 

        cout << "get end point   " << endl;

        cout << "end x=" << end_node->x << "  end y=" << end_node->y << endl;

        return;
    }
}

void apf::subMapCallback(const nav_msgs::OccupancyGrid& msg)
{
    if(map_flag == false)
    {
        row = msg.info.height;//栅格地图的行数

        col = msg.info.width;//栅格地图的列数

        origin_x = msg.info.origin.position.x ;//栅格地图原点，原为收到地图的左下角的点

        origin_y = msg.info.origin.position.y ;

        grid_map.resize(col);//设置地图列数
        
        for (size_t i = 0; i < grid_map.size(); i++)
        {
            grid_map[i].resize(row);//设置地图行数
        }//设置地图大小
        
        cout << "col=" << col <<"  row=" << row << endl;

//--------------------------------生成距离位图--------------------------------------
        image_map.create(row, col, CV_8UC1);    

        for(int i = 0; i < row; i++)
        {
            for(int j = 0; j < col; j++)
            image_map.at<uchar>(i, j) = 255;                                                     //opencv中255为白色，0为黑色
        }

        for(int j = 0;j < col;j++)
        {
            for(int i = 0;i < row;i++)
            {
                grid_map[j][i] = msg.data[i*col+j];                                               //对二维地图进行构建  grid_map[列][行] ，OccupancyGrid地图中0为白色，100为黑色

                if(grid_map[j][i] == 100)
                {
                    image_map.at<uchar>(i, j) = 0;
                }
            }
        }

        image_dis.create(row, col, CV_8UC1);    

        distanceTransform(image_map, image_dis, CV_DIST_L2, 5, CV_32FC1);//设置距离位图  ，可以直接通过image_dis.at<float>(y, x)得到当前节点离最近障碍物的距离。
    //-------------------------------------------------------------------------------------------------


    //------------------------用于生成发布的地图------------------------

        map.header.frame_id="map";
        map.header.stamp = ros::Time::now(); 
        map.info.origin.position.x = origin_x;
        map.info.origin.position.y = origin_y;
        map.info.origin.position.z = 0;
        map.info.resolution = map_resolution;    
        map.info.width = col;   
        map.info.height = row; 
        map.data.resize(col*row);

        // long int p[map.info.width*map.info.height] = {100};   

        for( int i = 0; i < row; ++i)
        {
            for ( int j = 0; j < col; ++j)
            {
                if(grid_map[col-1-j][i] == 0)
                map.data[i*col+col-1-j] = 0;

                else
                map.data[i*col+col-1-j] = 100;
            }
        }

        map_flag = true;

        cout << "get the map  " << endl;

        //-----------------------------------------------------------

        //--------------------------------------------------------------------------------------------
        //打开下两段注释，可以查看二维栅格地图与二值化地图
        //--------------------------------------------------------------------------------------------

//        Mat distShow;
//    distShow=Mat::zeros(image_dis.size(),CV_8UC1); //定义细化后的字符轮廓
//     for(int i=0;i<image_dis.rows;i++)
//        {
//              for(int j=0;j<image_dis.cols;j++)
//          {
//              distShow.at<uchar>(i,j)=image_dis.at<float>(i,j);
//             }
//        }
//         normalize(distShow,distShow,0,255,CV_MINMAX); //为了显示清晰，做了0~255归一化

//         //cv::imshow("img",image_map);
//         cv::imshow("dis",distShow);
//          cv::waitKey(0);

//---------------------------地图队列初始化-------------------------更新cost

        maplist.resize(image_dis.cols);
        for (size_t i = 0; i < image_dis.cols; i++)
        {
            maplist[i].resize(image_dis.rows);
        }

        for (size_t i = 0; i < image_dis.rows; i++)
        {
            for (size_t j = 0; j < image_dis.cols; j++)
            {
                maplist[ j][i].y = i;
                maplist[ j][i].x = j;
      //          maplist[ j][i].costF = 100000;    //将代价定义的足够大      
            }
        }
    }//初始化
}
