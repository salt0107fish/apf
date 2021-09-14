 #include"a_star_path/a_star.h" 
 #include "ros/ros.h"
float S=0;
FILE*fp1=NULL;
FILE*fp2=NULL;
int flag_points = 1;
struct dian tmp1;
int ca;

// float k_att = ps->get<int>("k_att");   //引力系数，需要小，太小不容易出路，目前出路最多的为1,1000
// float k_rep = ps->get<int>("k_rep");    // 斥力系数

// float limit_att = ps->get<int>("limit_att");
// float limit_rep = ps->get<int>("limit_rep");
// int limit_obs = ps->get<int>("limit_obs");


bool apf::start()
{
    return(map_flag && end_pose_flag && initial_pose_flag);
}

int H_cal(int a, int b ,int c ,int d)//（a，b）：当前点------------（c，d）：目标点
{
    int s2;
   // s1 = sqrt((a-c)*(a-c)+(b-d)*(b-d));         //欧氏距离
    s2 = fabs(a-c)+fabs(b-d);                         //曼哈顿距离
    //s3 = (fabs(a-c)>fabs(b-d))?fabs(a-c):fabs(b-d);//切比雪夫距离
return s2;
}

void apf::count_att(vector<vector<node>>& maplist, node* temp_node, node* goal_node)  //计算两个点之间的引力场
{

    if(sqrt((pow(temp_node->x-goal_node->x,2)+pow(temp_node->y-goal_node->y,2))) <= limit_att)
    maplist[temp_node->x][temp_node->y].U_att = 0.5*k_att*(pow(temp_node->x-goal_node->x,2)+pow(temp_node->y-goal_node->y,2));
    else
    maplist[temp_node->x][temp_node->y].U_att = limit_att*k_att*sqrt(pow(temp_node->x-goal_node->x,2)+pow(temp_node->y-goal_node->y,2)) - 0.5*k_att*limit_att*limit_att;
}

void apf::count_rep(vector<vector<node>>& maplist, node* temp_node, node* goal_node, node* end_node)  //计算两个点之间的斥力场
{
    if(sqrt((pow(temp_node->x-goal_node->x,2)+pow(temp_node->y-goal_node->y,2))) <= limit_rep)
    maplist[temp_node->x][temp_node->y].U_rep += 0.5*k_rep*pow(1/limit_rep-1/sqrt((pow(temp_node->x-goal_node->x,2)+pow(temp_node->y-goal_node->y,2))),2)*((pow(temp_node->x-end_node->x,2)+pow(temp_node->y-end_node->y,2)));
    else
    maplist[temp_node->x][temp_node->y].U_rep += 0;
}

void apf::count_U(vector<vector<node>>& maplist, node* temp_node, node* goal_node, Mat image_map)  //image_map = 0黑点，image_map = 255，白点，计算这个点的总场
{
      count_att(maplist, temp_node, goal_node);
     // count_rep(maplist, temp_node, goal_node);

      for(int i = -limit_obs; i <= limit_obs; i++)
      {
          for(int j = -limit_obs; j <= limit_obs; j++)
          {
              
              if(image_map.at<uchar>(temp_node->y + j, temp_node->x + i) == 0)
              {
                  node* now_goal_node = new node;
                  now_goal_node->x = temp_node->x + i;
                  now_goal_node->y = temp_node->y + j;
                  count_rep(maplist, temp_node, now_goal_node, goal_node);
              }
          }
      }

      maplist[temp_node->x][temp_node->y].U_sum = maplist[temp_node->x][temp_node->y].U_att + maplist[temp_node->x][temp_node->y].U_rep;
}

void apf::apfextend()
{  
      count_U(maplist, start_node, end_node, image_map);
      node_queue.push(maplist[start_node->x][start_node->y]); 
      maplist[start_node->x][start_node->y].flag= 1;                                                                    //flag标签=1，表示该节点在openlist中
      maplist[start_node->x][start_node->y].visit= 0; 

      while(node_queue.size()!=0)                                                           // 队列中还有点，每个栅格为一个节点，探索完openlist所有节点后停止
        {   
            tmp_node = node_queue.top();                                                //优先队列里top就是最小的，找到队列中代价最小的节点作为父节点
            node_queue.pop();                                                                        //将代价最小的节点弹出（此节点最短路径已经找到）
            maplist[tmp_node.x][tmp_node.y].visit = true;                //该点已弹出，visit=1进入关闭列表
            maplist[tmp_node.x][tmp_node.y].flag= false; 

            if ((tmp_node.x==end_node->x)&&(tmp_node.y==end_node->y))//找到终点，退出。                                                 
                {cout<<"okkkkkkkkkkkkkkk"<<endl;break;}
            
            for(int i = -1;i<= 1;i++)                                                                    //节点扩展
            {
                for(int j =-1;j <= 1;j++)
                {
                    if(tmp_node.x + i >= col||tmp_node.x + i <= 0||tmp_node.y + j >= row || tmp_node.y + j <= 0)//跳过超过地图的点
                       continue;

                    if(image_dis.at<float>(tmp_node.y + j, tmp_node.x + i) < dis2wall)       //跳过离障碍物太近的点----碰撞检测 
                       continue;                                        //image_dis.at<float>(tmp_node.y + j, tmp_node.x + i) 该函数返回点(tmp_node.y + j, tmp_node.x + i) 到周围障碍物的最小距离[定义在sub中]

                    if(maplist[tmp_node.x + i][tmp_node.y + j].visit)//跳过已进找到路径的点,已经在关闭列表之中，跳过。
                       continue;

                    if(!maplist[tmp_node.x+i][tmp_node.y+j].flag)//未在开启列表之中，计算并添加。
                        {  
                            count_U(maplist, &tmp_node, end_node, image_map);
                            maplist[tmp_node.x + i][tmp_node.y + j].parent_node = &maplist[tmp_node.x][tmp_node.y];                                       //更新父节点
                            node_queue.push(maplist[tmp_node.x + i][tmp_node.y + j]);                                                                                                         //将新节点压入队列
                            maplist[tmp_node.x+i][tmp_node.y+j].flag=1;                                                                                                                                        //进入开启列表，flag=1
                        }
                }
             }
        }
 }



void apf::findPath()
{
    cout<<"start find path"<<endl;

    finall_path = end_node;
    ca=0;
    path_msg.poses.clear();
    path_msg0.poses.clear();

    while(finall_path)                                                              //将起点到终点的父节点依次连接，就得到了最终路径
    {
      tmp1.x=(finall_path->x)*map_resolution + origin_x;
      tmp1.y=(finall_path->y)*map_resolution + origin_y;
        POSE_msg.pose.position.x =  tmp1.x;
        POSE_msg.pose.position.y =  tmp1.y;
         path_msg0.poses.insert(path_msg0.poses.begin(), POSE_msg);
     //fp1= fopen("old.txt","w+");
     // fprintf(fp1,"%d %f  %f  \n",flag_points,tmp1.x ,tmp1.y );
        cout<<"OLD"<<tmp1.x<<","<<tmp1.y<<endl;
        finall_path = finall_path->parent_node;
        badpath.insert(badpath.begin(),tmp1);
        flag_points++;
    }
cout<<"总点数"<<badpath.size()<<endl;

 simu();                 //利用预瞄算法仿真得到平滑路径。

    while(ca<goodpath.size() )
    {
        POSE_msg.pose.position.x =  goodpath[ca].x;
        POSE_msg.pose.position.y =  goodpath[ca].y;
          cout<<"NEW"<< POSE_msg.pose.position.x<<","<<POSE_msg.pose.position.y<<endl;
         path_msg.poses.insert(path_msg.poses.begin(), POSE_msg);
        //fp2= fopen("new.txt","w+");
       //  fprintf(fp2,"%d %f  %f  \n",ca,POSE_msg.pose.position.x ,POSE_msg.pose.position.y );
         ca++;
    }

    path_msg.header.stamp = ros::Time::now();
    path_msg0.header.stamp = ros::Time::now();

    path_msg.header.frame_id = "map";
    path_msg0.header.frame_id = "map";
    path_flag = true;

    cout<<"start pub"<<endl;
}


void apf::reset( )
{
    initial_pose_flag = false;
    end_pose_flag = false;
    goodpath.clear();
    badpath.clear();
    for (size_t i = 0; i < image_dis.rows; i++)
    {
        for (size_t j = 0; j < image_dis.cols; j++)
        {
             maplist[ j][i].y = i;
            maplist[ j][i].x = j;                                                                           //更新地图
           maplist[ j][i].flag= 0;                                                                    //更新标签：flag标签=1，表示该节点在openlist中
           maplist[ j][i].visit= 0;                                                                         //visit=1 ,在关闭列表中
        maplist[j][ i].parent_node = NULL;
        }
    }
  while( node_queue.size()!=0)
    node_queue.pop(); 
    //cout<<"finish "<<node_queue.size()<<endl;

    cout<<"finish reset"<<endl;
}


void apf::simu()//模拟预瞄算法平滑路径
{
    int i;
    float  pre_dis=3;   //预瞄距离
    float  L=0.7;
    float v=1.1;
    float tim=1/20;
    int dis_min;
    int near_num=0;
    int aim_num=0;
    float dis_tmp;
    float  X0=badpath[0].x-0.1;                             //最初点是起点
    float  Y0=badpath[0].y-0.1;
    float yaw=3.14/2;
    cout<<"初始点"<<X0<<","<<Y0<<endl;
    tmp1.x=X0;
    tmp1.y=Y0;
    goodpath.insert(goodpath.begin(),tmp1);

    int fin_flag=0;                                                        //记录goodpath的坐标索引

    float kn,kn1;
    float aim_dis;
    float Xt,Yt;
    float pre_angle;
    float tmp_steer,R;
    float Lon,d_yaw;
    float dx,dy;
    float dis=sqrt(pow((badpath[badpath.size()-1].x-X0),2)+pow((badpath[badpath.size()-1].y-Y0),2));//badpath[badpath.size()-1]是终点

    //cout<<"最后一个"<<badpath.back().x<<","<<badpath.back().y<<endl;
    //cout<<"索引1---"<<badpath.at(badpath.size()-1).x<<","<<badpath.at(badpath.size()-1).y<<endl;
    //cout<<"索引2---"<<badpath[badpath.size()-1].x<<","<<badpath[badpath.size()-1].y<<endl;

    while(dis>0.5)
    {
    //   cout<<"当前点"<<X0<<"，"<<Y0<<endl;
    cout<<"到终点距离"<<dis<<endl;
        dis_min=INT_MAX;
        near_num=0;
        for (i=0;i<=badpath.size()-1;i++)                                   //找最近点
        {
            dis_tmp=sqrt(pow((badpath.at(i).x-X0),2)+pow((badpath.at(i).y-Y0),2));
        if (dis_tmp<dis_min)
        {dis_min=dis_tmp;
            near_num=i;}
        }
    //cout<<"当前最近点"<<badpath.at(near_num).x<<","<<badpath.at(near_num).y<<endl;

    if (dis_min>pre_dis)
        aim_num=near_num;
    else
    {

        for(i=near_num;i<badpath.size()-1;i++)   //找到符合预瞄距离的点
        {
            
            kn=sqrt(pow(badpath.at(i).x-badpath.at(near_num).x,2)+pow(badpath.at(i).y-badpath.at(near_num).y,2))-pre_dis;
            kn1=sqrt(pow(badpath.at(i+1).x-badpath.at(near_num).x,2)+pow(badpath.at(i+1).y-badpath.at(near_num).y,2))-pre_dis;
        //cout<<"kn***"<<kn<<"kn1****"<<kn1<<endl;
            if ((kn*kn1)<=0)
                {
                    if (kn1==0)
                {aim_num=i+1;
                    break;}
                else
                {aim_num=i;
                    break;}
                }
        } 

        int last_min = sqrt(pow(badpath.back().x-badpath.at(aim_num).x,2)+pow(badpath.back().y-badpath.at(aim_num).y,2));
        int now_temp,now_min;

        for(i=aim_num+1;i<badpath.size()-1;i++) 
        {
            now_min = sqrt(pow(badpath.back().x-badpath.at(i).x,2)+pow(badpath.back().y-badpath.at(i).y,2));
            now_temp = sqrt(pow(badpath.at(near_num).x-badpath.at(i).x,2)+pow(badpath.at(near_num).y-badpath.at(i).y,2));
            if(now_temp <= pre_dis && now_min <= last_min)
            {
                last_min = now_min;
                aim_num = i;
            }

        }
        
    }
    cout<<"预瞄目标点"<<aim_num<<endl;

    Xt= (badpath[aim_num].x-X0)*cos(yaw)+(badpath[aim_num].y-Y0)*sin(yaw);          
    Yt=-(badpath[aim_num].x-X0) *sin(yaw)+(badpath[aim_num].y-Y0)*cos(yaw);     

    aim_dis=sqrt(pow(Xt,2)+pow(Yt,2));

    pre_angle=Yt/aim_dis;
    tmp_steer=-atan(2* L *pre_angle/aim_dis)/3.14*180;
    if( fabs(tmp_steer)>25)
        tmp_steer=tmp_steer/fabs(tmp_steer)*25;
    //cout<<"转角"<<tmp_steer<<endl;

    R=L/tan(tmp_steer/180*3.14);
    if (fabs(R)<1.7)
    R=R/fabs(R)*1.7;
    //cout<<"半径"<<R<<endl;

    //更新坐标，夹角。
    Lon=v*tim;
    d_yaw=L/R;                   //弧度



    dx=R*sin(d_yaw);//局部坐标下前进方向位移
    dy=R-R*cos(d_yaw);//局部坐标下的横向位移

    yaw=yaw-d_yaw;
    //cout<<"航向"<<yaw<<endl;
    X0=X0+dx*cos(yaw)-dy*sin(yaw);
    Y0=Y0+dy*cos(yaw)+dx*sin(yaw);
    dis=sqrt(pow((badpath[badpath.size()-1].x-X0),2)+pow((badpath[badpath.size()-1].y-Y0),2));
    cout<<"当前点"<<X0<<"，"<<Y0<<endl;
    fin_flag++;
    tmp1.x=X0;
    tmp1.y=Y0;
    goodpath.insert(goodpath.begin(),tmp1);
    }
}

