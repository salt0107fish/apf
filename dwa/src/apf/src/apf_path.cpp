 #include"apf_path/apf.h"


int main(int argc,char** argv)
{
    ros::init(argc, argv, "apf_path");

    ROS_INFO("\033[1;32m---->\033[0m global path find Started.");   //“\033[字背景颜色；字体颜色m字符串\033[0m”代码中是一个绿色箭头

    ros::NodeHandle nh;    //节点句柄

    apf Apf(nh);  //类的构造函数，定义在sub中，完成变量的初始化和节点的通信设置
                                      //已订阅/initialpose、/move_base_simple/goal、/map三个话题

    ros::Rate rate(20);          //频率
    //clock_t start , finish;
    double totaltime;

    while(ros::ok())
    {
        ros::spinOnce();         //回调一次
        
        if(!Apf.pubPath())//publish.cpp
        {
            rate.sleep();             //20Hz的循环控制
            continue;
        }
     //   start=clock();

        Apf.apfextend();//广度优先扩展节点

        Apf.findPath();//找父节点

     //  finish=clock();

        Apf.reset();//
     
        rate.sleep();//20Hz的循环控制
       
    //totaltime=(double) (finish-start)/CLOCKS_PER_SEC; 
    //cout << "total time="<<totaltime << endl;
    }
}
