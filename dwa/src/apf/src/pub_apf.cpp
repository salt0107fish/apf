 #include"a_star_path/a_star.h"

bool apf::pubPath()
{
   
    if(map_flag)
    {
        pubMap.publish(map);
    }

    if(!start())//判断是否收到起点，终点，栅格地图
    {
        if(path_flag)//状态切换
        {
                pubThePath.publish(path_msg);//发布新路径
                pubThePath0.publish(path_msg0);//发布原路径
        }

        return false;
    }

    return true;
}