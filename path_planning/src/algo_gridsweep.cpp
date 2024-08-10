#include "algo_gridsweep.h"

std::vector<geometry_msgs::msg::PoseStamped> grid_sweep(const nav_msgs::msg::OccupancyGrid& grid) {
    std::cout<<"Started Grid Sweep Algorithm" <<std::endl;
    std::vector<geometry_msgs::msg::PoseStamped> path;
    int GRID_HEGHT = grid.info.height;
    int GRID_WIDTH = grid.info.width;

    for (int i=0;i<GRID_HEGHT;++i){
        if(i%2==0){
            for (int j=0;j<GRID_WIDTH;++j){
                geometry_msgs::msg::PoseStamped pose_cell;
                pose_cell.pose.position.x=static_cast<double>(j);
                pose_cell.pose.position.y=static_cast<double>(i);
                path.push_back(pose_cell);

            }
                }
            else{ // left to right
                for (int j=GRID_WIDTH-1;j>=0;--j){
                geometry_msgs::msg::PoseStamped pose_cell;
                pose_cell.pose.position.x=static_cast<double>(j);
                pose_cell.pose.position.y=static_cast<double>(i);
                path.push_back(pose_cell);

            }
            }


    }

    return path;
}
