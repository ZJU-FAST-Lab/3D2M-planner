#ifndef PCSMAP_MANAGER_H
#define PCSMAP_MANAGER_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <Eigen/Eigen>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <string.h>

#define INF 0x7fffffff
#define HUGE_NUMBER 999999999

const double pi = 3.1415926535;

using namespace std;
using namespace Eigen;

class TravelGrid
{
    public:
        TravelGrid(){explored = false;}
        Vector3d plane_normal;
        Vector3d mass_center;
        double roughness;
        bool explored;
    public:
        typedef shared_ptr<TravelGrid> Ptr;
};

class GridMap
{
    public:
        void clearGridMap();
        void releaseMemory();
        void createGridMap(Vector3d boundary_xyzmin, Vector3d boundary_xyzmax);
        bool isInMap(Vector3d pos_w);
        bool isIndexValid(Vector3i index);
        bool isIndexValid(int ix,int iy, int iz);
        bool isIndexCanBeNeighbor(Vector3i index);
        bool isIndexOccupied(Vector3i index);
        bool isIndexOccupied(int ix, int iy, int iz);
        bool isCoordOccupied(Vector3d pos_w);
        bool isOverFloorInCells(Vector3i index, double distance);
        double getHeightToGround(Vector3d pos_w);
        double getHeightOfGround(Vector3d pos_w);
        double getHeightGap(Vector3d pos_w);
        Vector3i getGridIndex(Vector3d pos_w);
        Vector3d getGridCubeCenter(Vector3i index);
        Vector3d getGridCubeCenter(int ix, int iy, int iz);

        bool isGridBoundOfObs(Vector3i index);
        int getVoxelNum(int dim);
        void clearGridESDF();
        void clearGridPureESDF(double num);
        template <typename F_get_val, typename F_set_val>
        void fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim);
        void generateEmptyESDF();
        void generateESDF1dAt(int ix, int iy, int iz);
        void generateESDF3d();

        double getGridSDFValue(Vector3i index);
        double getGridSDFValue(int ix, int iy, int iz);
        double getSDFValue(Vector3d pos_w);
        Vector3d getSDFGrad(Vector3d pos_w);

        inline double getDistWithGradTrilinear(Eigen::Vector3d pos, Eigen::Vector3d& grad) {
            /*if (!isInMap( getGridCubeCenter(current_idx) )) {
                        grad.setZero();
                        return 0;
            }*/

            /* use trilinear interpolation */
            Eigen::Vector3d pos_m = pos - 0.5 * grid_resolution * Eigen::Vector3d::Ones();

            Eigen::Vector3i idx = getGridIndex(pos_m);

            Eigen::Vector3d idx_pos, diff;
            idx_pos = getGridCubeCenter(idx);

            diff = (pos - idx_pos) * (1.0/grid_resolution);

            double values[2][2][2];
            for (int x = 0; x < 2; x++) {
                for (int y = 0; y < 2; y++) {
                    for (int z = 0; z < 2; z++) {
                        Eigen::Vector3i current_idx = idx + Eigen::Vector3i(x, y, z);
                        if (!isInMap( getGridCubeCenter(current_idx) )) {
                            values[x][y][z] = 0;
                        }
                        values[x][y][z] = getGridSDFValue(current_idx);
                    }
                }
            }

            double v00 = (1 - diff[0]) * values[0][0][0] + diff[0] * values[1][0][0];
            double v01 = (1 - diff[0]) * values[0][0][1] + diff[0] * values[1][0][1];
            double v10 = (1 - diff[0]) * values[0][1][0] + diff[0] * values[1][1][0];
            double v11 = (1 - diff[0]) * values[0][1][1] + diff[0] * values[1][1][1];
            double v0 = (1 - diff[1]) * v00 + diff[1] * v10;
            double v1 = (1 - diff[1]) * v01 + diff[1] * v11;
            double dist = (1 - diff[2]) * v0 + diff[2] * v1;

            grad[2] = (v1 - v0) * (1.0/grid_resolution);
            grad[1] = ((1 - diff[2]) * (v10 - v00) + diff[2] * (v11 - v01)) * (1.0/grid_resolution);
            grad[0] = (1 - diff[2]) * (1 - diff[1]) * (values[1][0][0] - values[0][0][0]);
            grad[0] += (1 - diff[2]) * diff[1] * (values[1][1][0] - values[0][1][0]);
            grad[0] += diff[2] * (1 - diff[1]) * (values[1][0][1] - values[0][0][1]);
            grad[0] += diff[2] * diff[1] * (values[1][1][1] - values[0][1][1]);

            grad[0] *= (1.0/grid_resolution);

            // cout<<"dist = "<<dist<<" grad = "<<grad<<endl;

            return dist;
        }

    public:
        // max index size
        int X_size;
        int Y_size;
        int Z_size;

        // map size box
        Vector3d boundary_xyzmin;
        Vector3d boundary_xyzmax;

        // map resolution
        double grid_resolution;

        // Truncation distance
        double truncation_dis;
        int truncation_index;

        // debug
        bool debug_output;
    
        // maps
        pcl::PointCloud<pcl::PointXYZ>::Ptr grid_pointcloud;
        pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr grid_kdtree;
        double*** grid_map; 
        double*** grid_map_buffer_neg; 
        double*** grid_map_buffer_all; 
        double*** grid_esdf_buffer1; 
        double*** grid_esdf_buffer2; 
        double*** grid_esdf; 
        bool*** grid_map_flags;
    
    public:
        typedef shared_ptr<GridMap> Ptr;

};
/////////////////////////////////////////////////
class TravelGridMap : public GridMap
{
    public:
        TravelGrid::Ptr ***travel_grid;
        void clearGridMap();
        void clearGridESDF();
        void releaseMemory();
        void createGridMap(Vector3d boundary_xyzmin, Vector3d boundary_xyzmax);
        void copyTravelGrid( Vector3i src_index, Vector3i tgt_index);
        bool isTravelGridExplored(Vector3i index);
        Vector3d getSDFGrad(Vector3d pos_w);

        double getTravelCost(Vector3d pos_w);
        Vector3d getTravelPN(Vector3d pos_w);


        void generateESDF();

    public:
        typedef shared_ptr<TravelGridMap> Ptr;
};

/////////////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////////////
//// Surfel

class SurFel 
{
public:
    SurFel(Vector3d pos, Vector3d ang) {
        position = pos;
        pose     = ang;
        a        = pose.cross( Vector3d(1,0,0) );
        if( a.norm() == 0) { a = pose.cross( Vector3d(0,1,0) );}
        b        = pose.cross(a);
    }
    Vector3d position;
    Vector3d pose;
    Vector3d a,b;
public:
    typedef shared_ptr<SurFel> Ptr;
};
class SurFelMap 
{
public:
    vector<SurFel::Ptr> surfels;
    void init(ros::NodeHandle& nh);
    void renderSurfelMap();
    void appendSurfel(Vector3d pos, Vector3d ang);
public:
    ros::Publisher surfelmap_vis_pub;
public:
    typedef shared_ptr<SurFelMap> Ptr;
};
/////////////////////////////////////////////////////////////////////////

class PCSmapManager
{
    public:
        PCSmapManager();
        ~PCSmapManager();

        // param load
        void init(ros::NodeHandle& nh);

        // callback functions
        void rcvGlobalMapHandler(const sensor_msgs::PointCloud2& globalmap);
        void rcvOdomHandler(const nav_msgs::Odometry odom);
        void rcvRenderGrad(const std_msgs::Int16 msg);


        // other functions
        pcl::PointCloud<pcl::PointXYZ> snowProj(const sensor_msgs::PointCloud2& globalmap);
        void dropPoint(Vector3d &point);
        void deepDropPoint(Vector3d &point);
        void deepDropPointToPC(Vector3d &point);
        void dropIndex(Vector3i &index);



        // map size box
        Vector3d boundary_xyzmin;
        Vector3d boundary_xyzmax;

        // map resolutions
        double occupancy_resolution;
        double travelmap_resolution;

        // some params
        double offground_threshold;
        double backcost_step_roh;
        double backcost_roug_roh;
        double backcost_tilt_roh;
        double min_height;
        double max_height;
        int downproj;


        // as its name 统计滤波生成方块
        int sta_threshold;

        // map received flag
        bool recieved_globalmap;

        // debug
        bool debug_output;
        
        // global 3D occupancy grid map
        GridMap::Ptr occupancy_map;
        GridMap::Ptr snowed_occupancy_map;
        GridMap::Ptr zcost_map;
        // global 3D travelling cost map
        TravelGridMap::Ptr travelcost_map;

        SurFelMap::Ptr surfel_map;

    private:

        //
        //pcl::PointCloud<pcl::PointXYZ> global_pointcloud;
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_pointcloud;
        pcl::KdTreeFLANN<pcl::PointXYZ> global_kdtree;
        // ros
        ros::Subscriber globalmap_sub;
        ros::Subscriber odometry_sub;
        ros::Subscriber debug_grad_sub;

        ros::Publisher  globalmap_vis_pub;
        ros::Publisher  gridmap_vis_pub;
        ros::Publisher  costmap_vis_pub;
        ros::Publisher  voxel_vis_pub;
        ros::Publisher  rcvmap_signal_pub;

        ros::Publisher  debug_grad_pub;
        
     public:
         typedef shared_ptr<PCSmapManager> Ptr;
};


#endif