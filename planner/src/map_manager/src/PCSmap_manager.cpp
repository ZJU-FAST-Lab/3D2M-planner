#include "map_manager/PCSmap_manager.h"

// #define ENABLE_SNOWPROJ
#define RENDER_OCCUPANCY
// #define RENDER_ASTAR_SPACE
// #define RENDER_SURFEL

// #define SHOW_ORI_PC

// #define ENABLE_RANSAC_SURFEL
#define ENABLE_GRIDDROP_SURFEL


// #define ENABLE_ASTAR_EXPAND

PCSmapManager::PCSmapManager()
{
    recieved_globalmap = false;
    boundary_xyzmin = Vector3d(HUGE_NUMBER, HUGE_NUMBER, HUGE_NUMBER);
    boundary_xyzmax = Vector3d(-HUGE_NUMBER, -HUGE_NUMBER, -HUGE_NUMBER);

}

PCSmapManager::~PCSmapManager()
{
    // release memory
    occupancy_map -> releaseMemory();
}



void PCSmapManager::init(ros::NodeHandle& nh)
{
    nh.param("occupancy_resolution", occupancy_resolution, 0.1);
    nh.param("travelmap_resolution", travelmap_resolution, 0.1);
    nh.param("sta_threshold", sta_threshold, 2);
    nh.param("debug_output", debug_output, true);


    nh.param("backcost_step_roh", backcost_step_roh, 1.0);
    nh.param("backcost_roug_roh", backcost_roug_roh, 0.000002);
    nh.param("backcost_tilt_roh", backcost_tilt_roh, 100.0);

    nh.param("min_height", min_height, 0.3);
    nh.param("max_height", max_height, 0.6);
    nh.param("downproj", downproj, 0);

    occupancy_map.reset(new GridMap);

    occupancy_map->grid_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    occupancy_map->grid_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    occupancy_map->grid_resolution = occupancy_resolution;
    occupancy_map->debug_output    = debug_output;

    snowed_occupancy_map.reset(new GridMap);
    snowed_occupancy_map->grid_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    snowed_occupancy_map->grid_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    snowed_occupancy_map->grid_resolution = occupancy_resolution;
    snowed_occupancy_map->debug_output    = debug_output;

    zcost_map.reset(new GridMap);
    zcost_map->grid_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    zcost_map->grid_kdtree.reset(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    zcost_map->grid_resolution = occupancy_resolution;
    zcost_map->debug_output    = debug_output;

    travelcost_map.reset(new TravelGridMap);
    travelcost_map->grid_resolution     = travelmap_resolution;
    travelcost_map->debug_output        = debug_output;

    surfel_map.reset(new SurFelMap);
    surfel_map -> init(nh);


    globalmap_sub        = nh.subscribe("globalmap",1, &PCSmapManager::rcvGlobalMapHandler, this);
    odometry_sub         = nh.subscribe("odom", 1 , &PCSmapManager::rcvOdomHandler, this);
    debug_grad_sub       = nh.subscribe("/renderGrad", 1 , &PCSmapManager::rcvRenderGrad, this); 

    globalmap_vis_pub    = nh.advertise<sensor_msgs::PointCloud2>("globalmap_vis", 10);
    gridmap_vis_pub      = nh.advertise<sensor_msgs::PointCloud2>("gridmap_vis", 10);
    costmap_vis_pub      = nh.advertise<sensor_msgs::PointCloud2>("costmap_vis", 10);
    voxel_vis_pub        = nh.advertise<sensor_msgs::PointCloud2>("voxel_vis", 10);
    rcvmap_signal_pub    = nh.advertise<std_msgs::Empty>("rcvmap_signal", 10);

    debug_grad_pub       = nh.advertise<sensor_msgs::PointCloud2>("grad_vis",10);
}

pcl::PointCloud<pcl::PointXYZ> PCSmapManager::snowProj(const sensor_msgs::PointCloud2& globalmap)
{
    ros::Time before_planning = ros::Time::now();       
    double RADIUS = 0.3;
    double MAX_D  = 0.4;
    pcl::PointCloud<pcl::PointXYZ> snow_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> tmp_kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_global_pc;
    tmp_global_pc.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(globalmap, *tmp_global_pc);
    tmp_kdtree.setInputCloud(tmp_global_pc);
    Vector3d c_point;
    Vector3d n_point;
    pcl::PointXYZ c_pt;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    bool flag_success;
    int N = tmp_global_pc -> points.size();
    for(int i = 0 ; i < N; i++)
    {
        cout<<" snow Proj : "<< (i*100)/N << " %"<<endl;
        c_point(0) = tmp_global_pc -> points[i].x;
        c_point(1) = tmp_global_pc -> points[i].y;
        c_point(2) = tmp_global_pc -> points[i].z;
        c_pt = tmp_global_pc -> points[i];
        flag_success = true;
        if ( tmp_kdtree.radiusSearch (c_pt, RADIUS, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
        {
            for(int j = 0 ; j < pointIdxRadiusSearch.size() ; j++)
            {
                n_point(0) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].x;
                n_point(1) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].y;
                n_point(2) = tmp_global_pc -> points[ pointIdxRadiusSearch[j] ].z;
                if( n_point(2) > c_point(2) )
                {
                    double dis_xy = ( n_point.head(2) - c_point.head(2) ).norm() ;
                    double ang    = atan2( n_point(2) - c_point(2) , dis_xy );
                    if( ang > MAX_D ) { flag_success = false; break;}
                }

            }
        }
        if( flag_success ){
            snow_cloud.points.push_back( c_pt );
        }
    }
    ros::Time after_planning = ros::Time::now();
    std::cout<<"[PCSMap Manager] snow cost = " << 1000*(after_planning.toSec() - before_planning.toSec())<<"ms"<<std::endl;
    return snow_cloud;
}

void PCSmapManager::rcvOdomHandler(const nav_msgs::Odometry odom)
{

}

void PCSmapManager::rcvRenderGrad(const std_msgs::Int16 msg)
{

    int layer_z = msg.data;
    if(layer_z < 0){layer_z = 0;}
    if(layer_z >= occupancy_map -> Z_size){layer_z = occupancy_map -> Z_size - 1;}
    
    sensor_msgs::PointCloud2 grad_vis;
    pcl::PointCloud<pcl::PointXYZI> grad_vis_pcl;
    pcl::PointXYZI pt;
    Vector3d pos;
    double z = layer_z * occupancy_resolution + boundary_xyzmin(2);
    double i;

    double resolution = 0.04;
    for (double x = boundary_xyzmin(0)+0.5 ; x < boundary_xyzmax(0)-0.5; x += resolution)
	{
		for (double y = boundary_xyzmin(1)+0.5 ; y < boundary_xyzmax(1)-0.5; y += resolution)
		{
            pos = Vector3d(x,y,z);
            pt.x = x;
            pt.y = y;
            pt.z = z;// + 0.01 * travelcost_map -> getSDFValue(pos);
            i = travelcost_map -> getSDFValue(pos);
            // i = occupancy_map -> getSDFValue(pos);
            i = sqrt(i);
            pt.intensity = i;
            grad_vis_pcl.points.push_back(pt);
		}
	}

    pcl::toROSMsg( grad_vis_pcl, grad_vis);
    grad_vis.header.frame_id = "world";
    debug_grad_pub.publish(grad_vis);

}

void PCSmapManager::rcvGlobalMapHandler(const sensor_msgs::PointCloud2& globalmap)
{
    if(recieved_globalmap == true){return;}

    sensor_msgs::PointCloud2 globalmap_vis;
    pcl::PointCloud<pcl::PointXYZ> gridmap_vis;

    pcl::PointCloud<pcl::PointXYZ> global_cloud;
    pcl::fromROSMsg( globalmap, global_cloud);


    std::cout<<"[MapHandler] Get global point cloud!" <<std::endl;
    std::cout<<"[MapHandler] get boundary " <<std::endl;


    ros::Time before_planning = ros::Time::now();
      
    

// STEP1 生成占用栅格地图
    global_pointcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(global_cloud, *global_pointcloud); //复制
    global_kdtree.setInputCloud(global_pointcloud);

    for(int i = 0 ; i < global_cloud.points.size(); i++)
    {
        if( global_cloud.points[i].x > boundary_xyzmax(0) ){
            boundary_xyzmax(0) = global_cloud.points[i].x;
        }

        if( global_cloud.points[i].x < boundary_xyzmin(0) ){
            boundary_xyzmin(0) = global_cloud.points[i].x;
        }

        if( global_cloud.points[i].y > boundary_xyzmax(1) ){
            boundary_xyzmax(1) = global_cloud.points[i].y;
        }

        if( global_cloud.points[i].y < boundary_xyzmin(1) ){
            boundary_xyzmin(1) = global_cloud.points[i].y;
        }

        if( global_cloud.points[i].z > boundary_xyzmax(2) ){
            boundary_xyzmax(2) = global_cloud.points[i].z;
        }

        if( global_cloud.points[i].z < boundary_xyzmin(2) ){
            boundary_xyzmin(2) = global_cloud.points[i].z;
        }
    }

    std::cout<<"[MapHandler] get boundary finished!" <<std::endl;
    std::cout<<"[MapHandler] boundary x : " <<boundary_xyzmin(0)<< " -> "<<boundary_xyzmax(0)<<std::endl;
    std::cout<<"[MapHandler] boundary y : " <<boundary_xyzmin(1)<< " -> "<<boundary_xyzmax(1)<<std::endl;
    std::cout<<"[MapHandler] boundary z : " <<boundary_xyzmin(2)<< " -> "<<boundary_xyzmax(2)<<std::endl;
    std::cout<<"[MapHandler] create empty map" <<std::endl;

    // create 3D grid maps

    occupancy_map  -> createGridMap( boundary_xyzmin, boundary_xyzmax );
    travelcost_map -> createGridMap( boundary_xyzmin, boundary_xyzmax );
    zcost_map      -> createGridMap( boundary_xyzmin, boundary_xyzmax );


    std::cout<<"[MapHandler] create empty map finished!" <<std::endl;
    // generate occupancy grid map
    
    std::cout<<"[MapHandler] genrate occupancy map." <<std::endl;
    Vector3i index;
    Vector3d center_coord;
    pcl::PointXYZ  s_point;
    for(int i = 0 ; i < global_cloud.points.size(); i++)
    {
        index = occupancy_map -> getGridIndex(  Vector3d(global_cloud.points[i].x, global_cloud.points[i].y, global_cloud.points[i].z)  );
        if( occupancy_map -> isIndexValid(index) )
        {
            occupancy_map -> grid_map[ index(0) ][ index(1) ][ index(2) ] ++;
        }
    }

    for (int i = 0; i < occupancy_map -> X_size; i++)
	{
		for (int j = 0; j < occupancy_map -> Y_size; j++)
		{
			for (int k = 0; k < occupancy_map -> Z_size; k++)
			{
                if(occupancy_map -> grid_map[i][j][k] >= sta_threshold) {
                    
                    center_coord = occupancy_map -> getGridCubeCenter(i, j, k);
                    s_point.x    = center_coord(0);
                    s_point.y    = center_coord(1);
                    s_point.z    = center_coord(2);
#ifdef RENDER_OCCUPANCY
                    gridmap_vis.push_back(s_point);
#endif
                    occupancy_map -> grid_pointcloud->push_back(s_point);
                    occupancy_map -> grid_map[i][j][k] = 1; 
                }
                else {  occupancy_map -> grid_map[i][j][k] = 0; }
			}
		}
	}

//STEP1.5 构建ZCOSTMAP
    zcost_map -> generateEmptyESDF();
    double hg, gap;
    double hex = (max_height - min_height) * 0.3;
    for (int i = 0; i < occupancy_map -> X_size; i++)
    {
        for (int j = 0; j < occupancy_map -> Y_size; j++)
        {
            for (int k = 0; k < occupancy_map -> Z_size; k++)
            {
                if( occupancy_map -> isIndexOccupied(i,j,k) == true ){
                    zcost_map -> grid_map[i][j][k] = 1;
                    zcost_map -> grid_esdf[i][j][k] = occupancy_map -> getGridCubeCenter(i,j,k)(2);
                }
                else{
                    zcost_map -> grid_map[i][j][k] = 0;
                    hg  = occupancy_map -> getHeightOfGround( occupancy_map -> getGridCubeCenter(i,j,k) );
                    gap = occupancy_map -> getHeightGap( occupancy_map -> getGridCubeCenter(i,j,k) );
                    if (gap <= min_height){
                        zcost_map -> grid_map[i][j][k] = 1; 
                        zcost_map -> grid_esdf[i][j][k] = occupancy_map -> getGridCubeCenter(i,j,k)(2);
                        continue;
                    }
                    else{
                        if( gap > max_height + hex){ zcost_map -> grid_esdf[i][j][k] = max_height;}
                        else if( gap < min_height + hex){ zcost_map -> grid_esdf[i][j][k] = min_height;}
                        else { zcost_map -> grid_esdf[i][j][k] = gap - hex;}
                        zcost_map -> grid_esdf[i][j][k] += hg;
                    }
                }
                // if(zcost_map -> grid_esdf[i][j][k] < 0){
                //     std::cout<<"1,"<<zcost_map -> grid_esdf[i][j][k]<<" "<<hg<<" "<<gap<<std::endl;
                // }
            }
        }
    }


    std::cout<<"[MapHandler] STEP1 genrate occupancy map finished!" <<std::endl;

//STEP2 雪地工程
#ifdef ENABLE_SNOWPROJ
// use snow proj
    global_cloud = snowProj( globalmap );

#ifdef SHOW_ORI_PC
    globalmap_vis = globalmap;
#else
    pcl::toROSMsg( global_cloud, globalmap_vis);
#endif

    snowed_occupancy_map -> createGridMap( boundary_xyzmin, boundary_xyzmax );

    // the occupancymap after snow proj
    for(int i = 0 ; i < global_cloud.points.size(); i++)
    {
        index = snowed_occupancy_map -> getGridIndex(  Vector3d(global_cloud.points[i].x, global_cloud.points[i].y, global_cloud.points[i].z)  );
        snowed_occupancy_map -> grid_map[ index(0) ][ index(1) ][ index(2) ] ++;
    }

    for (int i = 0; i < snowed_occupancy_map -> X_size; i++)
	{
		for (int j = 0; j < snowed_occupancy_map -> Y_size; j++)
		{
			for (int k = 0; k < snowed_occupancy_map -> Z_size; k++)
			{
                if(snowed_occupancy_map -> grid_map[i][j][k] >= sta_threshold) {
                    
                    center_coord = snowed_occupancy_map -> getGridCubeCenter(i, j, k);
                    s_point.x    = center_coord(0);
                    s_point.y    = center_coord(1);
                    s_point.z    = center_coord(2);
                    snowed_occupancy_map -> grid_pointcloud->push_back(s_point);
                    snowed_occupancy_map -> grid_map[i][j][k] = 1; 
                }
                else {  snowed_occupancy_map -> grid_map[i][j][k] = 0; }
			}
		}
	}

#else
    //pcl::fromROSMsg( globalmap, global_cloud);
    globalmap_vis = globalmap;
    snowed_occupancy_map = occupancy_map;
#endif

    std::cout<<"[MapHandler] STEP2 finished!" <<std::endl;
//STEP3 build A* space
    int index_exp_x ,index_exp_y, index_exp_z;
    int exp_ind = 0;
#ifdef ENABLE_ASTAR_EXPAND
    exp_ind = ceil(0.25/occupancy_resolution);
#endif
    for (int i = 0; i < snowed_occupancy_map -> X_size; i++)
	{
		for (int j = 0; j < snowed_occupancy_map -> Y_size; j++)
		{
			for (int k = 1; k < snowed_occupancy_map -> Z_size; k++)
			{
//                 if( occupancy_map -> grid_map[i][j][k-1] == 1 &&
//                     occupancy_map -> grid_map[i][j][k]   == 0 ) {
//                     occupancy_map -> grid_map_flags[i][j][k] = 1; 
//                     center_coord = occupancy_map -> getGridCubeCenter(i, j, k);
// #ifdef RENDER_ASTAR_SPACE
//                         center_coord = occupancy_map -> getGridCubeCenter(i, j, k);
//                         s_point.x    = center_coord(0);
//                         s_point.y    = center_coord(1);
//                         s_point.z    = center_coord(2);
//                         gridmap_vis.push_back(s_point);
// #endif
//                 }
                if( snowed_occupancy_map -> grid_map[i][j][k-1] == 1)
                {
                    bool defeat = false;
                    for(int x_exp = -exp_ind ; x_exp <= exp_ind ; x_exp++)
                    {
                        for(int y_exp = -exp_ind ; y_exp <= exp_ind ; y_exp++)
                        {
                            index_exp_x = i+x_exp;
                            index_exp_y = j+y_exp;
                            index_exp_z = k;
                            if(snowed_occupancy_map -> isIndexValid(index_exp_x, index_exp_y, index_exp_z) == false){defeat = true;break;}
                            if(!((snowed_occupancy_map -> grid_map[index_exp_x][index_exp_y][index_exp_z]   == 0 &&
                                  index_exp_z-1 >= 0 &&
                                  snowed_occupancy_map -> grid_map[index_exp_x][index_exp_y][index_exp_z-1] != 0 ) ||
                                ( snowed_occupancy_map -> grid_map[index_exp_x][index_exp_y][index_exp_z] != 0 &&
                                    index_exp_z+1 < snowed_occupancy_map -> Z_size &&
                                  snowed_occupancy_map -> grid_map[index_exp_x][index_exp_y][index_exp_z+1] == 0 )) )
                                  {defeat = true;break;}
                        }
                        if(defeat){break;}
                    }
                    if(!defeat){
                        occupancy_map -> grid_map_flags[i][j][k] = 1; 
#ifdef RENDER_ASTAR_SPACE
                        center_coord = occupancy_map -> getGridCubeCenter(i, j, k);
                        s_point.x    = center_coord(0);
                        s_point.y    = center_coord(1);
                        s_point.z    = center_coord(2);
                        gridmap_vis.push_back(s_point);
#endif
                    }
                }
			}
		}
	}

#ifdef ENABLE_SNOWPROJ
    snowed_occupancy_map -> releaseMemory(); //release memory
#endif

    ROS_INFO("generate occupancy map and A* space done!");
    occupancy_map -> grid_kdtree -> setInputCloud(occupancy_map -> grid_pointcloud);


    std::cout<<"[MapHandler] STEP3 finished!" <<std::endl;

//PR STEP 可视化点云地图
    globalmap_vis.header.frame_id = "world";
    globalmap_vis_pub.publish(globalmap_vis);

// STEP4 生成ESDF地图
    // generate occupancy ESDF map
    occupancy_map -> generateESDF3d();

// STEP5 生成 SURFEL 地图 和 TRAVEL VALUE

    Vector3i ori_index, dropped_index;
    Vector3d surfel_center, surfel_pose;
    int N = occupancy_map -> Z_size;

#ifdef ENABLE_GRIDDROP_SURFEL
    
    for (int k = 0 ; k < N ; k++)
	{
        cout<<" surfel : "<< (k*100)/N << " %"<<endl;
		for (int j = 0; j < occupancy_map -> Y_size; j++)
		{
			for (int i = 0; i < occupancy_map -> X_size; i++)
			{
                //center_coord   = occupancy_map -> getGridCubeCenter(i, j, k);
                //ori_index      = occupancy_map -> getGridIndex(center_coord);
                ori_index        = Vector3i(i,j,k);
                dropped_index    = ori_index;
                dropIndex(dropped_index);

                if( travelcost_map -> isTravelGridExplored(dropped_index)) { 
                    travelcost_map -> copyTravelGrid( dropped_index, ori_index );
                    continue; 
                }
                surfel_center    =  occupancy_map -> getGridCubeCenter(dropped_index);
                surfel_center(2) += 0.5 * (occupancy_map->grid_resolution);
                // s_point.x    = surfel_center(0);
                // s_point.y    = surfel_center(1);
                // s_point.z    = surfel_center(2);
                // gridmap_vis.push_back(s_point);
                occupancy_map -> getDistWithGradTrilinear(surfel_center, surfel_pose);
                surfel_map -> appendSurfel( surfel_center, surfel_pose );

                if( occupancy_map -> isIndexOccupied(dropped_index) == false ){ 
                    surfel_center(2) = -1e2; //地板空洞
                }

                travelcost_map -> travel_grid[ dropped_index(0) ][ dropped_index(1) ][ dropped_index(2) ]->mass_center  = surfel_center;
                travelcost_map -> travel_grid[ dropped_index(0) ][ dropped_index(1) ][ dropped_index(2) ]->plane_normal = surfel_pose;
                travelcost_map -> travel_grid[ dropped_index(0) ][ dropped_index(1) ][ dropped_index(2) ]->explored = true;
            }
        }
    }
#endif

#ifdef ENABLE_RANSAC_SURFEL

    double radius = travelcost_map->grid_resolution + offground_threshold;
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    Vector3d mass_center;
    Vector3d plane_normal;
    Vector3d vec_p;
    double roughness = 0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    // RANSAC
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);


    for (int k = 0; k < N; k++)
	{
        cout<<" surfel : "<< (k*100)/N << " %"<<endl;
		for (int j = 0; j < travelcost_map -> Y_size; j++)
		{
			for (int i = 0; i < travelcost_map -> X_size; i++)
			{   

                center_coord = travelcost_map -> getGridCubeCenter(i, j, k);
                ori_index = travelcost_map -> getGridIndex(center_coord);
                deepDropPoint(center_coord);
                index = travelcost_map -> getGridIndex(center_coord);
                

                if( travelcost_map -> isTravelGridExplored(index)) { 
                    travelcost_map -> copyTravelGrid( index, ori_index );
                    continue; 
                }

                travelcost_map->travel_grid[ index(0) ][ index(1) ][ index(2) ]->explored = true;
                s_point.x    = center_coord(0);
                s_point.y    = center_coord(1);
                s_point.z    = center_coord(2);

                if ( global_kdtree.radiusSearch (s_point, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
                {
                    if(pointIdxRadiusSearch.size() < 1){continue ;}
                    mass_center = Vector3d(0,0,0);
                    temp_cloud->points.resize( pointIdxRadiusSearch.size() );
                    for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
                    {
                        temp_cloud->points[i] = global_pointcloud->points[ pointIdxRadiusSearch[i] ];
                        mass_center(0) = (mass_center(0) * i + global_pointcloud->points[ pointIdxRadiusSearch[i] ].x ) / (i+1);
                        mass_center(1) = (mass_center(1) * i + global_pointcloud->points[ pointIdxRadiusSearch[i] ].y ) / (i+1);
                        mass_center(2) = (mass_center(2) * i + global_pointcloud->points[ pointIdxRadiusSearch[i] ].z ) / (i+1);
                    }   
                    travelcost_map->travel_grid[ index(0) ][ index(1) ][ index(2) ]->mass_center = mass_center;
                    surfel_map -> appendSurfel( center_coord , Vector3d(0,0,1) );


                    //RANSAC
                    
                    seg.setInputCloud (temp_cloud);
                    seg.segment (*inliers, *coefficients);
                    plane_normal(0) = coefficients->values[0];
                    plane_normal(1) = coefficients->values[1];
                    plane_normal(2) = coefficients->values[2];
                    plane_normal.normalize();
                    if(plane_normal(2) < 0){ plane_normal *= -1;}
                    travelcost_map->travel_grid[ index(0) ][ index(1) ][ index(2) ]->plane_normal = plane_normal;
                    mass_center = travelcost_map->travel_grid[ index(0) ][ index(1) ][ index(2) ]->mass_center;
                    surfel_map -> appendSurfel( mass_center, plane_normal );
                    

                    //roughness , don't use it.
                    
                    for(int i = 0 ; i < pointIdxRadiusSearch.size() ; i++)
                    {
                       vec_p(0) = temp_cloud->points[i].x - mass_center(0);
                       vec_p(1) = temp_cloud->points[i].y - mass_center(1);
                       vec_p(2) = temp_cloud->points[i].z - mass_center(2);
                       vec_p.normalize();
                       roughness += abs( vec_p.transpose() * plane_normal );
                    }
                    travelcost_map->travel_grid[ index(0) ][ index(1) ][ index(2) ]->roughness = roughness;    
                }
			}
		}
	}
#endif

    // PR STEP 可视化 SURFEL
#ifdef RENDER_SURFEL
    for(int i = 0 ; i < 100 ; i++)
    {
        surfel_map -> renderSurfelMap();
        ros::Duration(0.01).sleep();
    }
#endif


// STEP6 DKR DIF
    
    Vector3d value_dvector;
    Vector3i c_ind, cplus_ind, cmin_ind, c_indup;
    
    Vector3d dposition;
    double cost_dist;
    for (int k = 0; k < travelcost_map -> Z_size; k++)
	{
        cout<<"space dS: " <<(double)k*100/travelcost_map -> Z_size <<" %"<<endl;
		for (int j = 1; j < travelcost_map -> Y_size-1; j++)
		{
			for (int i = 1; i < travelcost_map -> X_size-1; i++)
			{   
                c_ind       = Vector3i(i,j,k);
                cplus_ind   = Vector3i(i+1, j, k);
                cmin_ind    = Vector3i(i-1, j, k);
                c_indup     = Vector3i(i, j, k+1);
                if(occupancy_map -> isIndexOccupied(c_ind) || occupancy_map -> isIndexOccupied(c_indup)) 
                { 
                    travelcost_map->grid_map[ c_ind(0) ][ c_ind(1) ][ c_ind(2) ] = 0 ;
                    continue;
                }

                dposition   = ( travelcost_map->travel_grid[ cplus_ind(0) ][ cplus_ind(1) ][ cplus_ind(2) ]->mass_center 
                              - 2*travelcost_map->travel_grid[   c_ind(0)   ][   c_ind(1)   ][   c_ind(2)   ]->mass_center
                              + travelcost_map->travel_grid[ cmin_ind(0)  ][ cmin_ind(1)  ][ cmin_ind(2)  ]->mass_center );
                cost_dist   = dposition.norm() - occupancy_map->grid_resolution; 


                c_ind       = Vector3i(i,j, k);
                cplus_ind   = Vector3i(i,j+1, k);
                cmin_ind    = Vector3i(i,j-1, k);
                dposition   = ( travelcost_map->travel_grid[ cplus_ind(0) ][ cplus_ind(1) ][ cplus_ind(2) ]->mass_center 
                              - 2*travelcost_map->travel_grid[   c_ind(0)   ][   c_ind(1)   ][   c_ind(2)   ]->mass_center
                              + travelcost_map->travel_grid[ cmin_ind(0)  ][ cmin_ind(1)  ][ cmin_ind(2)  ]->mass_center );
                cost_dist  += dposition.norm() - occupancy_map->grid_resolution; 


                cost_dist *= 40;
                //if(cost_dist > 0 && cost_dist < 2){cost_dist *= 10;}
                // if(cost_dist > 0){cost_dist = 100;}
                if(cost_dist > 400){cost_dist = 400;}//限幅
                if(cost_dist < 100){cost_dist = 0;}//限幅
                //if(cost_dist > 2){cost_dist = 400;}//二值

                travelcost_map->grid_map[ c_ind(0) ][ c_ind(1) ][ c_ind(2) ] = cost_dist ;
			}
		}
	}

// STEP7 根据 TRAVEL GRID COST 生成水平场
    //这个ESDF是 override 过的，和 occupancy_map 的不一样
    travelcost_map -> generateESDF();


///////////////////////////////////////////////////////////////////////////////////////////

// PR STEP 可视化 TRAVEL GRID COST
    
    double cost;
    pcl::PointCloud<pcl::PointXYZI> cost_cloud;
    pcl::PointXYZI ipoint;

    for (int i = 0; i < travelcost_map -> X_size-1; i++)
	{
		for (int j = 0; j < travelcost_map -> Y_size-1; j++)
		{
			for (int k = 0; k < travelcost_map -> Z_size; k++)
			{   
                center_coord = travelcost_map -> getGridCubeCenter( Vector3i(i,j,k)  );
                cost = travelcost_map -> getTravelCost(center_coord);
                if(cost > 0.2){
                    ipoint.x = center_coord(0);
                    ipoint.y = center_coord(1);
                    ipoint.z = center_coord(2);
                    ipoint.intensity = cost;
                    if(ipoint.intensity > 1000) {ipoint.intensity = 1000;}
                    cost_cloud.push_back(ipoint);
                }  
			}
		}
	}
    sensor_msgs::PointCloud2  cost_cloud_vis , occupancy_map_vis;
    pcl::toROSMsg(cost_cloud, cost_cloud_vis);
    pcl::toROSMsg(gridmap_vis, occupancy_map_vis);
    cost_cloud_vis.header.frame_id    = "world";
    occupancy_map_vis.header.frame_id = "world";
    costmap_vis_pub.publish(cost_cloud_vis);
    gridmap_vis_pub.publish(occupancy_map_vis);
    

    ///////////////////////////////////////////////


    //可视化某个 esdf
/*
    pcl::PointCloud<pcl::PointXYZI> cost_cloud;
    pcl::PointXYZI ipoint;
    sensor_msgs::PointCloud2  cost_cloud_vis;

    double cost;


    double k = 0.4;
    cost_cloud.points.clear();
    for (double i = boundary_xyzmin(0); i < boundary_xyzmax(0); i+=0.02)
	{
		for (double j = boundary_xyzmin(1); j < boundary_xyzmax(1); j+=0.02)
		{ 
            center_coord(0) = ipoint.x = i;
            center_coord(1) = ipoint.y = j;
            center_coord(2) = ipoint.z = k;
            //cost =  travelcost_map -> getSDFValue(center_coord);
            cost =  occupancy_map -> getSDFValue(center_coord);
            ipoint.intensity = cost;
            ipoint.z += cost;
            cost_cloud.points.push_back(ipoint);
		}
	}

    pcl::toROSMsg(cost_cloud, cost_cloud_vis);
    cost_cloud_vis.header.frame_id = "world";
    voxel_vis_pub.publish(cost_cloud_vis);
*/


    ///////////////////////////////////////////////////////
    // 可视化 occupancy grid map
/*
    sensor_msgs::PointCloud2  gridmap_cloud;

    pcl::toROSMsg(gridmap_vis, gridmap_cloud);
    gridmap_cloud.header.frame_id = "world";
    gridmap_vis_pub.publish(gridmap_cloud);
*/

    // publish esdf grid map
    /*
    sensor_msgs::PointCloud2  sdf_cloud;
    pcl::PointXYZI  i_point;
    pcl::PointCloud<pcl::PointXYZI> sdfmap_vis;

    double k = 0.3;
    double sdf_v;
    Vector3d grad;
    for (double i = boundary_xyzmin(0); i < boundary_xyzmax(0); i += 0.04)
	{
		for (double j = boundary_xyzmin(1); j < occupancy_map -> boundary_xyzmax(1); j += 0.04)
		{
			//for (double k = boundary_xyzmin(2); k < occupancy_map -> boundary_xyzmax(2) / 5; k+=0.1)
			//{
                //sdf_v = occupancy_map -> getSDFValue( Vector3d(i,j,k) );
                grad  = occupancy_map -> getSDFGrad( Vector3d(i,j,k) );
                sdf_v = grad(2);
                i_point.x    = i;
                i_point.y    = j;
                i_point.z    = k + sdf_v;
                //index = occupancy_map -> getGridIndex( Vector3d(i,j,k));
                i_point.intensity = sdf_v;
                sdfmap_vis.push_back(i_point);
			//}
		}
	}
    //pcl::toROSMsg(*occupancy_map -> grid_pointcloud, sdf_cloud);
    pcl::toROSMsg(sdfmap_vis, sdf_cloud);
    sdf_cloud.header.frame_id = "world";
    voxel_vis_pub.publish(sdf_cloud);
    */



    // flag trig
    ros::Time after_planning = ros::Time::now();
    std::cout<<"[PCSMap Manager] total cost = " << 1000*(after_planning.toSec() - before_planning.toSec())<<"ms"<<std::endl;
    recieved_globalmap = true;

    std_msgs::Empty s;
    rcvmap_signal_pub.publish(s);
    
}

void PCSmapManager::dropPoint(Vector3d &point)
{
  Vector3i grid_index = occupancy_map -> getGridIndex(point);
  Vector3i grid_index_scan = grid_index;
  
  if( occupancy_map -> isOverFloorInCells(grid_index, offground_threshold) ) {return ;}

  for( int sz = grid_index(2) - 1 ; sz >= 0; sz-- )
  {
    grid_index_scan(2) = sz;
    if( occupancy_map -> isOverFloorInCells(grid_index_scan, offground_threshold) )
    {
      point = occupancy_map -> getGridCubeCenter( grid_index_scan );
      //ROS_WARN("This point is unreachable for ground vehicle, corrected.");
      return ;
    }
  }
  ROS_ERROR("There is a hole in the floor! Relocate please!");
}

void PCSmapManager::dropIndex(Vector3i &index)
{
    Vector3i index_scan = index;
    if( occupancy_map -> isIndexOccupied(index) ) {
        for( int sz = index(2) + 1 ; sz < occupancy_map->Z_size; sz++ )
        {
            index_scan(2) = sz;
            index = index_scan;
            if( !occupancy_map -> isIndexOccupied(index_scan) )
            {
                index(2) = index(2) - 1;
                return ;
            }
        }
        return ;
    }
    else {

        for( int sz = index(2) - 1 ; sz >= 0; sz--)
        {
            index_scan(2) = sz;
            index = index_scan;
            if( occupancy_map -> isIndexOccupied(index_scan) )
            {
                return ;
            }
        }
        return ;
    }
}

void PCSmapManager::deepDropPoint(Vector3d &point)
{
    Vector3i grid_index = occupancy_map -> getGridIndex(point);
    if( occupancy_map -> isOverFloorInCells(grid_index, offground_threshold) ) {return ;}

    Vector3i grid_index_scan = grid_index;
    for( int sz = grid_index(2) - 1 ; sz >= 0; sz-- )
    {
      grid_index_scan(2) = sz;
      if( occupancy_map -> isOverFloorInCells(grid_index_scan, offground_threshold) )
      {
        point = occupancy_map -> getGridCubeCenter( grid_index_scan );
        //ROS_WARN("This point is unreachable for ground vehicle, corrected.");
        return ;
      }
    }
    ROS_ERROR("There is a hole in the floor! Relocate please!");
}

void PCSmapManager::deepDropPointToPC(Vector3d &point)
{
    deepDropPoint(point);
    // point(2) += 0.5 * occupancy_map -> grid_resolution;
    pcl::PointXYZ  s_point;
    s_point.x    = point(0);
    s_point.y    = point(1);
    s_point.z    = point(2);

    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;

    Eigen::Vector2d t1,t2;

    t1(0) = point(0);
    t1(1) = point(1);

    double min_xydis = 1e9;
    int ind = 0;
    if ( global_kdtree.radiusSearch (s_point, 2*(occupancy_map -> grid_resolution), pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
    {
        for (std::size_t i = 0; i < pointIdxRadiusSearch.size(); ++i)
        {
            t2(0) = global_pointcloud->points[ pointIdxRadiusSearch[i] ].x ;
            t2(1) = global_pointcloud->points[ pointIdxRadiusSearch[i] ].y ;

            if( (t2-t1).norm() < min_xydis){
                min_xydis = (t2-t1).norm();
                ind       = i;
            }
        }   

        point(2) = global_pointcloud->points[ pointIdxRadiusSearch[ind] ].z;
    }

}
