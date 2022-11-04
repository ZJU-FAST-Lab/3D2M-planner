#include "planner_algorithm/front_end_Astar.h"


void AstarPathSearcher::init(ros::NodeHandle& nh)
{

}

void AstarPathSearcher::initGridMap(PCSmapManager::Ptr env)
{   
    environment = env;
    GLX_SIZE    = (environment -> occupancy_map) -> X_size;
    GLY_SIZE    = (environment -> occupancy_map) -> Y_size;
    GLZ_SIZE    = (environment -> occupancy_map) -> Z_size;
    GLYZ_SIZE   = GLY_SIZE * GLZ_SIZE;
    GLXYZ_SIZE  = GLX_SIZE * GLYZ_SIZE;
  
    data = new uint8_t[GLXYZ_SIZE];
    memset(data, 0, GLXYZ_SIZE * sizeof(uint8_t));
    
    GridNodeMap = new GridNode *** [GLX_SIZE];
    for(int i = 0; i < GLX_SIZE; i++){
        GridNodeMap[i] = new GridNode ** [GLY_SIZE];
        for(int j = 0; j < GLY_SIZE; j++){
            GridNodeMap[i][j] = new GridNode* [GLZ_SIZE];
            for( int k = 0; k < GLZ_SIZE; k++){
                Vector3i tmpIdx(i,j,k);
                Vector3d pos = (environment -> occupancy_map) -> getGridCubeCenter(tmpIdx);
                GridNodeMap[i][j][k] = new GridNode(tmpIdx, pos);
            }
        }
    }

}

void AstarPathSearcher::reset()
{
    for(int i = 0; i < GLX_SIZE; i++){
        for(int j = 0; j < GLY_SIZE; j++){
            for( int k = 0; k < GLZ_SIZE; k++){
                GridNodeMap[i][j][k] -> reset();
            }
        }
    }
}

double AstarPathSearcher::getHeu(GridNode* node1, GridNode* node2)
{

    double cost;
    double p = 1.0/1000;

    //Manhattan
    //return (node1->index-node2->index).lpNorm<1>();

    //Euclidean
    //return (node1->index-node2->index).norm();

    //Diagonal
    Eigen::Vector3i d = node1->index-node2->index;
    int dx = abs(d(0)), dy = abs(d(1)), dz = abs(d(2));
    int dmin = min(dx,min(dy,dz));
    int dmax = max(dx,max(dy,dz));
    int dmid =  dx+dy+dz-dmin-dmax;
    double h = sqrt(3)*dmin + sqrt(2)*(dmid-dmin) + (dmax-dmid);

    cost = h*(1+p);

    
    return cost;
    

}


// in this function, add customized costs
double AstarPathSearcher::getCustomCost(GridNode* node_neighbor, GridNode* node_current)
{
    double cost = 0.0;
    
    //double drop = (environment -> occupancy_map) -> getHeightOfGround( node_neighbor -> coord ) -  (environment -> occupancy_map) -> getHeightOfGround( node_current -> coord );
    //drop = abs(drop);
    //cost += 1000 * drop * drop;
    // cost += 1000 * (environment -> travelcost_map) -> getSDFValue( node_current -> coord );
    // cost += (-1000)* (environment -> occupancy_map)  -> getGridSDFValue( node_current -> index );
    cost += 10000 * (environment -> travelcost_map) -> getGridSDFValue( node_current -> index );
    cost += 100000 * node_current -> coord(2);

    return cost;
}




inline void AstarPathSearcher::AstarGetSucc(GridNode* currentPtr, vector<GridNode*> & neighborPtrSets, vector<double> & edgeCostSets)
{   
    
    neighborPtrSets.clear();
    edgeCostSets.clear();

    for (int i=-1;i<2;i++)
    {
        for (int j=-1;j<2;j++)
        {
            for (int k=-1;k<2;k++)
            {   
                Eigen::Vector3i vi(i,j,k);
                vi = vi + currentPtr -> index;
                if( (environment -> occupancy_map) -> isIndexCanBeNeighbor(vi) )
                {
                    GridNode* p = GridNodeMap[vi(0)][vi(1)][vi(2)];
                    neighborPtrSets.push_back(p);
                    edgeCostSets.push_back(sqrt(i*i+j*j+k*k));
                }
            }
        }
    }
    
}

void AstarPathSearcher::AstarPathSearch(Vector3d start, Vector3d end)
{
    
    
    ros::Time time_1 = ros::Time::now();    
    if( !(environment -> occupancy_map) -> isInMap(start) || !(environment -> occupancy_map) -> isInMap(end) )
    {
       ROS_ERROR("[A*] start or target position is out of map.");
       success_flag = false;
       return ;
    }
    //index of start_point and end_point
    Vector3i start_idx = (environment -> occupancy_map) -> getGridIndex(start);
    Vector3i end_idx   = (environment -> occupancy_map) -> getGridIndex(end);

    goalIdx = end_idx;

    //position of start_point and end_point
    start_pt = (environment -> occupancy_map) -> getGridCubeCenter(start_idx);
    end_pt   = (environment -> occupancy_map) -> getGridCubeCenter(end_idx);

    //Initialize the pointers of struct GridNode which represent start node and goal node
    GridNode* startPtr = new GridNode(start_idx, start_pt);
    GridNode* endPtr   = new GridNode(end_idx,   end_pt);

    //openSet is the open_list implemented through multimap in STL library
    openSet.clear();
    // currentPtr represents the node with lowest f(n) in the open_list
    GridNode* currentPtr  = NULL;
    GridNode* neighborPtr = NULL;


    //put start node in open set
    startPtr -> gScore = 0;
    startPtr -> fScore = getHeu(startPtr,endPtr);   
    startPtr -> id = 1; 
    startPtr -> coord = start_pt;

    openSet.insert( make_pair(startPtr -> fScore, startPtr) );
 
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> id = startPtr -> id; 
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> gScore = startPtr -> gScore;
    GridNodeMap[start_idx(0)][start_idx(1)][start_idx(2)] -> fScore = startPtr -> fScore;

    vector<GridNode*> neighborPtrSets;
    vector<double> edgeCostSets;


    // this is the main loop
    while ( !openSet.empty() ){

        auto iter  = std::begin(openSet);
        currentPtr = iter -> second;
        openSet.erase(iter);
        currentPtr -> id = -1;

        // if the current node is the goal 
        if( currentPtr -> index == goalIdx ){
            ros::Time time_2 = ros::Time::now();
            terminatePtr = currentPtr;
            success_flag = true;
            ROS_WARN("[A*]{sucess}  Time in A*  is %f ms, path cost if %f m", (time_2 - time_1).toSec() * 1000.0, currentPtr -> gScore * (environment->occupancy_map->grid_resolution) );            
            return;
        }
        //get the succetion
        AstarGetSucc(currentPtr, neighborPtrSets, edgeCostSets);  //STEP 4: finish AstarPathFinder::AstarGetSucc yourself     

        for(int i = 0; i < (int)neighborPtrSets.size(); i++){

            neighborPtr = neighborPtrSets[i];
            double ec = edgeCostSets[i];
            if(neighborPtr -> id == 0){

                double tg = ec + currentPtr -> gScore; 
                
                neighborPtr -> father = currentPtr;
                neighborPtr -> gScore = tg;
                neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);

                neighborPtr -> id = 1;
                openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                
                continue;
            }
        
            else if(neighborPtr -> id == 1){ 
                double tg = ec + currentPtr->gScore;
                if (tg < neighborPtr->gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);;
                }
                continue;
            }
        
            else{

                double tg = ec + currentPtr->gScore;
                if(tg < neighborPtr -> gScore)
                {
                    neighborPtr -> father = currentPtr;
                    neighborPtr -> gScore = tg;
                    neighborPtr -> fScore = tg + getHeu(neighborPtr, endPtr) + getCustomCost(neighborPtr, currentPtr);;
                     
                    neighborPtr -> id = 1;
                    openSet.insert( make_pair(neighborPtr -> fScore, neighborPtr) );
                }
                continue;
            }
        }      
    }
    //if search fails
    success_flag = false;
    ros::Time time_2 = ros::Time::now();
    if((time_2 - time_1).toSec() > 0.1)
        ROS_WARN("Time consume in Astar path finding is %f", (time_2 - time_1).toSec() );
    
}

vector<Vector3d> AstarPathSearcher::getPath() 
{   
    vector<Vector3d> path;
    vector<GridNode*> gridPath;
    GridNode* p = terminatePtr;
    while (p -> father != NULL)
    {
        gridPath.push_back(p);
        p = p -> father;
    }
    gridPath.push_back(p);

    for (auto ptr: gridPath)
        path.push_back(ptr -> coord);
    
    reverse(path.begin(),path.end());
    return path;
}