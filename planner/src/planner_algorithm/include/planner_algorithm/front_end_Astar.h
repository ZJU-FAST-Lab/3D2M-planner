#ifndef ASTAR_H
#define ASTAR_H

#include <Eigen/Eigen>
#include "map_manager/PCSmap_manager.h"
using namespace Eigen;
using namespace std;

class GridNode
{
    
    public:
        GridNode(){
            coord = Vector3d(0,0,0);
            index = Vector3i(0,0,0);
            father = NULL;
            gScore = 0;
            fScore = 0;
            id     = 0;
        }
        GridNode(Vector3i ind, Vector3d cor){
            coord = cor;
            index = ind;
            father = NULL;
            gScore = 0;
            fScore = 0;
            id     = 0;
        }
        ~GridNode(){}

        void reset()
        {
            father = NULL;
            gScore = 0;
            fScore = 0;
            id     = 0;
        }
    
        Vector3d coord;
        Vector3i index;
        double gScore;
        double fScore;
        int id;
        GridNode* father;

};


class AstarPathSearcher
{
    
    public:
        GridNode ****GridNodeMap;
        GridNode *terminatePtr;
        int GLX_SIZE;  
        int GLY_SIZE; 
        int GLZ_SIZE;  
        int GLYZ_SIZE; 
        int GLXYZ_SIZE;
        uint8_t * data;
    
    public:
        void reset();
        void init(ros::NodeHandle& nh);
        void initGridMap(PCSmapManager::Ptr env);
        inline void AstarGetSucc(GridNode* currentPtr, vector<GridNode*> & neighborPtrSets, vector<double> & edgeCostSets);
        void AstarPathSearch(Vector3d start, Vector3d end);
        double getHeu(GridNode* node1, GridNode* node2);
        double getCustomCost(GridNode* node_neighbor, GridNode* node_current);
        bool success_flag;
        vector<Vector3d> getPath();

    private:
        Vector3i goalIdx;
        Vector3d start_pt;
        Vector3d end_pt;

        // params

        std::multimap<double, GridNode*> openSet;

    private:
        PCSmapManager::Ptr environment;
    
    public:
         typedef shared_ptr<AstarPathSearcher> Ptr;
};

#endif