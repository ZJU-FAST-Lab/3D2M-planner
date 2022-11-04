#include "map_manager/PCSmap_manager.h"

void GridMap::clearGridMap()
{
    for (int i = 0;i < X_size; i++)
	{
		for (int j = 0; j < Y_size; j++)
		{
			for (int k = 0; k < Z_size; k++)
			{
				grid_map[i][j][k]            = 0;
				grid_map_buffer_neg[i][j][k] = 0;
				grid_map_buffer_all[i][j][k] = 0;
                grid_map_flags[i][j][k]      = 0;
			}
		}
	}
}

void GridMap::releaseMemory()
{
    for (int x = 0; x < X_size; x++)
	{
		for (int y = 0; y < Y_size; y++)
		{
			delete [] grid_map[x][y];
		}
	}
	for (int x = 0; x < X_size; x++)
	{
		delete [] grid_map[x];
	}
	delete [] grid_map;
}


void GridMap::createGridMap(Vector3d boundary_xyzmin, Vector3d boundary_xyzmax)
{
    this->boundary_xyzmin = boundary_xyzmin;
    this->boundary_xyzmax = boundary_xyzmax;
    X_size = ceil( ( boundary_xyzmax(0) - boundary_xyzmin(0) ) / grid_resolution );
    Y_size = ceil( ( boundary_xyzmax(1) - boundary_xyzmin(1) ) / grid_resolution );
    Z_size = ceil( ( boundary_xyzmax(2) - boundary_xyzmin(2) ) / grid_resolution );

    // create 3D grid map
    grid_map             = new double **[X_size];
    grid_map_buffer_neg  = new double **[X_size];
    grid_map_buffer_all  = new double **[X_size];
    grid_map_flags       = new bool**[X_size];
	for(int x = 0; x < X_size; x++)
	{
		grid_map[x]              = new double *[Y_size];
        grid_map_buffer_neg[x]   = new double *[Y_size];
        grid_map_buffer_all[x]   = new double *[Y_size];
        grid_map_flags[x]        = new bool *[Y_size];
		for (int y = 0; y < Y_size; y++)
		{
			grid_map[x][y]              = new double [Z_size];
			grid_map_buffer_neg[x][y]   = new double [Z_size];
			grid_map_buffer_all[x][y]   = new double [Z_size];
            grid_map_flags[x][y]        = new bool [Z_size];
		}
	}
    clearGridMap();
}


bool GridMap::isInMap(Vector3d pos_w)
{
    if( pos_w(0) < boundary_xyzmin(0) ) {return false;}
    if( pos_w(1) < boundary_xyzmin(1) ) {return false;}
    if( pos_w(2) < boundary_xyzmin(2) ) {return false;}
    
    if( pos_w(0) > boundary_xyzmax(0) ) {return false;}
    if( pos_w(1) > boundary_xyzmax(1) ) {return false;}
    if( pos_w(2) > boundary_xyzmax(2) ) {return false;}
    return true;
}

bool GridMap::isIndexValid(Vector3i index)
{
    if(index(0) < 0)       {return false;}
    if(index(0) >= X_size) {return false;}

    if(index(1) < 0)       {return false;}
    if(index(1) >= Y_size) {return false;}

    if(index(2) < 0)       {return false;}
    if(index(2) >= Z_size) {return false;}

    return true;
}


bool GridMap::isIndexValid(int ix,int iy, int iz)
{
    if(ix < 0)       {return false;}
    if(ix >= X_size) {return false;}

    if(iy < 0)       {return false;}
    if(iy >= Y_size) {return false;}

    if(iz < 0)       {return false;}
    if(iz >= Z_size) {return false;}

    return true;
}


Vector3i GridMap::getGridIndex(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            cout<<pos_w<<endl;
            ROS_WARN("[getGridIndex()] this point is out of map!");
            cout << (pos_w(0) <= boundary_xyzmin(0)) <<"\t"<<(pos_w(1) <= boundary_xyzmin(1))<<"\t"
             << (pos_w(2) <= boundary_xyzmin(2)) <<"\t"<<(pos_w(0) >= boundary_xyzmax(0))<<"\t"
             << (pos_w(1) >= boundary_xyzmax(1)) <<"\t"<<(pos_w(2) >= boundary_xyzmax(2))<<"\n";
            cout<<"boundary is "<< boundary_xyzmax<<endl<<endl;
        }
        return Vector3i(0,0,0);
    }

    double dx = pos_w(0) - boundary_xyzmin(0);
    double dy = pos_w(1) - boundary_xyzmin(1);
    double dz = pos_w(2) - boundary_xyzmin(2);
    int    ix = floor( dx / grid_resolution );
    int    iy = floor( dy / grid_resolution );
    int    iz = floor( dz / grid_resolution );

    return Vector3i(ix, iy, iz);
}


Vector3d GridMap::getGridCubeCenter(int ix, int iy, int iz)
{
    return getGridCubeCenter(Vector3i(ix, iy, iz));
}

Vector3d GridMap::getGridCubeCenter(Vector3i index)
{
    
    if( !isIndexValid(index) ){
        if(debug_output)
        {
            cout<<index<<endl;
            ROS_WARN("[getGridCubeCenter()] this index is out of map!");
        }
        return Vector3d(0,0,0);
    }

    double dx = ( index(0) + 0.5 ) * grid_resolution; 
    double dy = ( index(1) + 0.5 ) * grid_resolution; 
    double dz = ( index(2) + 0.5 ) * grid_resolution; 
    Vector3d dpos_w = Vector3d(dx, dy, dz);

    return dpos_w + boundary_xyzmin;
}


bool GridMap::isIndexCanBeNeighbor(Vector3i index)
{
    if( !isIndexValid(index) ){
        if(debug_output) 
        {
            cout<<index<<endl;
            ROS_WARN("[isIndexOccupied()] this index is out of map!");
        }
        return false;
    }
    return grid_map_flags[index(0)][index(1)][index(2)];
}

bool GridMap::isIndexOccupied(Vector3i index)
{
    if( !isIndexValid(index) ){
        if(debug_output) 
        {
            cout<<index<<endl;
            ROS_WARN("[isIndexOccupied()] this index is out of map!");
        }
        
        return true;
    }
    if( grid_map[index(0)][index(1)][index(2)] == 0) {return false;}
    else {return true;}
}

bool GridMap::isIndexOccupied(int ix, int iy, int iz)
{
    bool invalid = false;
    if(ix < 0)       {invalid = true;}
    if(ix >= X_size) {invalid = true;}
    if(iy < 0)       {invalid = true;}
    if(iy >= Y_size) {invalid = true;}
    if(iz < 0)       {invalid = true;}
    if(iz >= Z_size) {invalid = true;}
    if(invalid)
    {
        if(debug_output) 
        {
            ROS_WARN("[isIndexOccupied()] this index is out of map!");
        }
        return true;
    }
    if( grid_map[ix][iy][iz] == 0) {return false;}
    else {return true;}
}

bool GridMap::isCoordOccupied(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            ROS_WARN("[isCoordOccupied()] this point is out of map!");
        }
        return true;
    }
    return isIndexOccupied(getGridIndex(pos_w));
}


bool GridMap::isOverFloorInCells(Vector3i index, double distance)
{
    int indz_depth = floor( distance / grid_resolution );
    Vector3i ind = index;
    for(int dz = 0; dz <= indz_depth; dz++)
    {
        ind(2) = index(2) - dz;
        if(isIndexOccupied(ind)) {
            return true;
        }
    }

    return false;
}


double GridMap::getHeightToGround(Vector3d pos_w)
{
    double height = 0.0;
    Vector3i index_scan = getGridIndex(pos_w);
    for(int sz = index_scan(2); sz >= 0 ; sz--)
    {
        index_scan(2) = sz;
        if( !isIndexOccupied(index_scan) ) { height += grid_resolution; }
        else {return height;}
    }
    return height;
}


double GridMap::getHeightOfGround(Vector3d pos_w)
{
    double height = 0.0;
    Vector3i index_scan = getGridIndex(pos_w);
    for(int sz = index_scan(2); sz >= 0 ; sz--)
    {
        index_scan(2) = sz;
        if( isIndexOccupied(index_scan) ) { height = grid_resolution * (sz+1); return height;}

    }
    return height;
}

double GridMap::getHeightGap(Vector3d pos_w)
{
    double gap = 0.0;
    int    bz  = 0;
    int    sz  = 0;
    Vector3i index_scan = getGridIndex(pos_w);
    if( isIndexOccupied(index_scan) ) { index_scan(2) = index_scan(2) + 1 ;}
    bz = index_scan(2);
    for(sz = bz; sz < Z_size ; sz++)
    {
        index_scan(2) = sz;
        // std::cout<<" bz-sz = "<<bz <<" - "<<sz<<"  "<<grid_resolution * (sz - bz)<<std::endl;
        if( isIndexOccupied(index_scan) ) {
             gap = grid_resolution * (sz - bz);
             return gap;
        }

    }
    gap = grid_resolution * (sz - bz);
    return gap;
}














bool GridMap::isGridBoundOfObs(Vector3i index)
{
    if( !isIndexOccupied(index) ) {
        cout << "[isGridBoundOfObs] this function got a free index!"<<endl;
        return false;
    }

    int idx,idy,idz;
    Vector3i index_scan;
    for (idx = 0; idx < X_size; idx++)
	{
		for (idy = 0; idy < Y_size; idy++)
		{
			for (idz = 0; idz < Z_size; idz++)
			{
				index_scan = index + Vector3i(idx,idy,idz);
                if(index_scan == index)         {continue;}
                if( !isIndexValid(index_scan) ) {continue;}
                if( !isIndexOccupied(index_scan) ) {return true;}
			}
		}
	}
    return false;
}


void GridMap::clearGridESDF()
{
    for (int i = 0;i < X_size; i++)
	{
		for (int j = 0; j < Y_size; j++)
		{
			for (int k = 0; k < Z_size; k++)
			{
				grid_esdf[i][j][k] = 0;
				grid_esdf_buffer1[i][j][k] = 0;
				grid_esdf_buffer2[i][j][k] = 0;
			}
		}
	}
}

void GridMap::clearGridPureESDF(double num)
{
    for (int i = 0;i < X_size; i++)
	{
		for (int j = 0; j < Y_size; j++)
		{
			for (int k = 0; k < Z_size; k++)
			{
				grid_esdf[i][j][k] = num;
			}
		}
	}
}


int GridMap::getVoxelNum(int dim)
{
    if(dim == 0){return X_size;}
    if(dim == 1){return Y_size;}
    if(dim == 2){return Z_size;}
}

void debugP(double num)
{
    cout<<num<<endl;
}


void GridMap::generateEmptyESDF()
{
    grid_esdf = new double **[X_size];
	for(int x = 0; x < X_size; x++)
	{
		grid_esdf[x] = new double *[Y_size];
		for (int y = 0; y < Y_size; y++)
		{
			grid_esdf[x][y] = new double [Z_size];
		}
	}
    clearGridPureESDF(-Z_size * grid_resolution);
}

void GridMap::generateESDF1dAt(int ix, int iy, int iz)
{
    double num = 0;
    for(int i = iz ; i >= 0 ; i--)
    {
        grid_esdf[ix][iy][i] = max( num, grid_esdf[ix][iy][i] );
        num -= grid_resolution;
        if( isIndexOccupied(ix,iy,i) == true ){
            break;
        }
    }
    num = -1;
    for(int i = iz + 1; i < Z_size; i++)
    {
        grid_esdf[ix][iy][i] = max( num, grid_esdf[ix][iy][i] );
        num -= grid_resolution;
        if( isIndexOccupied(ix,iy,i) == true ){
            break;
        }
    }

}

void GridMap::generateESDF3d()
{
    Vector3i min_esdf(0,0,0);
    Vector3i max_esdf(X_size-1, Y_size-1, Z_size-1);

    grid_esdf = new double **[X_size];
    grid_esdf_buffer1 = new double **[X_size];
    grid_esdf_buffer2 = new double **[X_size];
	for(int x = 0; x < X_size; x++)
	{
		grid_esdf[x] = new double *[Y_size];
		grid_esdf_buffer1[x] = new double *[Y_size];
		grid_esdf_buffer2[x] = new double *[Y_size];
		for (int y = 0; y < Y_size; y++)
		{
			grid_esdf[x][y] = new double [Z_size];
			grid_esdf_buffer1[x][y] = new double [Z_size];
			grid_esdf_buffer2[x][y] = new double [Z_size];
		}
	}

    clearGridESDF();

    /* ========== compute positive DT ========== */

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        fillESDF(
            [&](int z) {
                return isIndexOccupied(x, y, z) == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int z, double val) { grid_esdf_buffer1[x][y][z] = val; }, min_esdf(2),
            max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
        fillESDF([&](int y) { return grid_esdf_buffer1[x][y][z]; },
                [&](int y, double val) { grid_esdf_buffer2[x][y][z] = val; }, min_esdf(1),
                max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
        fillESDF([&](int x) { return grid_esdf_buffer2[x][y][z]; },
                [&](int x, double val) {
                    grid_esdf[x][y][z] =  grid_resolution * std::sqrt(val);
                },
                min_esdf(0), max_esdf(0), 0);
        }
    }

    /* ========== compute negative distance ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

            if (grid_map[x][y][z] == 0) {
            grid_map_buffer_neg[x][y][z] = 1;

            } else if (grid_map[x][y][z] == 1) {
            grid_map_buffer_neg[x][y][z] = 0;
            } else {
            ROS_ERROR("what?");
            }
        }

    ros::Time t1, t2;

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        fillESDF(
            [&](int z) {
                return grid_map_buffer_neg[x][y][z] == 1 ?
                    0 :
                    std::numeric_limits<double>::max();
            },
            [&](int z, double val) { grid_esdf_buffer1[x][y][z] = val; }, min_esdf(2),
            max_esdf(2), 2);
        }
    }

    for (int x = min_esdf(0); x <= max_esdf(0); x++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
        fillESDF([&](int y) { return grid_esdf_buffer1[x][y][z]; },
                [&](int y, double val) { grid_esdf_buffer2[x][y][z] = val; }, min_esdf(1),
                max_esdf(1), 1);
        }
    }

    for (int y = min_esdf(1); y <= max_esdf(1); y++) {
        for (int z = min_esdf(2); z <= max_esdf(2); z++) {
        fillESDF([&](int x) { return grid_esdf_buffer2[x][y][z]; },
                [&](int x, double val) {
                    grid_map_buffer_neg[x][y][z] = grid_resolution * std::sqrt(val);
                },
                min_esdf(0), max_esdf(0), 0);
        }
    }

    /* ========== combine pos and neg DT ========== */
    for (int x = min_esdf(0); x <= max_esdf(0); ++x)
        for (int y = min_esdf(1); y <= max_esdf(1); ++y)
        for (int z = min_esdf(2); z <= max_esdf(2); ++z) {

            grid_map_buffer_all[x][y][z] = grid_map[x][y][z];

            if (grid_map_buffer_neg[x][y][z] > 0.0)
            grid_map_buffer_all[x][y][z] += (-grid_map_buffer_neg[x][y][z] + grid_resolution);
    }
}

template <typename F_get_val, typename F_set_val>
void GridMap::fillESDF(F_get_val f_get_val, F_set_val f_set_val, int start, int end, int dim) {

  int v[getVoxelNum(dim)];
  double z[getVoxelNum(dim) + 1];

  int k = start;
  v[start] = start;
  z[start] = -std::numeric_limits<double>::max();
  z[start + 1] = std::numeric_limits<double>::max();

//Lantern 
  double ky = 1;
  if( dim == 2){ 
    //   ky = 16; 
    ky = 64;
}

  for (int q = start + 1; q <= end; q++) {
    k++;
    double s;

    do {
      k--;
      s = ((f_get_val(q) + ky * q * q) - (f_get_val(v[k]) + ky * v[k] * v[k])) / (2 * ky*q - 2 * ky*v[k]);
    } while (s <= z[k]);

    k++;

    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<double>::max();
  }

  k = start;

  for (int q = start; q <= end; q++) {
    while (z[k + 1] < q) k++;
    double val = ky * (q - v[k]) * (q - v[k]) + f_get_val(v[k]);
    f_set_val(q, val);
  }
}



double GridMap::getGridSDFValue(Vector3i index)
{
    if( !isIndexValid(index) ){return 0;}
    return grid_esdf[index(0)][index(1)][index(2)];
}

double GridMap::getGridSDFValue(int ix, int iy, int iz)
{
    if(ix < 0)       { ix = 0;}
    if(ix >= X_size) { ix = X_size - 1;}
    if(iy < 0)       { iy = 0;}
    if(iy >= Y_size) { iy = Y_size - 1;}
    if(iz < 0)       { iz = 0;}
    if(iz >= Z_size) { iz = Z_size - 1;}

    return grid_esdf[ix][iy][iz];
}

double GridMap::getSDFValue(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            cout<<pos_w<<endl;
            ROS_WARN("[getSDFValue()] this point is out of map!");
        }
        return 0;
    }

    Vector3i index  = getGridIndex( pos_w );
    Vector3d center = getGridCubeCenter( index );
    
    double ix = index(0);
    double iy = index(1);
    double iz = index(2);
    if( pos_w(0) < center(0) ){ ix-- ; }
    if( pos_w(1) < center(1) ){ iy-- ; }
    if( pos_w(2) < center(2) ){ iz-- ; }

    center = getGridCubeCenter( ix,iy,iz );

    double x_l = (pos_w(0) - center(0)) / grid_resolution;
    double x_r = 1.0 - x_l;

    double y_l = (pos_w(1) - center(1)) / grid_resolution;
    double y_r = 1.0 - y_l;

    double z_l = (pos_w(2) - center(2)) / grid_resolution;
    double z_r = 1.0 - z_l;


    double d1 = x_r * getGridSDFValue(ix,iy,iz)   + x_l * getGridSDFValue(ix+1,iy,iz);
    double d2 = x_r * getGridSDFValue(ix,iy+1,iz) + x_l * getGridSDFValue(ix+1,iy+1,iz);


    double d3 = x_r * getGridSDFValue(ix,iy,iz+1)   + x_l * getGridSDFValue(ix+1,iy,iz+1);
    double d4 = x_r * getGridSDFValue(ix,iy+1,iz+1) + x_l * getGridSDFValue(ix+1,iy+1,iz+1);

    double d5 = y_r * d1 + y_l * d2; //down
    double d6 = y_r * d3 + y_l * d4; //up
    double value = z_r * d5 + z_l * d6;

    return value;
}

Vector3d GridMap::getSDFGrad(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            ROS_WARN("[getSDFGrad()] this point is out of map!");
        }
        return Vector3d::Zero();
    }

    Vector3i index  = getGridIndex( pos_w );
    Vector3d center = getGridCubeCenter( index );
    
    double ix = index(0);
    double iy = index(1);
    double iz = index(2);
    if( pos_w(0) < center(0) ){ ix-- ; }
    if( pos_w(1) < center(1) ){ iy-- ; }
    if( pos_w(2) < center(2) ){ iz-- ; }

    center = getGridCubeCenter( ix,iy,iz );

    double x_l = (pos_w(0) - center(0)) / grid_resolution;
    double x_r = 1.0 - x_l;

    double y_l = (pos_w(1) - center(1)) / grid_resolution;
    double y_r = 1.0 - y_l;

    double z_l = (pos_w(2) - center(2)) / grid_resolution;
    double z_r = 1.0 - z_l;


    double g1 = y_r * ( getGridSDFValue(ix+1,iy,iz) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                y_l * ( getGridSDFValue(ix+1,iy+1,iz) - getGridSDFValue(ix,iy+1,iz) ) / grid_resolution;
    double g2 = y_r * ( getGridSDFValue(ix+1,iy,iz+1) - getGridSDFValue(ix,iy,iz+1) ) / grid_resolution + 
                y_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix,iy+1,iz+1) ) / grid_resolution;
    double gx = z_r * g1 + z_l * g2;


    double g3 = x_r * ( getGridSDFValue(ix,iy+1,iz) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz) - getGridSDFValue(ix+1,iy,iz) ) / grid_resolution;
    double g4 = x_r * ( getGridSDFValue(ix,iy+1,iz+1) - getGridSDFValue(ix,iy,iz+1) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix+1,iy,iz+1) ) / grid_resolution;
    double gy = z_r * g3 + z_l * g4;


    double g5 = x_r * ( getGridSDFValue(ix,iy,iz+1) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy,iz+1) - getGridSDFValue(ix+1,iy,iz) ) / grid_resolution;
    double g6 = x_r * ( getGridSDFValue(ix,iy+1,iz+1) - getGridSDFValue(ix,iy+1,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix+1,iy+1,iz) ) / grid_resolution;
    double gz = y_r * g5 + y_l * g6;

    return -Vector3d(gx,gy,gz);
}


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////


void TravelGridMap::clearGridMap()
{
    for (int i = 0;i < X_size; i++)
	{
		for (int j = 0; j < Y_size; j++)
		{
			for (int k = 0; k < Z_size; k++)
			{
				travel_grid[i][j][k]->plane_normal  = Vector3d(0,0,0);
				travel_grid[i][j][k]->mass_center   = Vector3d(0,0,0);
				travel_grid[i][j][k]->explored      = false;
			}
		}
	}
}

void TravelGridMap::releaseMemory()
{
    GridMap::releaseMemory();
    for (int x = 0; x < X_size; x++)
	{
		for (int y = 0; y < Y_size; y++)
		{
			delete [] travel_grid[x][y];
		}
	}
	for (int x = 0; x < X_size; x++)
	{
		delete [] travel_grid[x];
	}
	delete [] travel_grid;
}

void TravelGridMap::createGridMap(Vector3d boundary_xyzmin, Vector3d boundary_xyzmax)
{
    GridMap::createGridMap(boundary_xyzmin, boundary_xyzmax);
    // create 3D grid map
    travel_grid = new TravelGrid::Ptr **[X_size];
	for(int x = 0; x < X_size; x++)
	{
		travel_grid[x] = new TravelGrid::Ptr *[Y_size];
		for (int y = 0; y < Y_size; y++)
		{
			travel_grid[x][y] = new TravelGrid::Ptr [Z_size];
            for (int z = 0; z < Z_size; z++)
		    {
			    travel_grid[x][y][z].reset(new TravelGrid);
		    }
		}
	}
    clearGridMap();
}

void TravelGridMap::copyTravelGrid( Vector3i src_index, Vector3i tgt_index)
{
    travel_grid[tgt_index(0)][tgt_index(1)][tgt_index(2)]->explored      = travel_grid[src_index(0)][src_index(1)][src_index(2)]->explored;
    travel_grid[tgt_index(0)][tgt_index(1)][tgt_index(2)]->plane_normal  = travel_grid[src_index(0)][src_index(1)][src_index(2)]->plane_normal;
    travel_grid[tgt_index(0)][tgt_index(1)][tgt_index(2)]->mass_center   = travel_grid[src_index(0)][src_index(1)][src_index(2)]->mass_center;
    travel_grid[tgt_index(0)][tgt_index(1)][tgt_index(2)]->roughness     = travel_grid[src_index(0)][src_index(1)][src_index(2)]->roughness;
}

bool TravelGridMap::isTravelGridExplored(Vector3i index)
{
    if( !isIndexValid(index) ){
        if(debug_output) 
        {
            cout<<index<<endl;
            ROS_WARN("[isIndexOccupied()] this index is out of map!");
        }
        
        return true;
    }
    if( travel_grid[index(0)][index(1)][index(2)]->explored == 0) {return false;}
    else {return true;}
}


double TravelGridMap::getTravelCost(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            ROS_WARN("[getTravelCost()] this point is out of map!");
        }
        return 0;
    }
    Vector3i index = getGridIndex(pos_w);
    return grid_map[index(0)][index(1)][index(2)];
    // return grid_esdf[index(0)][index(1)][index(2)];
}


Vector3d TravelGridMap::getTravelPN(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            ROS_WARN("[getTravelPN()] this point is out of map!");
        }
        return Vector3d(0,0,0);
    }

    Vector3i index = getGridIndex(pos_w);
    return travel_grid[index(0)][index(1)][index(2)] -> plane_normal;
}

Vector3d TravelGridMap::getSDFGrad(Vector3d pos_w)
{
    if( !isInMap(pos_w) )
    {   
        if(debug_output)
        {
            ROS_WARN("[getSDFGrad()] this point is out of map!");
        }
        return Vector3d::Zero();
    }

    Vector3i index  = getGridIndex( pos_w );
    Vector3d center = getGridCubeCenter( index );
    
    double ix = index(0);
    double iy = index(1);
    double iz = index(2);
    if( pos_w(0) < center(0) ){ ix-- ; }
    if( pos_w(1) < center(1) ){ iy-- ; }
    if( pos_w(2) < center(2) ){ iz-- ; }

    center = getGridCubeCenter( ix,iy,iz );

    double x_l = (pos_w(0) - center(0)) / grid_resolution;
    double x_r = 1.0 - x_l;

    double y_l = (pos_w(1) - center(1)) / grid_resolution;
    double y_r = 1.0 - y_l;

    double z_l = (pos_w(2) - center(2)) / grid_resolution;
    double z_r = 1.0 - z_l;


    double g1 = y_r * ( getGridSDFValue(ix+1,iy,iz) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                y_l * ( getGridSDFValue(ix+1,iy+1,iz) - getGridSDFValue(ix,iy+1,iz) ) / grid_resolution;
    double g2 = y_r * ( getGridSDFValue(ix+1,iy,iz+1) - getGridSDFValue(ix,iy,iz+1) ) / grid_resolution + 
                y_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix,iy+1,iz+1) ) / grid_resolution;
    double gx = z_r * g1 + z_l * g2;


    double g3 = x_r * ( getGridSDFValue(ix,iy+1,iz) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz) - getGridSDFValue(ix+1,iy,iz) ) / grid_resolution;
    double g4 = x_r * ( getGridSDFValue(ix,iy+1,iz+1) - getGridSDFValue(ix,iy,iz+1) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix+1,iy,iz+1) ) / grid_resolution;
    double gy = z_r * g3 + z_l * g4;


    double g5 = x_r * ( getGridSDFValue(ix,iy,iz+1) - getGridSDFValue(ix,iy,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy,iz+1) - getGridSDFValue(ix+1,iy,iz) ) / grid_resolution;
    double g6 = x_r * ( getGridSDFValue(ix,iy+1,iz+1) - getGridSDFValue(ix,iy+1,iz) ) / grid_resolution + 
                x_l * ( getGridSDFValue(ix+1,iy+1,iz+1) - getGridSDFValue(ix+1,iy+1,iz) ) / grid_resolution;
    double gz = y_r * g5 + y_l * g6;
    //double gz = 0;
    //cout<<"gz = "<<gz<<endl;
    return -Vector3d(gx,gy,gz);

}


void TravelGridMap::clearGridESDF()
{
    for (int i = 0;i < X_size; i++)
	{
		for (int j = 0; j < Y_size; j++)
		{
			for (int k = 0; k < Z_size; k++)
			{
                grid_esdf[i][j][k] = 0;
			}
		}
	}
}

void TravelGridMap::generateESDF()
{
    truncation_index = ceil( truncation_dis / grid_resolution );
    // create 3D grid esdf
    grid_esdf = new double **[X_size];
	for(int x = 0; x < X_size; x++)
	{
		grid_esdf[x] = new double *[Y_size];
		for (int y = 0; y < Y_size; y++)
		{
			grid_esdf[x][y] = new double [Z_size];
		}
	}
    clearGridESDF();

    // generateESDF
    double turcation = truncation_dis;
    // turcation = 3.0;
    std::cout<<"turcation_dis = "<<turcation<<std::endl;
    double distance;
    int turcation_id = ceil( turcation/grid_resolution );
    int sx,sy, ix,iy;
    Vector3i current_index;
    vector<Vector2i> occ_ids;
    for (int z = 0; z < Z_size; z++)
    {
        cout<<"TRAVEL ESDF: " << (double)z*100/Z_size <<" %"<<endl;
        occ_ids.clear();
	    for (int y = 1; y < Y_size - 1; y++)
		{
			for (int x = 1; x < X_size - 1; x++)
			{   
                if( grid_map[x][y][z] > 2 ) { occ_ids.push_back(Vector2i(x,y)); grid_esdf[x][y][z] = grid_map[x][y][z];}
			}
		}

        for(Vector2i occ_index : occ_ids)
        {
            sx = occ_index(0);
            sy = occ_index(1);
            for(int dx = -turcation_id; dx <= turcation_id; dx++)
            {
                for(int dy = -turcation_id; dy <= turcation_id; dy++)
                {
                    ix = sx + dx;
                    iy = sy + dy;
                    if( !(dx==0 && dy==0) && isIndexValid(ix,iy,z) ) {
                        distance = grid_resolution * sqrt(dx*dx + dy*dy);
                        grid_esdf[ix][iy][z] = max( grid_esdf[ix][iy][z] , grid_esdf[sx][sy][z] * (turcation - distance)/ turcation);
                    }  
                }   
            }
        }
	}

   cout << "[TravelGridMap::generateESDF()] done!"<<endl;

}

