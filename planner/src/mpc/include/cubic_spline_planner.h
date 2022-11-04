#pragma once
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/LU>
#include <vector>
#include <cmath>
#include <numeric>

using namespace std;
using namespace Eigen;

class cubic_spline_planner
{
private:
	vector<double> x;
	vector<double> y;
	vector<double> t;
	vector<VectorXd> coeff;
	double ds;
public:
	cubic_spline_planner(const vector<double>& _x, const vector<double>& _y, const double& _ds);
	vector<VectorXd> get_coeff(void);
	Vector3d get_state(double time);
	vector<Vector3d> get_path(void);
	double get_duration(void)
	{
		return t.back();
	}
	~cubic_spline_planner() {}
};

