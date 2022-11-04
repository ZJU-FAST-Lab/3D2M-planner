#include "cubic_spline_planner.h"
#include <iostream>

cubic_spline_planner::cubic_spline_planner(const vector<double>& _x, const vector<double>& _y, const double& _ds) \
	:x(_x), y(_y), ds(_ds), t{0} \
	{ 
		coeff = get_coeff(); 
	};

vector<VectorXd> cubic_spline_planner::get_coeff(void)
{
	vector<double> h;
	int n = x.size();

	// get time allocation
	double dx, dy;
	for (int i = 0; i < n - 1; i++)
	{
		dx = x[i + 1] - x[i];
		dy = y[i + 1] - y[i];
		double dist = sqrt(dx*dx + dy * dy);
		h.push_back(dist);
		//cout << "ti+1:" << t[i + 1] << endl;
		t.push_back(t[i] + dist);
	}

	// get A
	MatrixXd A = MatrixXd::Zero(n, n);
	A.row(0)[0] = 1.0;
	for (int i = 0; i < n - 2; i++)
	{
		A.row(i + 1)[i + 1] = 2.0*(h[i] + h[i + 1]);
		A.row(i + 1)[i] = h[i];
		A.row(i)[i + 1] = h[i];
	}
	A.row(0)[1] = 0.0;
	A.row(n - 1)[n - 2] = 0.0;
	A.row(n - 2)[n - 1] = h[n - 2];
	A.row(n - 1)[n - 1] = 1.0;
	//cout << "A:"<<A << endl;
	// get b
	VectorXd bx = VectorXd::Zero(n);
	VectorXd by = VectorXd::Zero(n);
	for (int i = 0; i < n - 2; i++)
	{
		bx[i + 1] = 3.0*(x[i + 2] - x[i + 1]) / h[i + 1]\
			- 3.0*(x[i + 1] - x[i]) / h[i];
		by[i + 1] = 3.0*(y[i + 2] - y[i + 1]) / h[i + 1]\
			- 3.0*(y[i + 1] - y[i]) / h[i];
	}
	
	//LU-decomposition and solve Ax=b
	VectorXd Cx = A.lu().solve(bx);
	VectorXd Cy = A.lu().solve(by);
	
	// get B and D, then return
	double tbx, tby, tdx, tdy;
	for (int i = 0; i < n - 1; i++)
	{
		tdx = (Cx[i + 1] - Cx[i]) / 3.0 / h[i];
		tdy = (Cy[i + 1] - Cy[i]) / 3.0 / h[i];
		tbx = (x[i + 1] - x[i]) / h[i] - h[i] * \
			(Cx[i + 1] + 2.0*Cx[i]) / 3.0;
		tby = (y[i + 1] - y[i]) / h[i] - h[i] * \
			(Cy[i + 1] + 2.0*Cy[i]) / 3.0;
		
		VectorXd temp(8);
		temp << x[i], tbx, Cx[i], tdx, y[i], tby, Cy[i], tdy;
		coeff.push_back(temp);
		//cout << tdx << " ";
	}
	return coeff;
}

Vector3d cubic_spline_planner::get_state(double time)
{
	// traverse search
	Vector3d state;
	size_t i;

	for (i = 0; i < t.size() - 1; i++)
	{
		if (t[i + 1] > time)
		{
			break;
		}
	}
	if (i==t.size()-1)
	{
		i--;
	}

	double dt = time - t[i];

	// position
	state[0] = coeff[i][0] + coeff[i][1] * dt + \
		coeff[i][2] * dt *dt + coeff[i][3] * dt*dt*dt;
	state[1] = coeff[i][4] + coeff[i][5] * dt + \
		coeff[i][6] * dt *dt + coeff[i][7] * dt*dt*dt;

	// yaw
	double dx = coeff[i][1] + 2.0 * coeff[i][2] * dt + 3.0 * coeff[i][3] * dt * dt;
	double dy = coeff[i][5] + 2.0 * coeff[i][6] * dt + 3.0 * coeff[i][7] * dt * dt;
	state[2] = atan2(dy, dx);

	return state;
}

vector<Vector3d> cubic_spline_planner::get_path(void)
{
	vector<Vector3d> path;
	for (double i = 0; i <= *(t.end() - 1); i += ds)
	{
		path.push_back(get_state(i));
	}
	path.push_back(get_state(t.back()));

	return path;
}