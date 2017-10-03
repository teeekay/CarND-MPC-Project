#include "MPC.h"
#include "transform_coords.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"
#include <limits>

using CppAD::AD;

// TODO: Set the timestep length and duration
size_t N = 14;// 9; //8;// 16;
double dt = 0.05;//0.1;// 0.05;

const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  size_t t;//timestep counter
  int i;//counter

  //helper function to return square of a number
  double squared(double in) { return(pow(in,2)); }
  CppAD::AD<double> squared(CppAD::AD<double> in) { return(CppAD::pow(in,2)); }

  double ref_v = 60.0 * 1609 / 3600; // reference velocity to try to be close to;
  double ref_epsi = 0.0;
  double ref_cte = 0.0;
  double ref_delta = 0.0; // keep steering close to straight as possible
  double ref_a = 0.0; //keep accel/deccel close to 0 as possible
  double ref_delta_a = 0.0;
  double ref_delta_delta = 0.0;


  //bias against cte
  double cte_factor = 250.0;//500.0
  //bias against steer error
  double epsi_factor = 10000.0;
  //bias against changing velocity from a good speed
  double v_factor = 5.0;
  //bias against large steer in cost funcs
  double steer_factor = 5000.0; //set these up, need way to manipulate from outside
  //bias against large accel/deccel in cost funcs
  double accel_factor = 5.0; //set these up, need way to manipulate from outside
  //bias against jerky steer in cost funcs
  double delta_factor = 5000.0; //set these up, need way to manipulate from outside
  //bias against large accel/deccel in cost funcs
  double delta_a_factor = 5.0; //set these up, need way to manipulate from outside
  double delta_cte_factor = 10000;// 5000.0;//

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  // `fg` is a vector containing the cost and constraints.
  // `vars` is a vector containing the variable values (state & actuators).
  void operator()(ADvector& fg, const ADvector& vars ) {
    fg[0] = 0;

    // Reference State Cost
    //
    for (t = 0; t < N; t++)
    {
      fg[0] += cte_factor * squared(vars[cte_start + t] - ref_cte); //minimize cross track error
      fg[0] += epsi_factor * squared(vars[epsi_start + t] - ref_epsi); //minimize direction error
      fg[0] += v_factor * squared(vars[v_start + t] - ref_v); //add this in to make sure we don't stop during un
    }

    // Minimize the use of actuators.
    for (t = 0; t < N - 1; t++) {
      fg[0] += steer_factor * squared(vars[delta_start + t] - ref_delta);  //minimize steering angles
      fg[0] += accel_factor * squared(vars[a_start + t] - ref_a);  //minimize use of accelerator/brake
    }

    // Minimize the value gap between sequential actuations.
    for (t = 0; t < N - 2; t++) {
      fg[0] += delta_factor * squared((vars[delta_start + t + 1] - vars[delta_start + t]) - ref_delta_delta); //remove jerkiness in steering
      fg[0] += delta_a_factor * squared((vars[a_start + t + 1] - vars[a_start + t]) - ref_delta_a) ; //remove jerkiness in acceleration/braking
      fg[0] += delta_cte_factor * squared((vars[cte_start + t + 1] - vars[cte_start + t]) - ref_cte); //minimize change in cte (similar to steer reduction?
    }

    //
    // Setup Constraints
    //
    // NOTE: In this section you'll setup the model constraints.

    // Initial constraints
    //
    // We add 1 to each of the starting indices due to cost being located at
    // index 0 of `fg`.
    // This bumps up the position of all the other values.
    fg[x_start + 1] = vars[x_start];
    fg[y_start + 1] = vars[y_start];
    fg[psi_start + 1] = vars[psi_start];
    fg[v_start + 1] = vars[v_start];
    fg[cte_start + 1] = vars[cte_start];
    fg[epsi_start + 1] = vars[epsi_start];

    // the rest of the constraints
    for (t = 1; t < N; t++)
    {
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y1 = vars[y_start + t];
      AD<double> y0 = vars[y_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> cte0 = vars[cte_start + t - 1];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> a0 = vars[a_start + t - 1];
      AD<double> delta0 = vars[delta_start + t - 1];

//     if (t > 1)
//     {   // use previous actuations (to account for latency)
//       a = vars[a_start + t - 2];
//       delta = vars[delta_start + t - 2];
//     }
      
      //set up to be flexible based on order of equation
      
      AD<double> f0 = 0.0;
      for (i = 0; i < coeffs.size( ); i++)
      {
        f0 += coeffs[i] * CppAD::pow(x0,i);
      }

      AD<double> psides0 = 0.0;
      for (i = 1; i < coeffs.size( ); i++)
      {
        psides0 += i * coeffs[i] * CppAD::pow(x0, (i-1)); 
      }
      psides0 += CppAD::atan(psides0);

      fg[x_start + t + 1] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[y_start + t + 1] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[psi_start + t + 1] = psi1 - (psi0 - v0 / Lf * delta0 * dt); //adapt with minus sign due to backwards psi in simulator//?
      fg[v_start + t + 1] = v1 - (v0 + a0 * dt);
      fg[cte_start + t + 1] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[epsi_start + t + 1] = epsi1 - ((psi0 - psides0) - v0 / Lf * delta0 * dt);//adapt with minus sign due to backwards psi in simulator
    }
  }
};



//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  size_t i;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // TODO: Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9

  // TMK 20170924 need to get actuators (2) in programatically
  size_t n_vars = (state.size() * N) + ( 2 * (N - 1));

  // TODO: Set the number of constraints
  size_t n_constraints = N * state.size();

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (i = 0; i < n_vars; i++) {
    vars[i] = 0;
  }

 // // Set the initial variable values
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;
  
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // TODO: Set lower and upper limits for variables.
  for (i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -std::numeric_limits<double>::max();// -1.0e19;
    vars_upperbound[i] = std::numeric_limits<double>::max();// 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians)., but try to use -15,15 to evaluate
  for (i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -(25. / 180.) * M_PI; // 0.436332;
    vars_upperbound[i] = (25. / 180.) * M_PI; // 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // try using -0.5 , 0.5
  for (i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;// -1.0;
    vars_upperbound[i] = 0.95;// 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";//0.5

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  //std::cout << "Cost " << cost << std::endl;

  // TODO: Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  //
  // {...} is shorthand for creating a vector, so auto x1 = {1.0,2.0}
  // creates a 2 element double vector.
  vector<double> results;
  results.push_back(solution.x[delta_start]);
  results.push_back(solution.x[a_start]);
  for(i = 1; i < N; i++)
  {
    results.push_back(solution.x[x_start + i]);
    results.push_back(solution.x[y_start + i]);
  }
  return(results);
}
