#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "Eigen-3.3/Eigen/Dense"
#include <cppad/cppad.hpp>
#include "MPC.h"
#include "transform_coords.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using namespace Eigen;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

const double Lf = 2.67;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (unsigned int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (unsigned int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}



int run_message_loop(double velocity, double stepduration, size_t stepcount)
{
  int step = 0;
  long latency_delay = 100;// mSeconds
  int order = 2;// 3;//polyfit equation of this order
  if (velocity < 50)
  {
    order = 3;
  }
  uWS::Hub h;
  // this works for start at beginning of track - could put whole waypoint array in here to make more robust.
  double old_ptsx0 = -24.01;
  double old_ptsy0 = 119.02;
  double old_ptsx1 = -32.16173;
  double old_ptsy1 = 113.361;

  // MPC is initialized here!
  MPC mpc;
  

  h.onMessage([&mpc, &step, &order, &old_ptsx0, &old_ptsx1, &old_ptsy0, &old_ptsy1, &latency_delay, &velocity, &stepduration, &stepcount]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    size_t i;

    string sdata = string(data).substr(0, length);
//    cout << sdata << endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {

          //step += 1;
          cout << "step = " << ++step << ": " ;

          // j[1] is the data JSON object 
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v = v * 1609 / 3600; //switch to m/s from mph
          // just to check applied correctly from last actuation
          double throt = j[1]["throttle"];
          double steer = j[1]["steering_angle"];

          // added another waypoint behind car to help stabilize curve match, 
          // this means we need to store waypoint[1] and move it to waypoint[0] if it is no longer wapoint[1]
          // check if we need to shift old waypoint behind car to help stabilize curve at car
          //          cout << "input ptsx: " << ptsx[0] << ", " << ptsx[1] << ", " << ptsx[2] << endl;
          //          cout << "input ptsy: " << ptsy[0] << ", " << ptsy[1] << ", " << ptsy[2] << endl;
          //          cout << "old_ptsx1:  " << old_ptsx1 << endl;
          //          cout << "old_ptsy1:  " << old_ptsy1 << endl;
          if ((fabs(old_ptsx1 - ptsx[0]) > 0.1) or (fabs(old_ptsy1 - ptsy[0]) > 0.1))
          {
            //cout << "sliding waypoints along!" << endl;
            old_ptsx0 = old_ptsx1;
            old_ptsy0 = old_ptsy1;
            old_ptsx1 = ptsx[0];
            old_ptsy1 = ptsy[0];
          }
          ptsx.insert(ptsx.begin( ), old_ptsx0);
          ptsy.insert(ptsy.begin( ), old_ptsy0);

          //don't need to look so far down road 
          //remove 7th set of points to prevent trying to fit a curve that is too complex          
          for (int i = 0; i < 1; i++)
          {
            ptsx.pop_back( );
            ptsy.pop_back( );
          }

          //cout << " original    (px, py) = (" << px << ", " << py << "), v = " << v << ", psi = " << psi << ", throt = " << throt << ", steer = " << steer << " ." << endl;


          //calculate a new car position based on 100 mS latency
          //use current throt and steer to figure out position and direction
          //for now disregard acceleration and rate of steer change
          px = px + (v * CppAD::cos(psi) * (latency_delay / 1000.0));
          py = py + (v * CppAD::sin(psi) * (latency_delay / 1000.0));
          psi = psi - (v / Lf * steer / (0.436332 * Lf) * (latency_delay/1000.0));
          //cout << " transformed (px, py) = (" << px << ", " << py << "), v = " << v << ", psi = " << psi << " ." << endl;

          // Transform from car co-ords to map coords here
          Eigen::Vector3f trans_p;
          transform_coords tc(px, py, psi);
          for (i = 0; i < ptsx.size( ); i++)
          {
            // Transform matrix from vehicle to global x position in global map.
            trans_p = tc.transform(ptsx[i], ptsy[i]);
            ptsx[i] = trans_p[0];
            ptsy[i] = trans_p[1];
          }
          
//          cout << " transformed co-ordinates: " << endl;
//          for (i = 0; i < ptsx.size( ); i++)
//          {
//            cout << ptsx[i] << ", " << ptsy[i] << endl;
//          }
//          cout << endl;

          Eigen::VectorXd xvals = VectorXd::Map(ptsx.data( ), ptsx.size( ));
          Eigen::VectorXd yvals = VectorXd::Map(ptsy.data( ), ptsy.size( ));

          //find coefficients to fit for order polynomial to waypoints
          auto coeffs = polyfit(xvals, yvals, order);
          for (i = 0; i < coeffs.size( ); i++)
         {
            cout << "coeff[" << i << "] = " << coeffs[i] << " ";
         }

          //since px and py translated to 0,0 to car perspective
          double cte = polyeval(coeffs, 0.0) - 0.0;
          //cout << "cte is : " << cte << endl;

                  
          // Calculate a slightly better approximation of CTE
          // assume track near linear at location of waypoint (0,cte)
          // use this to calculate angle of waypoint path

          //calculate a point on the polyline close to the cte to get a decent estimation of direction of line
          // this could become erroneous if the car is at a very large angle to the track, and the polyline is very curvy
          // could do a recursive check
          double pt1 = polyeval(coeffs, -1.0) - 0.0;
          double pt2 = polyeval(coeffs, 1.0) - 0.0;        
          double angle = atan2(pt2 - pt1, 1.0 - (-1));
          //cout << "translate (x1,y1) = (" << -1.0 << ", " << pt1 << "), (x2, y2) = (1.0, " << pt2 << "), angle = " << angle << endl;
          //run the transform using the current location of the cte on the polyline, and the line direction          
          transform_coords tc1(0, cte, angle);
          //now transform the car co-ordinates to the reference of the polyline
          trans_p = tc1.transform(0, 0);
          double cte1 = trans_p[1];
          //cout << "cte1 is at: (" << trans_p[0] << ", "<< trans_p[1] << "). CTE Difference of " << (-cte1-cte) << " from " << cte << endl;
          cte = -cte1;

          //exit if car has left track 
          if (fabs(cte) > 10.0)
          {
            cout << endl << endl;
            cout << "ERROR:: Car off track - cte value exceeds 10 - EXITING!" << endl << endl;
            exit(-1);
          }

          //calculate error in psi  
          //double epsi = psi - atan(coeffs[1] + (2 * px * coeffs[2]) + (3 * coeffs[3] * px * px));
          // second and third terms work out to zero since px = 0 by def'n, so:
          double epsi = -CppAD::atan(coeffs[1]);
          //cout << ",  epsi is : " << epsi << endl;

          // solve the equation!
          Eigen::VectorXd state(6);
          // remember px, py, psi changed to cars perspective (0,0,0)
          state << 0.0, 0.0, 0.0, v, cte, epsi;
          auto vars = mpc.Solve(state, coeffs, velocity, stepduration, stepcount);
          
          double steer_value = vars[0] / (0.436332 * Lf);  //convert from rads to 1 unit  = 25 degrees = 0.436332 rads
          double throttle_value = vars[1];
          //cout << "Apply steer of " << steer_value << " and throttle of " << throttle_value << "." << endl;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = Lf; //distance from centroid of car to front of car
          int num_points = 10;
          //make 
          for (int i = 0; i < num_points; i++)
          {
            next_x_vals.push_back((i + 1.5)*poly_inc);
            next_y_vals.push_back( polyeval(coeffs, (i + 1.5)*poly_inc) );
          }

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (i = 2; i < vars.size(); )
          {
            mpc_x_vals.push_back(vars[i++]);
            mpc_y_vals.push_back(vars[i++]);
          }
          
          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
//          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //

          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(latency_delay));

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
  return 0;
}



int main(int argc, char *argv[ ]) {
  double velocity_goal = 70.00;
  //defaults
  double stepduration = 0.05;
  size_t stepcount = 14;

  if (argc == 2)
  {
    cout << "The initial velocity argument supplied is " << argv[1] << endl;
    double tmp_vel = atof(argv[1]);
    if (tmp_vel > 0 and tmp_vel < 100)
    {
      velocity_goal = tmp_vel;
    }

    if (velocity_goal > 74)
    {
      stepcount = 12;
    }
    else if (velocity_goal >= 45)
    {
      stepcount = 24 - int((velocity_goal - 45) / 5 * 2);
    }else
      stepcount = 24;
    cout << stepcount << " steps of duration, " << stepduration << " seconds will be used by default." << endl;
  }
  else if (argc == 4)
  {
    cout << "The initial velocity argument supplied is " << argv[1] << endl;
    double tmp_vel = atof(argv[1]);
    if (tmp_vel > 0 and tmp_vel < 100)
    {
      velocity_goal = tmp_vel;
    }
    cout << "The initial stepduration argument supplied is " << argv[2] << endl;
    double tmp_stepduration = atof(argv[2]);
    if (tmp_stepduration > 0.01 and tmp_stepduration < 0.5)
    {
      stepduration = tmp_stepduration;
    }
    cout << "The initial stepcount argument supplied is " << argv[3] << endl;
    size_t tmp_stepcount = atoi(argv[3]);
    if (tmp_stepcount > 3 and tmp_stepcount < 31)
    {
      stepcount = tmp_stepcount;
    }

  }
  else
  {
    cout << "To specify the velocity goal to use at run time call " << endl << endl;
    cout << argv[0] << " velocity" << endl << endl;
    cout << "like this:" << endl << argv[0] << " 55" << endl;
    cout << "velocity goal should be a float between 1 and 100." << endl;
    cout << "Otherwise, " << velocity_goal << " will be used by default." << endl;
    cout << "You can also adjust the stepduration and stepcount " << endl << endl;
    cout << argv[0] << " velocity stepduration stepcount" << endl << endl;
    cout << "like this:" << endl << argv[0] << " 55 0.05 20" << endl;
    cout << "stepduration should be a float between 0.01 and 0.50." << endl;
    cout << "stepcount should be between 4 and 30." << endl;
    cout << "Otherwise, " << velocity_goal << ", " << stepcount << ", " << stepduration <<" will be used by default." << endl;

  }

  run_message_loop(velocity_goal, stepduration, stepcount);

}
