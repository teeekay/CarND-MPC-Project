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

int run_message_loop()
{
  int step = 0;
  int order = 4;// 3;//polyfit equation of this order
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;
  

  h.onMessage([&mpc, &step, &order](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
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
          step += 1;
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

          cout << "step = " << step << endl;
          cout << ", x = " << px << ", y = " << py << ", psi = " << psi << ", v = " << v << ", throt = " << throt <<
            ", steer = " << steer << ".";

//          cout << "  There are " << ptsx.size( ) << " points in path horizon on step " << step << "." << endl;
          
//          cout << " original co-ordinates: " << endl;
//          for (i = 0; i < ptsx.size( ); i++)
//          {
//            cout << ptsx[i] << ", " << ptsy[i] << endl;
//          }
//          cout << endl;

          // TMK need to transform from car co-ords to map coords here
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
  //        for (i = 0; i < coeffs.size( ); i++)
  //        {
  //          cout << "coeffs[" << i << "] = " << coeffs[i] << endl;
  //        }

          //since px and py translated to 0,0 to car perspective
          double cte = polyeval(coeffs, 0.0) - 0.0;
          cout << "cte is : " << cte ;
          if (fabs(cte) > 5.0)
          {
            cout << endl << endl;
            cout << "ERROR:: Car off track - cte value exceeds 5 - EXITING!" << endl << endl;
            exit(-1);
          }
//
//          transform_coords tc1(ptsx[0], ptsy[0], atan2(ptsy[1]-cte, ptsx[1] - 0.0)); 
//          trans_p = tc1.transform(0, 0);
//          double cte1 = trans_p[1];
//          cout << ", cte1 is at: (" << trans_p[0] << ", "<< cte1 << ") ";
         

          //double epsi = psi - atan(coeffs[1] + (2 * px * coeffs[2]) + (3 * coeffs[3] * px * px));
          // second and third terms work out to zero since px = 0 by def'n, so:
          double epsi = -CppAD::atan(coeffs[1]);
          cout << ",  epsi is : " << epsi << endl;

          Eigen::VectorXd state(6);
          // remember px, py, psi changed to cars perspective (0,0,0)
          state << 0.0, 0.0, 0.0, v, cte, epsi;

          auto vars = mpc.Solve(state, coeffs);

          //use current throt and steer to figure out position after latency.
          
          double steer_value = vars[0] / (0.436332 * Lf);  //convert from rads to 1 unit  = 25 degrees = 0.436332 rads
          double throttle_value = vars[1];

          cout << "Apply steer of " << steer_value << " and throttle of " << throttle_value << "." << endl;

          //Display the waypoints/reference line
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          double poly_inc = Lf; //distance from centroid of car to front of car
          size_t num_points = 20;
          for (i = 0; i < num_points; i++)
          {
            next_x_vals.push_back((i + 1)*poly_inc);
            next_y_vals.push_back( polyeval(coeffs, (i + 1)*poly_inc) );
          }

          //Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

//          std::cout << "vars: ";
            
          for (i = 2; i < vars.size(); )
          {
//            std::cout << "(" << vars[i] << ", ";
            mpc_x_vals.push_back(vars[i++]);
//            std::cout << vars[i] << "), ";
            mpc_y_vals.push_back(vars[i++]);
          }
//          std::cout << endl;
          //std::cout << "mpc_x_vals" << mpc_x_vals << endl;
          //std::cout << "mpc_y_vals" << mpc_y_vals << endl;
          
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
          // this_thread::sleep_for(chrono::milliseconds(100));

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

void test_suite_1( )
{
  MPC mpc;
  vector<double> ptsx = { 6,7,8,9,10,11,12 };
  vector<double> ptsy = { 6,7,8,9,10,11,12 };
  double px = 5;
  double py = 5;
  double psi = M_PI / 8.;
  double v = 10;
  transform_coords tc(px, py, psi);

  cout << "Transform Map Co-ords: " << endl;

  for (unsigned int i = 0; i < ptsx.size( ); i++)
  {
    cout << "(" << ptsx[i] << ", " << ptsy[i] << ") " << endl;
  }

  cout << "to co-ordinates relative to car at (" << px << ", " << py << ") and direction " << psi * 180 / M_PI << endl;

  Eigen::Vector3f trans_p;
  for (unsigned int i = 0; i < ptsx.size( ); i++)
  {
    // Transform matrix from vehicle to global x position in global map.
    trans_p = tc.transform(ptsx[i], ptsy[i]);
    ptsx[i] = trans_p[0];
    ptsy[i] = trans_p[1];
    cout << "(" << ptsx[i] << ", " << ptsy[i] << ") " << endl;
  }

  Eigen::VectorXd xvals = VectorXd::Map(ptsx.data( ), ptsx.size( ));
  Eigen::VectorXd yvals = VectorXd::Map(ptsy.data( ), ptsy.size( ));
  cout << "And translate to Eigen " << endl << xvals << endl << yvals << endl;

  //find coefficients to fit for order polynomial to waypoints
  auto coeffs = polyfit(xvals, yvals, 3);
  //since px and py translated to 0,0 to car perspective
  double cte = polyeval(coeffs, 0) - 0;

  //double epsi = psi - atan(coeffs[1] + 2 * px * coeffs[2] + 3 * coeffs[3] * px * px);
  double epsi = -CppAD::atan(coeffs[1]);

  cout << "coeffs: (" << coeffs.size( ) << "(";

  for (unsigned int i = 0; i < coeffs.size( ); i++)
  {
    cout << coeffs[i] << ", ";
  }
  cout << ")" << endl;

  cout << "cte: " << cte << ", epsi: " << epsi << "." << endl;

  Eigen::VectorXd state(6);
  // remember px, py, psi changed to cars perspective (0,0,0)
  state << 0.0, 0.0, 0.0, v, cte, epsi;

  auto vars = mpc.Solve(state, coeffs);

  double steer_value = vars[0] / 0.436332;  //convert from rads to 1 unit  = 25 degrees = 0.436332 rads

  double throttle_value = vars[1];
  cout << "Apply steer of " << steer_value << " and throttle of " << throttle_value << "." << endl;
}


int main() {
  //test_suite_1();

  run_message_loop( );

}
