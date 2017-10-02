
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
#include "transform_coords.h"


transform_coords::transform_coords( )
{
  cout << "Instantiating blank transform_coordinates" << endl;
}
transform_coords::~transform_coords( )
{
  //cout << "Cleaning up transform_coordinates" << endl;
  //cout << "T: " << endl << T << endl << "Ti: " << endl << Ti << endl << "P: " << endl << P << endl;
}

transform_coords::transform_coords(double px, double py, double psi)
{
  T << cos(psi), -sin(psi), px,
    sin(psi), cos(psi), py,
    0, 0, 1;
  Ti = T.inverse( );
}

Eigen::Vector3f transform_coords::transform(double x1, double y1)
{
  P << x1, y1, 1;
  // Transform matrix from vehicle to global x position in global map.
  Eigen::Vector3f trans_p = Ti*P;
  return trans_p;
}
