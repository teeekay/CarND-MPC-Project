#ifndef X_COORDS_H
#define X_COORDS_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;
class transform_coords
{
public:
  //Transform Matrix 3x3
  Eigen::Matrix3f T;

  //Inverse Transform Matrix 3x3
  Eigen::Matrix3f Ti;

  //Pose Vector (x,y,1) but actually a matrix of size (1,3)
  Eigen::Vector3f P;

  transform_coords( );

  //set up Transform Matrix and Inverse Matrix
  transform_coords(double px, double py, double psi);

  ~transform_coords( );

  Eigen::Vector3f transform(double x1, double y1);

};

#endif /* X_COORDS_H */
