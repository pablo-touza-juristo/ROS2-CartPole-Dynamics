#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
int main()
{
  Eigen::Matrix4d test_matrix 
  {
    {1, 2, 3, 4},
    {5, 6, 7, 8},
    {9, 10, 11, 12},
    {13, 14, 15, 16}
  };

  std::cout << "Test Matrix: " << test_matrix << std::endl; 

  return 0;
}
