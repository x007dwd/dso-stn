#include "Eigen/Core"
#include "util/NumType.h"
#include <iostream>
#include <vector>
using namespace std;
int main(int argc, char const *argv[]) {
  std::vector<char> v;
  // v.push_back('a');
  for (size_t i = 0; i < v.size(); i++) {
    v.push_back('a');
    /* code */
  }
  std::cout << v.size() << std::endl;
  for (float rotDelta = 0.02; rotDelta < 0.05; rotDelta++) {
    std::cout << rotDelta << std::endl;
  }
  Eigen::Matrix3d aa = 5 * Eigen::Matrix3d::Random(3, 3);
  std::cout << "/* aa */\n" << aa << std::endl;
  dso::Vec3 bb(1, 0, -1);
  std::cout << "mat mutliply vec \n" << (aa * bb) << std::endl;
  std::cout << "/* aa --bb */\n"
            << aa - bb.transpose().replicate(3, 1) << std::endl;
  return 0;
}
