#include "stn/randomGenerator.h"
namespace dso {
Vec3 randomGenerator::RandVec3Plane(Vec3 normvec) {
  float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100;
  float y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 100;
  float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * 30;
  z += -(normvec[0] * x + normvec[1] * y) / normvec[2];
  return Vec3(x, y, z);
}

Vec3 randomGenerator::RandVec3Box(Vec3 pos, Vec3 size) {
  float x = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * size[0];
  float y = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * size[1];
  float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * size[2];

  x += pos[0];
  y += pos[1];
  z += pos[2];
  return Vec3(x, y, z);
}
Vec3 randomGenerator::RandVec3columnar(Vec3 pos, float radius, float height) {
  float theta =
      static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * M_PI * 2;
  float x =
      static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * radius * 0.1;
  float y =
      static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * radius * 0.1;
  float z = static_cast<float>(rand()) / static_cast<float>(RAND_MAX) * height;

  x += radius * std::cos(theta);
  y += radius * std::sin(theta);
  return Vec3(x, y, z);
}
template <typename NUMTYPE>
void drawUniformVector(std::vector<NUMTYPE> &series, int size, NUMTYPE start,
                       NUMTYPE end) {
  std::default_random_engine generator;
  std::uniform_real_distribution<NUMTYPE> distribution(start, end);

  series.resize(size);

  for (int i = 0; i < size; ++i) {
    series.push_back(distribution(generator));
  }
}
}
