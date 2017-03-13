#ifndef RAND_GENE_H
#define RAND_GENE_H
#define _USE_MATH_DEFINES
#include "util/NumType.h"
#include <cmath>
#include <iostream>
#include <random>
#include <stdio.h>  /* printf, scanf, puts, NULL */
#include <stdlib.h> /* srand, rand */
#include <time.h>   /* time */
namespace dso {
class randomGenerator {
public:
  template <typename NUMTYPE>
  static void drawUniformVector(std::vector<NUMTYPE> &series, int size,
                                NUMTYPE start, NUMTYPE end);
  static Vec3 RandVec3Plane(Vec3 normvec);
  static Vec3 RandVec3Box(Vec3 pos, Vec3 size);
  static Vec3 RandVec3columnar(Vec3 pos, float radius, float height);
};
}

#endif /* end of include guard: RAND_GENE_H */
