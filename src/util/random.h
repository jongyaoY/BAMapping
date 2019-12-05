

#ifndef CERES_EXAMPLES_RANDOM_H_
#define CERES_EXAMPLES_RANDOM_H_

#include <math.h>
#include <stdlib.h>


// Return a random number sampled from a uniform distribution in the range
// [0,1].
inline double RandDouble() {
  double r = static_cast<double>(rand());
  return r / RAND_MAX;
}

// Marsaglia Polar method for generation standard normal (pseudo)
// random numbers http://en.wikipedia.org/wiki/Marsaglia_polar_method
inline double RandNormal() {
  double x1, x2, w;
  do {
    x1 = 2.0 * RandDouble() - 1.0;
    x2 = 2.0 * RandDouble() - 1.0;
    w = x1 * x1 + x2 * x2;
  } while ( w >= 1.0 || w == 0.0 );

  w = sqrt((-2.0 * log(w)) / w);
  return x1 * w;
}


#endif  // CERES_EXAMPLES_RANDOM_H_
