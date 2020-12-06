// Copyright 2020 Ray Burgemeestre
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of
// this software and associated documentation files (the "Software"), to deal in
// the Software without restriction, including without limitation the rights to
// use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
// of the Software, and to permit persons to whom the Software is furnished to do
// so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

//
// Header-only Perlin Noise generator
//
// Author: Ray Burgemeestre
// Ported from Christian Petry's javascript implementation. Original Copyright header below.

#pragma once

/*
 * Author: Christian Petry
 * Homepage: www.petry-christian.de
 *
 * License: MIT
 * Copyright (c) 2014 Christian Petry
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and permission notice shall be included in all copies or
 * substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

// Ported from Stefan Gustavson's java implementation
// http://staffwww.itn.liu.se/~stegu/simplexnoise/simplexnoise.pdf
//

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

class SimplexNoise {
private:
  std::vector<std::array<int, 3>> grad3;
  std::vector<std::array<int, 4>> grad4;
  std::vector<int> p;
  std::vector<int> perm;
  std::vector<int> permMod12;
  double F2;
  double G2;
  std::vector<std::array<int, 4>> simplex;

public:
  SimplexNoise(double seed = rand() * 256)
      : grad3({{1, 1, 0}, {-1, 1, 0}, {1, -1, 0}, {-1, -1, 0}, {1, 0, 1}, {-1, 0, 1}, {1, 0, -1}, {-1, 0, -1},
               {0, 1, 1}, {0, -1, 1}, {0, 1, -1}, {0, -1, -1}}),
        grad4({{0, 1, 1, 1},   {0, 1, 1, -1},   {0, 1, -1, 1},  {0, 1, -1, -1},  {0, -1, 1, 1},  {0, -1, 1, -1},
               {0, -1, -1, 1}, {0, -1, -1, -1}, {1, 0, 1, 1},   {1, 0, 1, -1},   {1, 0, -1, 1},  {1, 0, -1, -1},
               {-1, 0, 1, 1},  {-1, 0, 1, -1},  {-1, 0, -1, 1}, {-1, 0, -1, -1}, {1, 1, 0, 1},   {1, 1, 0, -1},
               {1, -1, 0, 1},  {1, -1, 0, -1},  {-1, 1, 0, 1},  {-1, 1, 0, -1},  {-1, -1, 0, 1}, {-1, -1, 0, -1},
               {1, 1, 1, 0},   {1, 1, -1, 0},   {1, -1, 1, 0},  {1, -1, -1, 0},  {-1, 1, 1, 0},  {-1, 1, -1, 0},
               {-1, -1, 1, 0}, {-1, -1, -1, 0}}),
        // Skewing and unskewing factors for 2 dimensions
        F2(0.5 * (sqrt(3.0) - 1.0)),
        G2((3.0 - sqrt(3.0)) / 6.0),
        // A lookup table to traverse the simplex around a given point in 4D.
        // Details can be found where this table is used, in the 4D noise method.
        simplex({{0, 1, 2, 3}, {0, 1, 3, 2}, {0, 0, 0, 0}, {0, 2, 3, 1}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
                 {1, 2, 3, 0}, {0, 2, 1, 3}, {0, 0, 0, 0}, {0, 3, 1, 2}, {0, 3, 2, 1}, {0, 0, 0, 0}, {0, 0, 0, 0},
                 {0, 0, 0, 0}, {1, 3, 2, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
                 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {1, 2, 0, 3}, {0, 0, 0, 0}, {1, 3, 0, 2}, {0, 0, 0, 0},
                 {0, 0, 0, 0}, {0, 0, 0, 0}, {2, 3, 0, 1}, {2, 3, 1, 0}, {1, 0, 2, 3}, {1, 0, 3, 2}, {0, 0, 0, 0},
                 {0, 0, 0, 0}, {0, 0, 0, 0}, {2, 0, 3, 1}, {0, 0, 0, 0}, {2, 1, 3, 0}, {0, 0, 0, 0}, {0, 0, 0, 0},
                 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {2, 0, 1, 3},
                 {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {3, 0, 1, 2}, {3, 0, 2, 1}, {0, 0, 0, 0}, {3, 1, 2, 0},
                 {2, 1, 0, 3}, {0, 0, 0, 0}, {0, 0, 0, 0}, {0, 0, 0, 0}, {3, 1, 0, 2}, {0, 0, 0, 0}, {3, 2, 0, 1},
                 {3, 2, 1, 0}}) {
    for (auto i = 0; i < 256; i++) {
      p.push_back(fastfloor(randomSeed(seed++) * 256));
    }

    // To remove the need for index wrapping, double the permutation table length
    for (auto i = 0; i < 512; i++) {
      auto v = p[i & 255];
      perm.push_back(v);
      permMod12.push_back(fastmod(v, 12));
    }
  }

  double dot(std::array<int, 3> g, double x, double y) {
    return g[0] * x + g[1] * y;
  }

  double dot3(std::array<int, 3> g, double x, double y, double z) {
    return g[0] * x + g[1] * y + g[2] * z;
  }

  double dot4(std::array<int, 4> g, double x, double y, double z, double w) {
    return g[0] * x + g[1] * y + g[2] * z + g[3] * w;
  }

  enum NoiseTypeEnum { PERLINNOISE = 0, FRACTALNOISE = 1, TURBULENCE = 2 };

  // 2D Multi-octave Simplex noise.
  //
  // For each octave, a higher frequency/lower amplitude function will be added to the original.
  // The higher the persistence [0-1], the more of each succeeding octave will be added.
  double simplexNoise(NoiseTypeEnum type,
                      double size,
                      int octaves,
                      double persistence,
                      double percentage,
                      double scale,
                      double x,
                      double y,
                      double z) {
    double total = 0;
    double frequency = 0.25;
    double amplitude = 1;
    double offset = size;
    double power = 1 / frequency;
    // We have to keep track of the largest possible amplitude,
    // because each octave adds more, and we need a value in [-1, 1].
    double maxAmplitude = 0;

    // x = (x+offset);
    // y = (y+offset);

    for (int i = 0; i < octaves; i++) {
      // var noise_v = this.noise(x * frequency, y * frequency);
      double x_0_1 = x / size * frequency;
      double y_0_1 = y / size * frequency;
      double z_0_1 = z;  //  / size * frequency;
      double noise_v = seamlessNoise(x_0_1, y_0_1, z_0_1, scale, scale, offset);

      // noise_v = Math.min (noise_v, (1-percentage));
      if (type == PERLINNOISE)
        total += noise_v * amplitude;
      else if (type == FRACTALNOISE)
        total += std::abs(noise_v) * amplitude;
      else if (type == TURBULENCE)
        total += std::abs(noise_v) * amplitude;

      frequency *= 2;
      maxAmplitude += amplitude;
      amplitude *= persistence;
    }

    if (type == TURBULENCE) total = sin((x / scale) + total);

    double retnoise = total / maxAmplitude;

    if (type == TURBULENCE) retnoise = total;

    if (type == PERLINNOISE || type == TURBULENCE)
      retnoise = std::max(retnoise + percentage, 0.) / (1.0 + percentage);  // [0, 1.0]

    retnoise = pow(retnoise, 1 + 2 * (1 - percentage));

    return retnoise;
  }

  // x, y are normalized coordinates (in [0..1] space).
  // dx, dy are noise scale in x and y axes.
  // xyOffset is noise offset (same offset will result in having the same noise).
  double seamlessNoise(double x, double y, double z, double dx, double dy, double xyoffset) {
    static double global = 0;
    double s = x;
    double t = y;
    double u = z;

    double nx = xyoffset + cos(s * 2.0 * M_PI) * dx / (2.0 * M_PI);
    double ny = xyoffset + cos(t * 2.0 * M_PI) * dy / (2.0 * M_PI);
    double nz = xyoffset + sin(s * 2.0 * M_PI) * dx / (2.0 * M_PI);
    double nw = xyoffset + sin(t * 2.0 * M_PI) * dy / (2.0 * M_PI);

    return noise4d(nx + u, ny + u, nz + u, nw);
  }

  double noise(double xin, double yin) {
    double n0, n1, n2;  // Noise contributions from the three corners
    // Skew the input space to determine which simplex cell we're in

    double s = (xin + yin) * F2;  // Hairy factor for 2D
    int i = fastfloor(xin + s);
    int j = fastfloor(yin + s);

    double t = (i + j) * G2;
    double X0 = i - t;  // Unskew the cell origin back to (x,y) space
    double Y0 = j - t;
    double x0 = xin - X0;  // The x,y distances from the cell origin
    double y0 = yin - Y0;

    // For the 2D case, the simplex shape is an equilateral triangle.
    // Determine which simplex we are in.
    int i1, j1;  // Offsets for second (middle) corner of simplex in (i,j) coords
    if (x0 > y0) {
      i1 = 1;
      j1 = 0;
    }  // lower triangle, XY order: (0,0)->(1,0)->(1,1)
    else {
      i1 = 0;
      j1 = 1;
    }  // upper triangle, YX order: (0,0)->(0,1)->(1,1)

    // A step of (1,0) in (i,j) means a step of (1-c,-c) in (x,y), and
    // a step of (0,1) in (i,j) means a step of (-c,1-c) in (x,y), where
    // c = (3-sqrt(3))/6
    double x1 = x0 - i1 + G2;  // Offsets for middle corner in (x,y) unskewed coords
    double y1 = y0 - j1 + G2;
    double x2 = x0 - 1.0 + 2.0 * G2;  // Offsets for last corner in (x,y) unskewed coords
    double y2 = y0 - 1.0 + 2.0 * G2;

    // Work out the hashed gradient indices of the three simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int gi0 = permMod12[ii + perm[jj]];
    int gi1 = permMod12[ii + i1 + perm[jj + j1]];
    int gi2 = permMod12[ii + 1 + perm[jj + 1]];

    // Calculate the contribution from the three corners
    double t0 = 0.5 - x0 * x0 - y0 * y0;
    if (t0 < 0)
      n0 = 0.0;
    else {
      t0 *= t0;
      n0 = t0 * t0 * dot(grad3[gi0], x0, y0);  // (x,y) of grad3 used for 2D gradient
    }
    double t1 = 0.5 - x1 * x1 - y1 * y1;
    if (t1 < 0)
      n1 = 0.0;
    else {
      t1 *= t1;
      n1 = t1 * t1 * dot(grad3[gi1], x1, y1);
    }
    double t2 = 0.5 - x2 * x2 - y2 * y2;
    if (t2 < 0)
      n2 = 0.0;
    else {
      t2 *= t2;
      n2 = t2 * t2 * dot(grad3[gi2], x2, y2);
    }

    // Add contributions from each corner to get the final noise value.
    // The result is scaled to return values in the interval [-1,1].

    double r = 70.0 * (n0 + n1 + n2);
    // r = (r + 1) / 2; //interval [0,1].
    return r;
  }

  // 4D simplex noise
  double noise4d(double x, double y, double z, double w) {
    // The skewing and unskewing factors are hairy again for the 4D case
    static double F4 = (sqrt(5.0) - 1.0) / 4.0;
    static double G4 = (5.0 - sqrt(5.0)) / 20.0;
    double n0, n1, n2, n3, n4;  // Noise contributions from the five corners
    // Skew the (x,y,z,w) space to determine which cell of 24 simplices we're in
    double s = (x + y + z + w) * F4;  // Factor for 4D skewing
    int i = floor(x + s);
    int j = floor(y + s);
    int k = floor(z + s);
    int l = floor(w + s);
    double t = (i + j + k + l) * G4;  // Factor for 4D unskewing
    double X0 = i - t;                // Unskew the cell origin back to (x,y,z,w) space
    double Y0 = j - t;
    double Z0 = k - t;
    double W0 = l - t;
    double x0 = x - X0;  // The x,y,z,w distances from the cell origin
    double y0 = y - Y0;
    double z0 = z - Z0;
    double w0 = w - W0;

    // For the 4D case, the simplex is a 4D shape I won't even try to describe.
    // To find out which of the 24 possible simplices we're in, we need to
    // determine the magnitude ordering of x0, y0, z0 and w0.
    // The method below is a good way of finding the ordering of x,y,z,w and
    // then find the correct traversal order for the simplex we’re in.
    // First, six pair-wise comparisons are performed between each possible pair
    // of the four coordinates, and the results are used to add up binary bits
    // for an integer index.
    int c1 = (x0 > y0) ? 32 : 0;
    int c2 = (x0 > z0) ? 16 : 0;
    int c3 = (y0 > z0) ? 8 : 0;
    int c4 = (x0 > w0) ? 4 : 0;
    int c5 = (y0 > w0) ? 2 : 0;
    int c6 = (z0 > w0) ? 1 : 0;
    int c = c1 + c2 + c3 + c4 + c5 + c6;
    int i1, j1, k1, l1;  // The integer offsets for the second simplex corner
    int i2, j2, k2, l2;  // The integer offsets for the third simplex corner
    int i3, j3, k3, l3;  // The integer offsets for the fourth simplex corner
    // simplex[c] is a 4-vector with the numbers 0, 1, 2 and 3 in some order.
    // Many values of c will never occur, since e.g. x>y>z>w makes x<z, y<w and x<w
    // impossible. Only the 24 indices which have non-zero entries make any sense.
    // We use a thresholding to set the coordinates in turn from the largest magnitude.
    // The number 3 in the "simplex" array is at the position of the largest coordinate.
    i1 = simplex[c][0] >= 3 ? 1 : 0;
    j1 = simplex[c][1] >= 3 ? 1 : 0;
    k1 = simplex[c][2] >= 3 ? 1 : 0;
    l1 = simplex[c][3] >= 3 ? 1 : 0;
    // The number 2 in the "simplex" array is at the second largest coordinate.
    i2 = simplex[c][0] >= 2 ? 1 : 0;
    j2 = simplex[c][1] >= 2 ? 1 : 0;
    k2 = simplex[c][2] >= 2 ? 1 : 0;
    l2 = simplex[c][3] >= 2 ? 1 : 0;
    // The number 1 in the "simplex" array is at the second smallest coordinate.
    i3 = simplex[c][0] >= 1 ? 1 : 0;
    j3 = simplex[c][1] >= 1 ? 1 : 0;
    k3 = simplex[c][2] >= 1 ? 1 : 0;
    l3 = simplex[c][3] >= 1 ? 1 : 0;
    // The fifth corner has all coordinate offsets = 1, so no need to look that up.
    double x1 = x0 - i1 + G4;  // Offsets for second corner in (x,y,z,w) coords
    double y1 = y0 - j1 + G4;
    double z1 = z0 - k1 + G4;
    double w1 = w0 - l1 + G4;
    double x2 = x0 - i2 + 2.0 * G4;  // Offsets for third corner in (x,y,z,w) coords
    double y2 = y0 - j2 + 2.0 * G4;
    double z2 = z0 - k2 + 2.0 * G4;
    double w2 = w0 - l2 + 2.0 * G4;
    double x3 = x0 - i3 + 3.0 * G4;  // Offsets for fourth corner in (x,y,z,w) coords
    double y3 = y0 - j3 + 3.0 * G4;
    double z3 = z0 - k3 + 3.0 * G4;
    double w3 = w0 - l3 + 3.0 * G4;
    double x4 = x0 - 1.0 + 4.0 * G4;  // Offsets for last corner in (x,y,z,w) coords
    double y4 = y0 - 1.0 + 4.0 * G4;
    double z4 = z0 - 1.0 + 4.0 * G4;
    double w4 = w0 - 1.0 + 4.0 * G4;
    // Work out the hashed gradient indices of the five simplex corners
    int ii = i & 255;
    int jj = j & 255;
    int kk = k & 255;
    int ll = l & 255;
    int gi0 = perm[ii + perm[jj + perm[kk + perm[ll]]]] % 32;
    int gi1 = perm[ii + i1 + perm[jj + j1 + perm[kk + k1 + perm[ll + l1]]]] % 32;
    int gi2 = perm[ii + i2 + perm[jj + j2 + perm[kk + k2 + perm[ll + l2]]]] % 32;
    int gi3 = perm[ii + i3 + perm[jj + j3 + perm[kk + k3 + perm[ll + l3]]]] % 32;
    int gi4 = perm[ii + 1 + perm[jj + 1 + perm[kk + 1 + perm[ll + 1]]]] % 32;
    // Calculate the contribution from the five corners
    double t0 = 0.6 - x0 * x0 - y0 * y0 - z0 * z0 - w0 * w0;
    if (t0 < 0)
      n0 = 0.0;
    else {
      t0 *= t0;
      n0 = t0 * t0 * dot4(grad4[gi0], x0, y0, z0, w0);
    }
    double t1 = 0.6 - x1 * x1 - y1 * y1 - z1 * z1 - w1 * w1;
    if (t1 < 0)
      n1 = 0.0;
    else {
      t1 *= t1;
      n1 = t1 * t1 * dot4(grad4[gi1], x1, y1, z1, w1);
    }
    double t2 = 0.6 - x2 * x2 - y2 * y2 - z2 * z2 - w2 * w2;
    if (t2 < 0)
      n2 = 0.0;
    else {
      t2 *= t2;
      n2 = t2 * t2 * dot4(grad4[gi2], x2, y2, z2, w2);
    }
    double t3 = 0.6 - x3 * x3 - y3 * y3 - z3 * z3 - w3 * w3;
    if (t3 < 0)
      n3 = 0.0;
    else {
      t3 *= t3;
      n3 = t3 * t3 * dot4(grad4[gi3], x3, y3, z3, w3);
    }
    double t4 = 0.6 - x4 * x4 - y4 * y4 - z4 * z4 - w4 * w4;
    if (t4 < 0)
      n4 = 0.0;
    else {
      t4 *= t4;
      n4 = t4 * t4 * dot4(grad4[gi4], x4, y4, z4, w4);
    }
    // Sum up and scale the result to cover the range [-1,1]
    return 27.0 * (n0 + n1 + n2 + n3 + n4);
  }

private:
  // inline helpers

  // This method is a *lot* faster than using (int)Math.floor(x)
  // Ray: I didn't bother to benchmark C++'s floor() yet, everything ported
  //  from Javascript/Java quite literally
  int fastfloor(double x) {
    auto xi = (int)x;
    return x < xi ? xi - 1 : xi;
  }

  double randomSeed(double seed) {
    auto x = sin(seed) * 10000;
    return x - floor(x);
  }

  int fastmod(int in, int n) {
    return ((in % n) + n) % n;
  }
};
