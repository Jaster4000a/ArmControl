#ifndef METHODS_H_
#define METHODS_H_

void point2joint(float x, float y, float z, float pitch, float rotation, float *alpha, float *beta, float *gamma, float *delta, float*epsilon) { //where y is up
  float alpha1, beta1, beta2, gamma1, gamma2;
  float dist_ce, height, dist_be, dist_zx, dist;

  if (x >= 0) {
    alpha1 = atan(z / x);
  } else if ( x < 0) {
    alpha1 = atan(z / x) + PI;
  } else if (x == 0) {
    alpha1 = 0;
  }

  if (pitch == 0) {
    dist_ce = Lc + Ld;
  } else {
    dist_ce = sqrt(pow(Lc, 2) + pow(Ld, 2) - 2 * Lc * Ld * cos(PI + pitch));
  }

  dist_zx = sqrt(pow(x, 2) + pow(z, 2));
  height = y - (La); //-6.5/39.3701
  dist = (pow(x, 2) + pow(height, 2) + pow(z, 2));
  dist_be = sqrt(pow(dist_zx, 2) + pow(height, 2));
  if (pitch < 0) {
    gamma2 = acos((pow(dist_ce, 2) + pow(Lc, 2) - pow(Ld, 2)) / (2 * dist_ce * Lc));
  } else if ( pitch > 0) {
    gamma2 = -acos((pow(dist_ce, 2) + pow(Lc, 2) - pow(Ld, 2)) / (2 * dist_ce * Lc));
  } else {
    gamma2 = 0;
  }

  if (dist_be == Lb + dist_ce) {
    gamma1 = PI;
  } else {
    gamma1 = acos((pow(dist_ce, 2) + pow(Lb, 2) - pow(dist_be, 2)) / (2 * Lb * dist_ce));
  }
  beta1 = atan(height / dist_zx);
  beta2 = acos((pow(Lb, 2) + pow(dist_be, 2) - pow(dist_ce, 2)) / (2 * Lb * dist_be));
  *alpha = alpha1;
  *beta = beta1 + beta2;
  *gamma = gamma1 + gamma2 - PI;
  *delta = pitch;
  *epsilon = rotation;

}

void point2jointV2(float xd, float yd, float zd, float pitch, float rotation, float *alpha, float *beta, float *gamma, float *delta, float *epsilon) {
  float rd = sqrt(pow(xd, 2) + pow(zd, 2));
  float c = rd - Ld * cos(pitch);
  float d = yd - La;

  *alpha = atan2(-zd, xd);
  *beta = 2 * atan((2 * d * Lb - pow(((- pow(c, 2) - pow(d, 2) + pow(Lb, 2) + 2 * Lb * Lc + pow(Lc, 2)) * (pow(c, 2) + pow(d, 2) - pow(Lb, 2) + 2 * Lb * Lc - pow(Lc, 2))), 0.5)) / (pow(c, 2) + 2 * c * Lb + pow(d, 2) + pow(Lb, 2) - pow(Lc, 2)));
  *gamma = 2 * atan((2 * d * Lc + pow(- pow(c, 4) - 2 * pow(c, 2) * pow(d, 2) + 2 * pow(c, 2) * pow(Lb, 2) + 2 * pow(c, 2) * pow(Lc, 2) - pow(d, 4) + 2 * pow(d, 2) * pow(Lb, 2) + 2 * pow(d, 2) * pow(Lc, 2) - pow(Lb, 4) + 2 * pow(Lb, 2) * pow(Lc, 2) - pow(Lc, 4), 0.5)) / (pow(c, 2) + 2 * c * Lc + pow(d, 2) - pow(Lb, 2) + pow(Lc, 2))) - *beta;
  *delta = pitch - *beta - *gamma;
  *epsilon = rotation;
}

void point2jointV3(float x, float y, float z, float pitch, float rotation, float *alpha, float *beta, float *gamma, float *delta, float *epsilon) {
  float alpha1, beta1, beta2, gamma1, gamma2;
  float dist_ce, height, dist_be, dist_zx, dist;
  float new_xz, new_y, new_xy, dist_ac, delta1, delta2, delta3;

  if (x >= 0) {
    alpha1 = atan(z / x);
  } else if ( x < 0) {
    alpha1 = atan(z / x) + PI;
  } else if (x == 0) {
    alpha1 = 0;
  }
  dist_zx = sqrt(pow(x, 2) + pow(z, 2));
  height = y - (La); //-6.5/39.3701
  dist = (pow(x, 2) + pow(height, 2) + pow(z, 2));
  dist_be = sqrt(pow(dist_zx, 2) + pow(height, 2));
  new_xz=dist_zx-Ld*cos(pitch);
  new_y=height-Ld*sin(pitch);
  beta1 = atan(new_y / new_xz);
  dist_ac=sqrt(pow(new_y,2)+pow(new_xz,2));
  beta2 = acos((pow(Lb, 2) + pow(dist_ac, 2) - pow(Lc, 2)) / (2 * Lb * dist_ac));
  gamma1=acos((pow(Lb,2)+pow(Lc,2)-pow(dist_ac,2))/(2*Lb*Lc));
  delta1=acos((pow(Lc,2)+pow(dist_ac,2)-pow(Lb,2))/(2*Lc*dist_ac));
  delta2=atan2(new_xz,new_y);
  delta3=pitch+PI/2;
  *delta = -(PI-(delta1+delta2+delta3));
  *alpha = alpha1;
  *beta = beta1+beta2;
  *gamma = -(PI - gamma1);
  *epsilon = rotation;
}

void joint2point(float q1, float q2, float q3, float q4, float q5, float *x, float *y, float *z) { //All angles in radians
  float xy = Lb * cos(q2) + Lc * cos(q2 + q3) + Ld * cos(q2 + q3 + q4);
  float ax,ay,az;
  *x = xy * cos(q1);
  *z = xy * sin(q1);
  *y = La + Lb * sin(q2) + Lc * sin(q2 + q3) + Ld * sin(q2 + q3 + q4);
        /**x=ax*cos(tilt*PI/180)-ay*sin(tilt*PI/180);
        *y=-ax*sin(tilt*PI/180)+ay*cos(tilt*PI/180);
        *z=az;*/
}



float TAU1(float Q1, float Q2, float Q3, float Q4, float Q5, float U1, float U2, float U3, float U4, float U5, float Q1D, float Q2D, float Q3D, float Q4D, float Q5D) {

  float TAU = -1530 * (0.04 * sin(Q2) * cos(Q2) * U1 * U2 + 0.022 * sin(Q2 + Q3) * cos(Q2 + Q3) * U1 * (U2 +
                       U3) + 0.0025 * sin(Q2 + Q3 + Q4) * cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) + 6.666666666666667E-07 *
                       sin(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * (sin(0.003333333333333333 * Q5) * U5 * (
                             U2 + U3 + U4) + cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 + 300 * sin(0.003333333333333333 *
                                 Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) + 1.51526461696E-05 * (6.599507365296038 * sin(
                                       0.003333333333333333 * Q5) * cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) + MF * sin(
                                       Q5) * (41.23763820954416 * sin(Q2 + Q3 + Q4) + cos(Q5) * cos(Q2 + Q3 + Q4))) * (Q4DDD + KP * (Q4D -
                                           Q4) + 2 * pow(KP, 0.5) * (Q4DD - U4)) + (3.333333333333334E-07 * sin(Q2 + Q3 + Q4) - 1.51526461696E-05 *
                                               MF * (sin(Q2 + Q3 + Q4) + 42.94442845986271 * cos(Q5) * (sin(Q2) - 2.038873654640384 * cos(
                                                   Q2)) - 41.23763820954417 * cos(Q5) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q5) * (sin(
                                                       Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)))) * (Q5DDD + KP * (Q5D - Q5) + 2 * pow(KP, 0.5) * (
                                                           Q5DD - U5)) + 1.253402312E-05 * MC * (1259.333253024987 * sin(Q2) * pow(U2, 2) + 18.85905168970201 *
                                                               sin(Q2 + Q3) * pow((U2 + U3), 2) - 2567.621392005219 * cos(Q2) * pow(U2, 2) - 1718.746050087867 *
                                                               cos(Q2 + Q3) * pow((U2 + U3), 2) - 4459.042797080783 * (sin(Q2) - 2.038873654640384 * cos(
                                                                   Q2)) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 * U2 - 66.77606455220899 * (cos(Q2) +
                                                                       2.038873654640384 * sin(Q2)) * (sin(Q2 + Q3) - 91.13639849804267 * cos(Q2 + Q3)) * U1 * U2 -
                                                               66.77606455220899 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (cos(Q2 + Q3) + 91.13639849804267 *
                                                                   sin(Q2 + Q3)) * U1 * (U2 + U3) - (sin(Q2 + Q3) - 91.13639849804267 * cos(Q2 + Q3)) * (cos(Q2 + Q3) +
                                                                       91.13639849804267 * sin(Q2 + Q3)) * U1 * (U2 + U3)) + 0.0003225805080002 * MDD * (48.93200834437961 *
                                                                           sin(Q2) * pow(U2, 2) + 3.71745770278921 * sin(Q2 + Q3) * pow((U2 + U3), 2) - 99.76618268199903 *
                                                                           cos(Q2) * pow(U2, 2) - 80.84718011687438 * cos(Q2 + Q3) * pow((U2 + U3), 2) - 173.2582847555233 * (
                                                                               sin(Q2) - 2.038873654640384 * cos(Q2)) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 *
                                                                           U2 - 13.16276128916434 * (cos(Q2) + 2.038873654640384 * sin(Q2)) * (sin(Q2 + Q3) - 21.74797578899545 *
                                                                               cos(Q2 + Q3)) * U1 * U2 - 13.16276128916434 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (
                                                                                   cos(Q2 + Q3) + 21.74797578899545 * sin(Q2 + Q3)) * U1 * (U2 + U3) - (sin(Q2 + Q3) - 21.74797578899545 *
                                                                                       cos(Q2 + Q3)) * (cos(Q2 + Q3) + 21.74797578899545 * sin(Q2 + Q3)) * U1 * (U2 + U3)) + cos(Q2 + Q3) * (
                         4.213094973272125E-08 * sin(0.0001404364991090709 * Q4) * (U4 + 7120.656 * sin(Q2 + Q3) *
                             U1) * (sin(0.0001404364991090709 * Q4) * U2 + sin(0.0001404364991090709 * Q4) * U3 + cos(
                                      0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1) + 4.213094973272125E-08 * cos(0.0001404364991090709 *
                                          Q4) * (U4 + 7120.656 * sin(Q2 + Q3) * U1) * (cos(0.0001404364991090709 * Q4) * U2 + cos(
                                              0.0001404364991090709 * Q4) * U3 - sin(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1) -
                         4.213094973272125E-08 * sin(0.0001404364991090709 * Q4) * (sin(0.0001404364991090709 *
                             Q4) * U4 * (U2 + U3) + cos(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1 * U4 - 7120.656 * sin(
                               0.0001404364991090709 * Q4) * sin(Q2 + Q3) * U1 * (U2 + U3)) - 4.213094973272125E-08 * cos(
                           0.0001404364991090709 * Q4) * (cos(0.0001404364991090709 * Q4) * U4 * (U2 + U3) - sin(
                                 0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1 * U4 - 7120.656 * sin(Q2 + Q3) * cos(0.0001404364991090709 *
                                     Q4) * U1 * (U2 + U3))) + 0.02 * cos(Q2 + Q3 + Q4) * (sin(Q5) * (U5 + sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) *
                                         U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1) + cos(Q5) * (U5 + sin(Q2 + Q3 + Q4) *
                                             U1) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - sin(Q5) * (
                                                 sin(Q5) * U4 * U5 + sin(Q5) * U5 * (U2 + U3) + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 +
                                                     Q3 + Q4) * U1 * (U2 + U3 + U4)) - cos(Q5) * (cos(Q5) * U4 * U5 + cos(Q5) * U5 * (U2 + U3) - sin(Q5) * cos(
                                                         Q2 + Q3 + Q4) * U1 * U5 - cos(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4))) - 1.895892737972456E-08 *
                       sin(9.479463689862282E-05 * Q2) * cos(9.479463689862282E-05 * Q2) * U1 * U2 - 0.06935369115168 *
                       MCC * (sin(Q2) - 1.403842448495628 * cos(Q2)) * (cos(Q2) + 1.403842448495628 * sin(Q2)) *
                       U1 * U2 - 9.5394499218E-06 * MB * (sin(Q2) - 156.061118055651 * cos(Q2)) * (cos(Q2) +
                           156.061118055651 * sin(Q2)) * U1 * U2 - cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * (
                         3.333333333333334E-07 * U5 - 0.0001 * sin(Q2 + Q3 + Q4) * U1) * (cos(0.003333333333333333 *
                             Q5) * U2 + cos(0.003333333333333333 * Q5) * U3 + cos(0.003333333333333333 * Q5) * U4 - sin(
                               0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1) - 3.333333333333334E-07 * cos(0.003333333333333333 *
                                   Q5) * cos(Q2 + Q3 + Q4) * (sin(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - cos(
                                         0.003333333333333333 * Q5) * U5 * (U2 + U3 + U4) - 300 * cos(0.003333333333333333 * Q5) * sin(
                                         Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - 0.0001 * sin(Q2 + Q3 + Q4) * (sin(0.003333333333333333 * Q5) *
                                             U2 + sin(0.003333333333333333 * Q5) * U3 + sin(0.003333333333333333 * Q5) * U4 + cos(
                                                 0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1) * (cos(0.003333333333333333 * Q5) * U2 +
                                                     cos(0.003333333333333333 * Q5) * U3 + cos(0.003333333333333333 * Q5) * U4 - sin(0.003333333333333333 *
                                                         Q5) * cos(Q2 + Q3 + Q4) * U1) - 1.51526461696E-05 * (15.5999016445218 * MC * (cos(Q2 + Q3) +
                                                             91.13639849804267 * sin(Q2 + Q3)) + 79.13993244565121 * MDD * (cos(Q2 + Q3) + 21.74797578899545 *
                                                                 sin(Q2 + Q3)) - 6.599507365296038 * sin(0.003333333333333333 * Q5) * cos(0.003333333333333333 *
                                                                     Q5) * cos(Q2 + Q3 + Q4) - MF * sin(Q5) * (cos(Q5) * cos(Q2 + Q3 + Q4) - 91.35188458218588 * sin(
                                                                         Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 *
                                                                         sin(Q2 + Q3 + Q4) * (-25.27925669291339 + sin(Q4) - 56.00000000000001 * cos(Q4)))) * (
                         Q3DDD + KP * (Q3D - Q3) + 2 * pow(KP, 0.5) * (Q3DD - U3)) - cos(Q2) * (1.404364991090709E-07 *
                             cos(0.0001404364991090709 * Q3) * (cos(0.0001404364991090709 * Q3) * U2 * U3 - 7120.656 *
                                 sin(Q2) * cos(0.0001404364991090709 * Q3) * U1 * U2 - cos(Q2) * sin(0.0001404364991090709 *
                                     Q3) * U1 * U3) - 1.404364991090709E-07 * sin(0.0001404364991090709 * Q3) * (U3 + 7120.656 *
                                         sin(Q2) * U1) * (sin(0.0001404364991090709 * Q3) * U2 + cos(Q2) * cos(0.0001404364991090709 *
                                             Q3) * U1) - 1.404364991090709E-07 * cos(0.0001404364991090709 * Q3) * (U3 + 7120.656 *
                                                 sin(Q2) * U1) * (cos(0.0001404364991090709 * Q3) * U2 - cos(Q2) * sin(0.0001404364991090709 *
                                                     Q3) * U1) - 1.404364991090709E-07 * sin(0.0001404364991090709 * Q3) * (7120.656 * sin(
                                                         Q2) * sin(0.0001404364991090709 * Q3) * U1 * U2 - sin(0.0001404364991090709 * Q3) * U2 * U3 -
                                                         cos(Q2) * cos(0.0001404364991090709 * Q3) * U1 * U3)) - 1.51526461696E-05 * (15.5999016445218 *
                                                             MC * (cos(Q2 + Q3) + 66.77606455220899 * cos(Q2) + 136.1479587760645 * sin(Q2) + 91.13639849804267 *
                                                                 sin(Q2 + Q3)) + 79.13993244565121 * MDD * (cos(Q2 + Q3) + 13.16276128916434 * cos(Q2) +
                                                                     26.83720721479747 * sin(Q2) + 21.74797578899545 * sin(Q2 + Q3)) - 6.599507365296038 *
                                                             sin(0.003333333333333333 * Q5) * cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) - MF *
                                                             sin(Q5) * (cos(Q5) * cos(Q2 + Q3 + Q4) + 42.94442845986271 * cos(Q3 + Q4) * cos(Q2 + Q3 + Q4) -
                                                                 91.35188458218588 * sin(Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q4) * cos(Q2 + Q3 +
                                                                     Q4) - 87.55826380040281 * sin(Q3 + Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * sin(Q2 + Q3 +
                                                                         Q4) * (-25.27925669291339 + sin(Q4) - 56.00000000000001 * cos(Q4) - 53.67445669291338 *
                                                                             cos(Q3 + Q4) - 26.32554330708661 * sin(Q3 + Q4)))) * (Q2DDD + KP * (Q2D - Q2) + 2 * pow(KP, 0.5) * (
                                                                                 Q2DD - U2)) - 8.0645E-05 * MFF * U1 * (187.9794283027094 * sin(Q2 + Q3 + Q4) * (cos(Q2) +
                                                                                     2.038873654640384 * sin(Q2)) * U2 + 50.98776353938621 * sin(Q2 + Q3 + Q4) * cos(Q2 + Q3 + Q4) * (
                                                                                         U2 + U3 + U4) + 693.0342304132928 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (cos(Q2) +
                                                                                             2.038873654640384 * sin(Q2)) * U2 + 187.9794283027094 * cos(Q2 + Q3 + Q4) * (sin(Q2) -
                                                                                                 2.038873654640384 * cos(Q2)) * (U2 + U3 + U4) + 7.140571653543307 * sin(Q2 + Q3 + Q4) * (
                                                                                                     56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * (U2 + U3) + 26.32554330708662 * (sin(Q2) -
                                                                                                         2.038873654640384 * cos(Q2)) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * (U2 + U3) -
                                                                                     26.32554330708662 * (cos(Q2) + 2.038873654640384 * sin(Q2)) * (sin(Q2 + Q3) + 56.00000000000001 *
                                                                                         cos(Q2 + Q3)) * U2 - 7.140571653543307 * cos(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 56.00000000000001 *
                                                                                             cos(Q2 + Q3)) * (U2 + U3 + U4) - (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) * (56.00000000000001 *
                                                                                                 sin(Q2 + Q3) - cos(Q2 + Q3)) * (U2 + U3)) - 7.10715724322E-05 * MD * U1 * (786.3867872769667 * (
                                                                                                     sin(Q2) - 2.038873654640384 * cos(Q2)) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U2 +
                                                                                                     28.04258881196539 * (cos(Q2) + 2.038873654640384 * sin(Q2)) * (cos(Q2 + Q3 + Q4) + 2.039898074042263 *
                                                                                                         sin(Q2 + Q3 + Q4)) * U2 + 29.87162612766583 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (
                                                                                                             56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * (U2 + U3) + 1.065223554357435 * (sin(Q2 +
                                                                                                                 Q3) + 56.00000000000001 * cos(Q2 + Q3)) * (sin(Q2 + Q3 + Q4) - 2.039898074042263 * cos(Q2 +
                                                                                                                     Q3 + Q4)) * (U2 + U3 + U4) + 1.065223554357435 * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 +
                                                                                                                         Q3)) * (cos(Q2 + Q3 + Q4) + 2.039898074042263 * sin(Q2 + Q3 + Q4)) * (U2 + U3) - 29.87162612766583 * (
                                                                                                                             cos(Q2) + 2.038873654640384 * sin(Q2)) * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) *
                                                                                                     U2 - 28.04258881196539 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (sin(Q2 + Q3 + Q4) -
                                                                                                         2.039898074042263 * cos(Q2 + Q3 + Q4)) * (U2 + U3 + U4) - 1.134701220757888 * (sin(Q2 + Q3) +
                                                                                                             56.00000000000001 * cos(Q2 + Q3)) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * (U2 +
                                                                                                                 U3) - (sin(Q2 + Q3 + Q4) - 2.039898074042263 * cos(Q2 + Q3 + Q4)) * (cos(Q2 + Q3 + Q4) + 2.039898074042263 *
                                                                                                                     sin(Q2 + Q3 + Q4)) * (U2 + U3 + U4)) - (0.03180000004271862 + 0.009079839153476099 * MBB +
                                                                                                                         0.021 * pow(cos(Q2), 2) + 0.0001 * pow(sin(9.479463689862282E-05 * Q2), 2) + 0.0001 *
                                                                                                                         pow(sin(0.003333333333333333 * Q5), 2) + 0.0113 * pow(cos(Q2 + Q3), 2) + 4.7697249609E-06 *
                                                                                                                         MB * pow((sin(Q2) - 156.061118055651 * cos(Q2)), 2) + 0.03467684557584 * MCC * pow((sin(
                                                                                                                             Q2) - 1.403842448495628 * cos(Q2)), 2) + 0.0001612902540001 * MDD * (55.27796708810733 +
                                                                                                                                 173.2582847555233 * pow((sin(Q2) - 2.038873654640384 * cos(Q2)), 2) + pow((sin(Q2 + Q3) -
                                                                                                                                     21.74797578899545 * cos(Q2 + Q3)), 2) + 26.32552257832868 * (sin(Q2) - 2.038873654640384 *
                                                                                                                                         cos(Q2)) * (sin(Q2 + Q3) - 21.74797578899545 * cos(Q2 + Q3))) + MC * (0.00891579735225 +
                                                                                                                                             0.02794487275584 * pow((sin(Q2) - 2.038873654640384 * cos(Q2)), 2) + 6.267011559999999E-06 *
                                                                                                                                             pow((sin(Q2 + Q3) - 91.13639849804267 * cos(Q2 + Q3)), 2) + 0.0008369727369599999 * (sin(
                                                                                                                                                 Q2) - 2.038873654640384 * cos(Q2)) * (sin(Q2 + Q3) - 91.13639849804267 * cos(Q2 + Q3))) +
                                                                                                                         4.03225E-05 * MFF * (50.98776353938621 * pow(sin(Q2 + Q3 + Q4), 2) + 693.0342304132928 *
                                                                                                                             pow((sin(Q2) - 2.038873654640384 * cos(Q2)), 2) + pow((sin(Q2 + Q3) + 56.00000000000001 *
                                                                                                                                 cos(Q2 + Q3)), 2) + 375.9588566054188 * sin(Q2 + Q3 + Q4) * (sin(Q2) - 2.038873654640384 *
                                                                                                                                     cos(Q2)) - 14.28114330708661 * sin(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 56.00000000000001 * cos(
                                                                                                                                         Q2 + Q3)) - 52.65108661417323 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (sin(Q2 + Q3) +
                                                                                                                                             56.00000000000001 * cos(Q2 + Q3))) + 3.55357862161E-05 * MD * (786.3867872769667 * pow((
                                                                                                                                                 sin(Q2) - 2.038873654640384 * cos(Q2)), 2) + 1.134701220757888 * pow((sin(Q2 + Q3) +
                                                                                                                                                     56.00000000000001 * cos(Q2 + Q3)), 2) + pow((cos(Q2 + Q3 + Q4) + 2.039898074042263 * sin(
                                                                                                                                                         Q2 + Q3 + Q4)), 2) + 56.08517762393078 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (cos(Q2 +
                                                                                                                                                             Q3 + Q4) + 2.039898074042263 * sin(Q2 + Q3 + Q4)) - 59.74325225533166 * (sin(Q2) - 2.038873654640384 *
                                                                                                                                                                 cos(Q2)) * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) - 2.130447108714871 * (sin(
                                                                                                                                                                     Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) * (cos(Q2 + Q3 + Q4) + 2.039898074042263 * sin(
                                                                                                                                                                         Q2 + Q3 + Q4))) - 0.0001 * pow(sin(Q2 + Q3 + Q4), 2) * (213 + pow(sin(0.003333333333333333 *
                                                                                                                                                                             Q5), 2)) - 1.51526461696E-05 * MF * (41.23763820954417 * cos(Q5) * sin(Q2 + Q3 + Q4) * cos(
                                                                                                                                                                                 Q2 + Q3 + Q4) + 3541.853607886941 * cos(Q2 + Q3 + Q4) * (sin(Q2) - 2.038873654640384 * cos(Q2)) +
                                                                                                                                                                                 1.631283653253319 * cos(Q5) * sin(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 56.00000000000001 * cos(
                                                                                                                                                                                     Q2 + Q3)) + 140.1090882897613 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (sin(Q2 + Q3) +
                                                                                                                                                                                         56.00000000000001 * cos(Q2 + Q3)) - 1700.542805101257 * pow(cos(Q2 + Q3 + Q4), 2) - 1844.223935744266 *
                                                                                                                                                                                 pow((sin(Q2) - 2.038873654640384 * cos(Q2)), 2) - pow(sin(Q5), 2) * pow(cos(Q2 + Q3 + Q4),
                                                                                                                                                                                     2) - 2.661086357371495 * pow((sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)), 2) -
                                                                                                                                                                                 134.5405702200077 * cos(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) -
                                                                                                                                                                                 42.94442845986271 * cos(Q5) * sin(Q2 + Q3 + Q4) * (sin(Q2) - 2.038873654640384 * cos(Q2)) -
                                                                                                                                                                                 sin(Q2 + Q3 + Q4) * (sin(Q2 + Q3 + Q4) + 42.94442845986271 * cos(Q5) * (sin(Q2) - 2.038873654640384 *
                                                                                                                                                                                     cos(Q2)) - 41.23763820954417 * cos(Q5) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q5) * (
                                                                                                                                                                                         sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3))))) * (Q1DDD + KP * (Q1D - Q1) + 2 * pow(KP, 0.5) * (
                                                                                                                                                                                             Q1DD - U1)) - 1.51526461696E-05 * MF * (1.631283653253319 * sin(Q4) * sin(Q5) * cos(Q2 + Q3 +
                                                                                                                                                                                                 Q4) * pow((U2 + U3), 2) + 3688.447871488533 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (
                                                                                                                                                                                                     cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 * U2 + 1539.668479562462 * sin(Q2 + Q3 + Q4) * (
                                                                                                                                                                                                         sin(Q2) - 2.038873654640384 * cos(Q2)) * U1 * (U2 + U3 + 1.650200077127465 * U4) + 140.1090882897613 * (
                                                                                                                                                                                                             sin(Q2) - 2.038873654640384 * cos(Q2)) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) *
                                                                                                                                                                                                 U1 * (U2 + U3) + 1.631283653253319 * sin(Q5) * (26.32554330708661 * sin(Q2) - 53.6744566929134 *
                                                                                                                                                                                                     cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 +
                                                                                                                                                                                                         Q4)) * pow(U1, 2) + 42.94442845986271 * (sin(Q2) - 2.038873654640384 * cos(Q2)) * U1 * (
                                                                                                                                                                                                             23.3113491101155 * sin(Q2 + Q3 + Q4) * (U2 + U3 + U4) + cos(Q2 + Q3 + Q4) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                                 U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 41.23763820954417 * cos(Q5) * cos(Q2 +
                                                                                                                                                                                                                     Q3 + Q4) * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(
                                                                                                                                                                                                                         Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) *
                                                                                                                                                                                                                             U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) *
                                                                                                                                                                                                                                 U1)) + 42.94442845986271 * sin(Q5) * (sin(Q2) - 2.038873654640384 * cos(Q2)) * ((U5 + sin(
                                                                                                                                                                                                                                     Q2 + Q3 + Q4) * U1) * (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 -
                                                                                                                                                                                                                                         sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 * sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(
                                                                                                                                                                                                                                             Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 1.631283653253319 *
                                                                                                                                                                                                 cos(Q5) * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) * (23.3113491101155 * cos(Q5) *
                                                                                                                                                                                                     U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                                                                                                                                                                     cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                         U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + sin(Q5) * cos(Q2 + Q3 + Q4) * (sin(Q5) * U2 *
                                                                                                                                                                                                             U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 +
                                                                                                                                                                                                                 Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                                     U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 *
                                                                                                                                                                                                                         sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) *
                                                                                                                                                                                                                             cos(Q2 + Q3 + Q4) * U1)) - 87.55826380040281 * sin(Q5) * cos(Q3 + Q4) * cos(Q2 + Q3 + Q4) * pow(
                                                                                                                                                                                                                                 U2, 2) - 42.94442845986271 * sin(Q5) * sin(Q3 + Q4) * cos(Q2 + Q3 + Q4) * pow(U2, 2) - 3541.853607886941 *
                                                                                                                                                                                                 cos(Q2 + Q3 + Q4) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 * U2 - 91.35188458218588 *
                                                                                                                                                                                                 sin(Q5) * cos(Q4) * cos(Q2 + Q3 + Q4) * pow((U2 + U3), 2) - 1478.475648643869 * sin(Q2 + Q3 + Q4) *
                                                                                                                                                                                                 cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + 1.650200077127465 * U4) - 17.92628909942867 * sin(Q5) * cos(
                                                                                                                                                                                                     Q2 + Q3 + Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) - 140.1090882897613 * (cos(Q2) +
                                                                                                                                                                                                         2.038873654640384 * sin(Q2)) * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) * U1 * U2 -
                                                                                                                                                                                                 134.5405702200077 * cos(Q2 + Q3 + Q4) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) *
                                                                                                                                                                                                 U1 * (U2 + U3) - 58.48572474278229 * sin(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 56.00000000000001 *
                                                                                                                                                                                                     cos(Q2 + Q3)) * U1 * (U2 + U3 + 1.650200077127465 * U4) - 5.322172714742989 * (sin(Q2 + Q3) +
                                                                                                                                                                                                         56.00000000000001 * cos(Q2 + Q3)) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * U1 * (
                                                                                                                                                                                                             U2 + U3) - 41.23763820954417 * cos(Q2 + Q3 + Q4) * U1 * (23.3113491101155 * sin(Q2 + Q3 + Q4) * (
                                                                                                                                                                                                                 U2 + U3 + U4) + cos(Q2 + Q3 + Q4) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 +
                                                                                                                                                                                                                     Q4) * U1)) - 1.631283653253319 * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3)) * U1 * (
                                                                                                                                                                                                                         23.3113491101155 * sin(Q2 + Q3 + Q4) * (U2 + U3 + U4) + cos(Q2 + Q3 + Q4) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                                             U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 41.23763820954417 * sin(Q5) * cos(Q2 +
                                                                                                                                                                                                                                 Q3 + Q4) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 *
                                                                                                                                                                                                                                     sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 * sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (
                                                                                                                                                                                                                                         U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 42.94442845986271 *
                                                                                                                                                                                                 cos(Q5) * (sin(Q2) - 2.038873654640384 * cos(Q2)) * (23.3113491101155 * cos(Q5) * U2 * U5 +
                                                                                                                                                                                                     23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                                                                                                                                                                     cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                         U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 1.631283653253319 * sin(Q5) * (sin(Q2 +
                                                                                                                                                                                                             Q3) + 56.00000000000001 * cos(Q2 + Q3)) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 *
                                                                                                                                                                                                                 sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                                                                                                                                                                                                 sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                                                                                                                                                                                     cos(Q2 + Q3 + Q4) * U1)) - sin(Q2 + Q3 + Q4) * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 *
                                                                                                                                                                                                                         cos(Q5) * U3 * U5 + 87.5582638004028 * sin(Q5) * sin(Q3 + Q4) * pow(U2, 2) + 1.631283653253319 *
                                                                                                                                                                                                                         sin(Q5) * cos(Q4) * pow((U2 + U3), 2) + 91.35188458218588 * sin(Q4) * sin(Q5) * pow((U2 + U3),
                                                                                                                                                                                                                             2) - 42.94442845986271 * sin(Q5) * cos(Q3 + Q4) * pow(U2, 2) - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 +
                                                                                                                                                                                                                                 U4) - 85.88885691972541 * cos(Q5) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 * U2 -
                                                                                                                                                                                                                         35.85257819885733 * cos(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + 1.650200077127465 * U4) -
                                                                                                                                                                                                                         23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - 3.262567306506638 *
                                                                                                                                                                                                                         cos(Q5) * (56.00000000000001 * sin(Q2 + Q3) - cos(Q2 + Q3)) * U1 * (U2 + U3) - sin(Q5) * U4 * (
                                                                                                                                                                                                                             cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (
                                                                                                                                                                                                                                 cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - cos(Q5) * U1 * (
                                                                                                                                                                                                                                     23.3113491101155 * sin(Q2 + Q3 + Q4) * (U2 + U3 + U4) + cos(Q2 + Q3 + Q4) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                                                                                                                                                         U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1))))) / cos(Q1);
  return TAU;
}

float TAU2(float Q1, float Q2, float Q3, float Q4, float Q5, float U1, float U2, float U3, float U4, float U5, float Q1D, float Q2D, float Q3D, float Q4D, float Q5D) {
  float TAU = 210.9824 * sin(Q2) * pow(U1, 2) - 10549.12 * (0.00218397 * GRAV * (MB * (sin(Q2) -
              156.061118055651 * cos(Q2)) + 85.26545694308989 * MCC * (sin(Q2) - 1.403842448495628 *
                  cos(Q2)) + 1.146261166591116 * MC * (sin(Q2 + Q3) + 66.77606455220899 * sin(Q2) - 136.1479587760646 *
                      cos(Q2) - 91.13639849804267 * cos(Q2 + Q3)) + 5.815102771558218 * MDD * (sin(Q2 + Q3) +
                          13.16276128916434 * sin(Q2) - 26.83720721479747 * cos(Q2) - 21.74797578899545 * cos(
                            Q2 + Q3)) - 2.907549096370371 * MFF * (sin(Q2 + Q3) + 53.67445669291339 * cos(Q2) + 56.00000000000001 *
                                cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 * sin(Q2 + Q3 + Q4)) -
              2.729520094140487 * MD * (57.17529553662943 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) +
                                        59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(
                                          Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) - 1.782368805432309 * MF * (87.55826380040281 * cos(Q2) +
                                              1.631283653253319 * sin(Q2 + Q3) + 91.35188458218588 * cos(Q2 + Q3) + 41.23763820954416 *
                                              cos(Q2 + Q3 + Q4) - 42.94442845986271 * sin(Q2) - cos(Q5) * sin(Q2 + Q3 + Q4))) + 9.479463689862282E-09 *
              sin(9.479463689862282E-05 * Q2) * cos(9.479463689862282E-05 * Q2) * pow(U1, 2) +
              0.03467684557584 * MCC * (sin(Q2) - 1.403842448495628 * cos(Q2)) * (cos(Q2) + 1.403842448495628 *
                  sin(Q2)) * pow(U1, 2) + 1.404364991090709E-07 * sin(0.0001404364991090709 * Q3) * (U3 +
                      7120.656 * sin(Q2) * U1) * (cos(0.0001404364991090709 * Q3) * U2 - cos(Q2) * sin(0.0001404364991090709 *
                          Q3) * U1) + 4.213094973272125E-08 * sin(0.0001404364991090709 * Q4) * (U4 + 7120.656 *
                              sin(Q2 + Q3) * U1) * (cos(0.0001404364991090709 * Q4) * U2 + cos(0.0001404364991090709 *
                                  Q4) * U3 - sin(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1) + 0.02 * sin(Q5) * (U5 + sin(Q2 +
                                      Q3 + Q4) * U1) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) +
              4.213094973272125E-08 * cos(0.0001404364991090709 * Q4) * (sin(0.0001404364991090709 *
                  Q4) * U4 * (U2 + U3) + cos(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1 * U4 - 7120.656 * sin(
                    0.0001404364991090709 * Q4) * sin(Q2 + Q3) * U1 * (U2 + U3)) + 0.02 * cos(Q5) * (sin(Q5) * U4 *
                        U5 + sin(Q5) * U5 * (U2 + U3) + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (
                          U2 + U3 + U4)) + 1.51526461696E-05 * (-1458.491127730424 + 6.599507365296038 * pow(sin(
                                0.003333333333333333 * Q5), 2) + 19.00167781107771 * MFF * (-7.140571653543308 + cos(
                                      Q4) + 56.00000000000001 * sin(Q4) + 53.67445669291339 * sin(Q3 + Q4) - 26.32554330708661 *
                                    cos(Q3 + Q4)) + 65995.07365296038 * MD * (-0.0001834067366645 + 0.002197016561 * cos(Q4) +
                                        0.0042863206755 * sin(Q4) + 0.005141117272184 * sin(Q3 + Q4) - 1.020849752000116E-06 *
                                        cos(Q3 + Q4)) + MF * (-1700.542805101257 + 67.27028511000385 * sin(Q4) + 1.631283653253319 *
                                            cos(Q4) * cos(Q5) + 91.35188458218588 * sin(Q4) * cos(Q5) + 87.55826380040281 * cos(Q5) *
                                            sin(Q3 + Q4) - 3767.135966160216 * cos(Q4) - 3610.696004856838 * cos(Q3 + Q4) - 1770.92680394347 *
                                            sin(Q3 + Q4) - pow(cos(Q5), 2) - 42.94442845986271 * cos(Q5) * cos(Q3 + Q4))) * (Q4DDD + KP * (
                                                Q4D - Q4) + 2 * pow(KP, 0.5) * (Q4DD - U4)) + MFF * (0.0616089446 * cos(Q3) * pow(U2, 2) + 0.0002879257005 *
                                                    sin(Q4) * pow((U2 + U3), 2) + 0.12013863196 * sin(Q3) * pow((U2 + U3), 2) + 0.016123839228 *
                                                    cos(Q4) * pow((U2 + U3 + U4), 2) + 0.007579800497735999 * sin(Q3 + Q4) * pow((U2 + U3 + U4), 2) +
                                                    0.015454255542264 * cos(Q3 + Q4) * pow((U2 + U3 + U4), 2) + 4.03225E-05 * cos(Q2 + Q3) * (sin(
                                                        Q2 + Q3) + 53.6744566929134 * cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 *
                                                        sin(Q2) - 7.140571653543308 * sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 1.387778780781446E-17 *
                                                    pow(U2, 2) - 0.12013863196 * sin(Q3) * pow(U2, 2) - 0.015454255542264 * cos(Q3 + Q4) * pow(
                                                        U2, 2) - 0.007579800497735999 * sin(Q3 + Q4) * pow(U2, 2) - 0.06160894460000001 * cos(Q3) *
                                                    pow((U2 + U3), 2) - 0.016123839228 * cos(Q4) * pow((U2 + U3), 2) - 0.0002879257005 * sin(Q4) *
                                                    pow((U2 + U3 + U4), 2) - 0.00216428828 * sin(Q2) * (sin(Q2 + Q3) + 53.6744566929134 * cos(Q2) +
                                                        56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                                        sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.00106151172 * cos(Q2) * (sin(Q2 + Q3) + 53.6744566929134 *
                                                            cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                                            sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.00225806 * sin(Q2 + Q3) * (sin(Q2 + Q3) + 53.6744566929134 *
                                                                cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                                                sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.0002879257005 * cos(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 53.6744566929134 *
                                                                    cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                                                    sin(Q2 + Q3 + Q4)) * pow(U1, 2)) + MD * (0.0616089446 * cos(Q3) * pow(U2, 2) + 0.002197016561 *
                                                                        sin(Q4) * pow((U2 + U3), 2) + 0.12013863196 * sin(Q3) * pow((U2 + U3), 2) + 0.0042863206755 *
                                                                        cos(Q4) * pow((U2 + U3 + U4), 2) + 1.020849752000116E-06 * sin(Q3 + Q4) * pow((U2 + U3 + U4), 2) +
                                                                        0.005141117272184 * cos(Q3 + Q4) * pow((U2 + U3 + U4), 2) + 3.78535565E-05 * cos(Q2 + Q3) * (
                                                                            57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 *
                                                                            cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 +
                                                                                Q3 + Q4)) * pow(U1, 2) + 3.55357862161E-05 * sin(Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) +
                                                                                    1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 *
                                                                                    sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 1.387778780781446E-17 *
                                                                        pow(U2, 2) - 0.12013863196 * sin(Q3) * pow(U2, 2) - 0.005141117272184 * cos(Q3 + Q4) * pow(
                                                                            U2, 2) - 1.020849752000116E-06 * sin(Q3 + Q4) * pow(U2, 2) - 0.06160894460000001 * cos(Q3) *
                                                                        pow((U2 + U3), 2) - 0.0042863206755 * cos(Q4) * pow((U2 + U3), 2) - 0.002197016561 * sin(Q4) *
                                                                        pow((U2 + U3 + U4), 2) - 0.002031769079032 * sin(Q2) * (57.17529553662944 * cos(Q2) +
                                                                            1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 *
                                                                            sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.0009965154409679999 *
                                                                        cos(Q2) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 *
                                                                            cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 +
                                                                                Q3 + Q4)) * pow(U1, 2) - 0.002119799164 * sin(Q2 + Q3) * (57.17529553662944 * cos(Q2) +
                                                                                    1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 *
                                                                                    sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 7.24893818618E-05 *
                                                                        cos(Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) +
                                                                            59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(
                                                                                Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2)) - 0.011 * sin(Q2 + Q3) * cos(Q2 + Q3) * pow(U1, 2) -
              0.0013 * sin(Q2 + Q3 + Q4) * cos(Q2 + Q3 + Q4) * pow(U1, 2) - MB * (sin(Q2) - 156.061118055651 *
                  cos(Q2)) * (-0.0007443686102159999 * sin(Q2) - 4.7697249609E-06 * cos(Q2)) * pow(U1, 2) -
              1.404364991090709E-07 * cos(0.0001404364991090709 * Q3) * (U3 + 7120.656 * sin(Q2) * U1) * (
                sin(0.0001404364991090709 * Q3) * U2 + cos(Q2) * cos(0.0001404364991090709 * Q3) * U1) -
              1.404364991090709E-07 * sin(0.0001404364991090709 * Q3) * (cos(0.0001404364991090709 *
                  Q3) * U2 * U3 - 7120.656 * sin(Q2) * cos(0.0001404364991090709 * Q3) * U1 * U2 - cos(Q2) * sin(
                    0.0001404364991090709 * Q3) * U1 * U3) - 1.404364991090709E-07 * cos(0.0001404364991090709 *
                        Q3) * (7120.656 * sin(Q2) * sin(0.0001404364991090709 * Q3) * U1 * U2 - sin(0.0001404364991090709 *
                               Q3) * U2 * U3 - cos(Q2) * cos(0.0001404364991090709 * Q3) * U1 * U3) - 4.213094973272125E-08 *
              cos(0.0001404364991090709 * Q4) * (U4 + 7120.656 * sin(Q2 + Q3) * U1) * (sin(0.0001404364991090709 *
                  Q4) * U2 + sin(0.0001404364991090709 * Q4) * U3 + cos(0.0001404364991090709 * Q4) * cos(
                    Q2 + Q3) * U1) - 0.02 * cos(Q5) * (U5 + sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) *
                        U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1) - 4.213094973272125E-08 * sin(0.0001404364991090709 *
                            Q4) * (cos(0.0001404364991090709 * Q4) * U4 * (U2 + U3) - sin(0.0001404364991090709 * Q4) *
                                   cos(Q2 + Q3) * U1 * U4 - 7120.656 * sin(Q2 + Q3) * cos(0.0001404364991090709 * Q4) * U1 * (U2 +
                                       U3)) - 2.4718264E-05 * MF * sin(Q5) * (-25.27925669291339 + sin(Q4) - 56 * cos(Q4) - 53.67445669291339 *
                                           cos(Q3 + Q4) - 26.32554330708661 * sin(Q3 + Q4)) * (Q5DDD + KP * (Q5D - Q5) + 2 * pow(KP, 0.5) * (
                                               Q5DD - U5)) - 6.666666666666667E-07 * cos(0.003333333333333333 * Q5) * (sin(0.003333333333333333 *
                                                   Q5) * U5 * (U2 + U3 + U4) + cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 + 300 * sin(
                                                       0.003333333333333333 * Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - sin(0.003333333333333333 *
                                                           Q5) * (3.333333333333334E-07 * U5 - 0.0001 * sin(Q2 + Q3 + Q4) * U1) * (cos(0.003333333333333333 *
                                                               Q5) * U2 + cos(0.003333333333333333 * Q5) * U3 + cos(0.003333333333333333 * Q5) * U4 - sin(
                                                                   0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1) - 0.02 * sin(Q5) * (cos(Q5) * U4 * U5 + cos(
                                                                       Q5) * U5 * (U2 + U3) - sin(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - cos(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 +
                                                                           U4)) - 3.333333333333334E-07 * sin(0.003333333333333333 * Q5) * (sin(0.003333333333333333 *
                                                                               Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - cos(0.003333333333333333 * Q5) * U5 * (U2 + U3 + U4) - 300 * cos(
                                                                                   0.003333333333333333 * Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - MC * (1.387778780781446E-17 *
                                                                                       pow(U2, 2) + 0.07817978280468801 * sin(Q3) * pow(U2, 2) + 0.037286099612272 * cos(Q3) *
                                                                                       pow((U2 + U3), 2) - 0.037286099612272 * cos(Q3) * pow(U2, 2) - 0.078179782804688 * sin(Q3) *
                                                                                       pow((U2 + U3), 2) - 0.0008532408315199999 * sin(Q2) * (sin(Q2 + Q3) + 66.77606455220899 *
                                                                                           sin(Q2) - 136.1479587760646 * cos(Q2) - 91.13639849804267 * cos(Q2 + Q3)) * pow(U1, 2) -
                                                                                       0.00041848636848 * cos(Q2) * (sin(Q2 + Q3) + 66.77606455220899 * sin(Q2) - 136.1479587760646 *
                                                                                           cos(Q2) - 91.13639849804267 * cos(Q2 + Q3)) * pow(U1, 2) - 0.000571152862924 * sin(Q2 + Q3) * (
                                                                                               sin(Q2 + Q3) + 66.77606455220899 * sin(Q2) - 136.1479587760646 * cos(Q2) - 91.13639849804267 *
                                                                                               cos(Q2 + Q3)) * pow(U1, 2) - 6.267011559999999E-06 * cos(Q2 + Q3) * (sin(Q2 + Q3) + 66.77606455220899 *
                                                                                                   sin(Q2) - 136.1479587760646 * cos(Q2) - 91.13639849804267 * cos(Q2 + Q3)) * pow(U1, 2)) -
              MDD * (1.387778780781446E-17 * pow(U2, 2) + 0.09626087746360003 * sin(Q3) * pow(U2, 2) +
                     0.04184291875974401 * cos(Q3) * pow((U2 + U3), 2) - 0.041842918759744 * cos(Q3) * pow(U2,
                         2) - 0.09626087746360001 * sin(Q3) * pow((U2 + U3), 2) - 0.004328579968327999 * sin(Q2) * (
                       sin(Q2 + Q3) + 13.16276128916434 * sin(Q2) - 26.83720721479747 * cos(Q2) - 21.74797578899545 *
                       cos(Q2 + Q3)) * pow(U1, 2) - 0.002123025111672 * cos(Q2) * (sin(Q2 + Q3) + 13.16276128916434 *
                           sin(Q2) - 26.83720721479747 * cos(Q2) - 21.74797578899545 * cos(Q2 + Q3)) * pow(U1, 2) -
                     0.0035077365389951 * sin(Q2 + Q3) * (sin(Q2 + Q3) + 13.16276128916434 * sin(Q2) - 26.83720721479747 *
                         cos(Q2) - 21.74797578899545 * cos(Q2 + Q3)) * pow(U1, 2) - 0.0001612902540001 * cos(Q2 +
                             Q3) * (sin(Q2 + Q3) + 13.16276128916434 * sin(Q2) - 26.83720721479747 * cos(Q2) - 21.74797578899545 *
                                    cos(Q2 + Q3)) * pow(U1, 2)) - 1.51526461696E-05 * (15.5999016445218 * MC * (cos(Q2 + Q3) +
                                        66.77606455220901 * cos(Q2) + 136.1479587760645 * sin(Q2) + 91.13639849804268 * sin(
                                          Q2 + Q3)) + 79.13993244565121 * MDD * (cos(Q2 + Q3) + 13.16276128916434 * cos(Q2) + 26.83720721479747 *
                                              sin(Q2) + 21.74797578899545 * sin(Q2 + Q3)) - 6.599507365296038 * sin(0.003333333333333333 *
                                                  Q5) * cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) - MF * sin(Q5) * (cos(Q5) * cos(Q2 +
                                                      Q3 + Q4) + 42.94442845986271 * cos(Q3 + Q4) * cos(Q2 + Q3 + Q4) - 91.35188458218588 * sin(Q4) *
                                                      cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q4) * cos(Q2 + Q3 + Q4) - 87.55826380040281 * sin(
                                                          Q3 + Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * sin(Q2 + Q3 + Q4) * (-25.27925669291339 +
                                                              sin(Q4) - 56 * cos(Q4) - 53.67445669291339 * cos(Q3 + Q4) - 26.32554330708661 * sin(Q3 + Q4)))) * (
                Q1DDD + KP * (Q1D - Q1) + 2 * pow(KP, 0.5) * (Q1DD - U1)) - 4.9436528E-05 * (675.6137890589727 +
                    754.2216478526161 * MC * (1.396206159175881 + sin(Q3) + 2.096754115277765 * cos(Q3)) +
                    846.3967930705815 * MDD * (1.827010682911259 + sin(Q3) + 2.300529702918576 * cos(Q3)) -
                    2.022795775625667 * pow(sin(0.003333333333333333 * Q5), 2) - 11.64829781330922 * MFF * (
                      -223.2305701993367 + cos(Q4) + 56.00000000000001 * sin(Q4) + 26.83722834645669 * sin(
                        Q3 + Q4) - 208.6278365414622 * cos(Q3) - 106.9875743863997 * sin(Q3) - 13.16277165354331 *
                      cos(Q3 + Q4)) - 20227.95775625667 * MD * (-0.1266750892366645 + 0.004394033122000001 *
                          cos(Q4) + 0.008572641351000001 * sin(Q4) + 0.005141117272184 * sin(Q3 + Q4) - 0.12013863196 *
                          cos(Q3) - 0.06160894460000001 * sin(Q3) - 1.020849752000116E-06 * cos(Q3 + Q4)) - MF * (
                      -3080.203338068725 + 41.23763820954416 * sin(Q4) + cos(Q4) * cos(Q5) + 56 * sin(Q4) * cos(
                        Q5) + 26.83722834645669 * cos(Q5) * sin(Q3 + Q4) - 2430.159172181348 * cos(Q3) - 2309.307739734473 *
                      cos(Q4) - 1246.223128776358 * sin(Q3) - 1106.703913098104 * cos(Q3 + Q4) - 542.8016152836623 *
                      sin(Q3 + Q4) - 13.16277165354331 * cos(Q5) * cos(Q3 + Q4) - 20227.95775625667 * pow(sin(
                            Q5), 2) * (-1.515264616959965E-05 + 1.734723475976807E-18 * sin(Q3 + Q4) + 3.469446951953614E-18 *
                                       cos(Q3 + Q4)))) * (Q3DDD + KP * (Q3D - Q3) + 2 * pow(KP, 0.5) * (Q3DD - U3)) - 4.9436528E-05 * (
                1100.40090195854 + 2083.824698781276 * MCC + 2349.917601025721 * MB + 1508.443295705232 *
                MC * (2.630617767504619 + sin(Q3) + 2.096754115277765 * cos(Q3)) + 1692.793586141163 *
                MDD * (2.635563393131584 + sin(Q3) + 2.300529702918576 * cos(Q3)) - 2.022795775625667 *
                pow(sin(0.003333333333333333 * Q5), 2) - 11.64829781330922 * MFF * (-473.4893523459483 +
                    cos(Q4) + 56.00000000000001 * sin(Q4) + 53.6744566929134 * sin(Q3 + Q4) - 417.2556730829245 *
                    cos(Q3) - 213.9751487727995 * sin(Q3) - 26.32554330708661 * cos(Q3 + Q4)) - 20227.95775625667 *
                MD * (-0.2707869595483445 + 0.004394033122000001 * cos(Q4) + 0.008572641351000001 *
                      sin(Q4) + 0.010282234544368 * sin(Q3 + Q4) - 0.24027726392 * cos(Q3) - 0.1232178892 * sin(
                        Q3) - 2.041699504000231E-06 * cos(Q3 + Q4)) - MF * (-5995.292162908529 + 41.23763820954416 *
                            sin(Q4) + cos(Q4) * cos(Q5) + 56 * sin(Q4) * cos(Q5) + 53.67445669291339 * cos(Q5) * sin(Q3 +
                                Q4) - 4860.318344362695 * cos(Q3) - 2492.446257552715 * sin(Q3) - 2309.307739734473 *
                            cos(Q4) - 2213.407826196209 * cos(Q3 + Q4) - 1085.603230567325 * sin(Q3 + Q4) - 26.32554330708661 *
                            cos(Q5) * cos(Q3 + Q4) - 20227.95775625667 * pow(sin(Q5), 2) * (-1.515264616959965E-05 +
                                1.734723475976807E-18 * sin(Q3 + Q4) + 3.469446951953614E-18 * cos(Q3 + Q4)))) * (Q2DDD +
                                    KP * (Q2D - Q2) + 2 * pow(KP, 0.5) * (Q2DD - U2)) - MF * (1.387778780781446E-17 * pow(U2, 2) +
                                        0.12013863196 * sin(Q3) * pow(U2, 2) + 0.05471159898758401 * sin(Q3 + Q4) * pow(U2, 2) +
                                        0.001019322828 * cos(Q4) * pow((U2 + U3), 2) + 0.05708207836800001 * sin(Q4) * pow((U2 +
                                            U3), 2) + 0.06160894460000001 * cos(Q3) * pow((U2 + U3), 2) + 0.000650721729408 * cos(Q5) *
                                        sin(Q3 + Q4) * pow(U2, 2) + 0.001326739390592 * cos(Q5) * cos(Q3 + Q4) * pow(U2, 2) + 0.001384222784 *
                                        cos(Q4) * cos(Q5) * pow((U2 + U3), 2) + 0.0002716307158576 * cos(Q5) * (U2 + U3 + U4) * (U2 + U3 +
                                            2.30040015425493 * U4) + 0.011665025844648 * cos(Q3 + Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 *
                                                U4) + 2.4718264E-05 * sin(Q4) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(
                                                    Q2 + Q3 + Q4) * U1) + 4.03225E-05 * cos(Q2 + Q3) * (26.32554330708661 * sin(Q2) - 53.6744566929134 *
                                                        cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 +
                                                            Q4)) * pow(U1, 2) + 0.0006248593406592 * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 *
                                                                sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                                                sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                                    cos(Q2 + Q3 + Q4) * U1)) + 0.001384222784 * cos(Q4) * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (
                                                                        23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) -
                                                                        23.3113491101155 * sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 +
                                                                            cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 0.000650721729408 * cos(Q5) * sin(Q3 + Q4) * ((
                                                                                U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) *
                                                                                    U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 * sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (
                                                                                        cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 0.001326739390592 *
                                        cos(Q5) * cos(Q3 + Q4) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 * sin(Q5) * U2 +
                                            23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 * sin(Q5) *
                                            U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 +
                                                Q3 + Q4) * U1)) + 2.4718264E-05 * cos(Q4) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 *
                                                    U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                    cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (
                                                        23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (
                                                            sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 0.001384222784 *
                                        sin(Q4) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 *
                                            U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (
                                                cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 *
                                                    sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(
                                                        Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 0.001326739390592 * sin(Q3 + Q4) * (
                                          sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(
                                              Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) *
                                                  U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) *
                                                      U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 +
                                                          sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 0.0616089446 * cos(Q3) * pow(U2, 2) - 0.026834227252416 *
                                        cos(Q3 + Q4) * pow(U2, 2) - 0.12013863196 * sin(Q3) * pow((U2 + U3), 2) - 2.4718264E-05 * sin(
                                          Q4) * cos(Q5) * pow((U2 + U3), 2) - 0.024813977804 * sin(Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 *
                                              U4) - 0.0004431067465 * cos(Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) - 0.023783513875352 *
                                        sin(Q3 + Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) - 0.0006248593406592 * U4 * (
                                          cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - 0.001384222784 *
                                        cos(Q4) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) -
                                        0.001326739390592 * cos(Q3 + Q4) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                            cos(Q2 + Q3 + Q4) * U1) - 0.000650721729408 * sin(Q3 + Q4) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 +
                                                cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - 0.00216428828 * sin(Q2) * (26.32554330708661 *
                                                    sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) -
                                                    25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.00106151172 * cos(Q2) * (26.32554330708661 *
                                                        sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) -
                                                        25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.00225806 * sin(Q2 + Q3) * (26.32554330708661 *
                                                            sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) -
                                                            25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.001019322828 * sin(Q2 + Q3 + Q4) * (
                                          26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 * cos(Q2 +
                                              Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 2.4718264E-05 * cos(
                                          Q5) * cos(Q2 + Q3 + Q4) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) -
                                              56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) *
                                        pow(U1, 2) - 2.4718264E-05 * sin(Q4) * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 *
                                            sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                            sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                cos(Q2 + Q3 + Q4) * U1)) - 1.51526461696E-05 * cos(Q5) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 +
                                                    sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 +
                                                        U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(
                                                            Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) *
                                                                U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 +
                                                                    Q4) * U1)) - 0.000650721729408 * cos(Q3 + Q4) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) *
                                                                        U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) -
                                                                        23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                                            cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 -
                                                                                U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) *
                                                                                    U1)) - 2.4718264E-05 * sin(Q5) * (25.27925669291339 * cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) +
                                                                                        589.2935780127277 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) + 25.27925669291339 *
                                                                                        sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) +
                                                                                        sin(Q4) * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(
                                                                                            Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) *
                                                                                                U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) *
                                                                                                    U1)) - 589.2935780127277 * cos(Q5) * U2 * U5 - 589.2935780127277 * cos(Q5) * U3 * U5 - 56 * cos(
                                                                                                        Q4) * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 +
                                                                                                            Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) -
                                                                                                            sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) -
                                                                                        53.67445669291339 * cos(Q3 + Q4) * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 *
                                                                                            cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (
                                                                                                U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(
                                                                                                    Q5) * cos(Q2 + Q3 + Q4) * U1)) - 26.32554330708661 * sin(Q3 + Q4) * (23.3113491101155 * cos(
                                                                                                        Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                                                                        cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                                                            U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1))))) / cos(Q2);
  return TAU;
}

float TAU3(float Q1, float Q2, float Q3, float Q4, float Q5, float U1, float U2, float U3, float U4, float U5, float Q1D, float Q2D, float Q3D, float Q4D, float Q5D) {
  float  TAU = -7120.656000000001 * (0.0025034 * GRAV * (MC * (sin(Q2 + Q3) - 91.13639849804267 *
                                     cos(Q2 + Q3)) + 5.073104577774227 * MDD * (sin(Q2 + Q3) - 21.74797578899545 * cos(Q2 + Q3)) -
                                     2.536550291603419 * MFF * (sin(Q2 + Q3) + 56.00000000000001 * cos(Q2 + Q3) - 7.140571653543308 *
                                         sin(Q2 + Q3 + Q4)) - 2.381237516976912 * MD * (1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 *
                                             cos(Q2 + Q3) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) - 1.554941279859391 *
                                     MF * (1.631283653253319 * sin(Q2 + Q3) + 91.35188458218588 * cos(Q2 + Q3) + 41.23763820954416 *
                                         cos(Q2 + Q3 + Q4) - cos(Q5) * sin(Q2 + Q3 + Q4))) + 4.213094973272125E-08 * sin(0.0001404364991090709 *
                                             Q4) * (U4 + 7120.656 * sin(Q2 + Q3) * U1) * (cos(0.0001404364991090709 * Q4) * U2 + cos(
                                                 0.0001404364991090709 * Q4) * U3 - sin(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1) +
                                     0.02 * sin(Q5) * (U5 + sin(Q2 + Q3 + Q4) * U1) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                         cos(Q2 + Q3 + Q4) * U1) + 4.213094973272125E-08 * cos(0.0001404364991090709 * Q4) * (sin(
                                             0.0001404364991090709 * Q4) * U4 * (U2 + U3) + cos(0.0001404364991090709 * Q4) * cos(Q2 +
                                                 Q3) * U1 * U4 - 7120.656 * sin(0.0001404364991090709 * Q4) * sin(Q2 + Q3) * U1 * (U2 + U3)) +
                                     0.02 * cos(Q5) * (sin(Q5) * U4 * U5 + sin(Q5) * U5 * (U2 + U3) + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 -
                                         sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) + 1.51526461696E-05 * (-1458.491127730424 +
                                             6.599507365296038 * pow(sin(0.003333333333333333 * Q5), 2) + 19.00167781107771 * MFF * (
                                                 -7.140571653543308 + cos(Q4) + 56.00000000000001 * sin(Q4)) - 12.10394109462279 * MD * (
                                                     1 - 23.37057380471704 * sin(Q4) - 11.97893055051152 * cos(Q4)) - MF * (1700.542805101257 +
                                                         pow(cos(Q5), 2) + 3767.135966160216 * cos(Q4) - 67.27028511000385 * sin(Q4) - 91.35188458218588 *
                                                         sin(Q4) * cos(Q5) - 1.631283653253319 * cos(Q4) * cos(Q5))) * (Q4DDD + KP * (Q4D - Q4) + 2 *
                                                             pow(KP, 0.5) * (Q4DD - U4)) + 1.515264616959965E-05 * (-2204.235460008928 + 6.599507365296192 *
                                                                 pow(sin(0.003333333333333333 * Q5), 2) + 38.00335562215631 * MFF * (-223.2305701993367 +
                                                                     cos(Q4) + 56.00000000000001 * sin(Q4)) + 289.9845395199443 * MD * (-28.82888811247893 +
                                                                         cos(Q4) + 1.950973311529808 * sin(Q4)) - 5045.155725447793 * MDD - 3435.642946295701 *
                                                                 MC - MF * (10049.37070817587 + 7534.27193232061 * cos(Q4) - 134.5405702200109 * sin(Q4) -
                                                                     pow(sin(Q5), 2) - 182.703769164376 * sin(Q4) * cos(Q5) - 3.262567306506715 * cos(Q4) *
                                                                     cos(Q5))) * (Q3DDD + KP * (Q3D - Q3) + 2 * pow(KP, 0.5) * (Q3DD - U3)) + 4.03225E-05 * MFF * (
                                       1527.904881889764 * cos(Q3) * pow(U2, 2) + 7.140571653543307 * sin(Q4) * pow((U2 + U3), 2) +
                                       399.8720125984253 * cos(Q4) * pow((U2 + U3 + U4), 2) + cos(Q2 + Q3) * (sin(Q2 + Q3) + 53.6744566929134 *
                                           cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                           sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 2979.444031496063 * sin(Q3) * pow(U2, 2) - 383.2663039807552 *
                                       cos(Q3 + Q4) * pow(U2, 2) - 187.9794283027094 * sin(Q3 + Q4) * pow(U2, 2) - 399.8720125984253 *
                                       cos(Q4) * pow((U2 + U3), 2) - 7.140571653543307 * sin(Q4) * pow((U2 + U3 + U4), 2) - 56.00000000000001 *
                                       sin(Q2 + Q3) * (sin(Q2 + Q3) + 53.6744566929134 * cos(Q2) + 56.00000000000001 * cos(Q2 + Q3) -
                                           26.32554330708661 * sin(Q2) - 7.140571653543308 * sin(Q2 + Q3 + Q4)) * pow(U1, 2) - 7.140571653543307 *
                                       cos(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 53.6744566929134 * cos(Q2) + 56.00000000000001 * cos(Q2 +
                                           Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 * sin(Q2 + Q3 + Q4)) * pow(U1, 2)) +
                                     MD * (0.0616089446 * cos(Q3) * pow(U2, 2) + 0.002197016561 * sin(Q4) * pow((U2 + U3), 2) +
                                         0.0042863206755 * cos(Q4) * pow((U2 + U3 + U4), 2) + 3.78535565E-05 * cos(Q2 + Q3) * (57.17529553662944 *
                                             cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 *
                                             sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2) + 3.55357862161E-05 *
                                         sin(Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) +
                                             59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(
                                                 Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 0.12013863196 * sin(Q3) * pow(U2, 2) - 0.005141117272184 *
                                         cos(Q3 + Q4) * pow(U2, 2) - 1.020849752000116E-06 * sin(Q3 + Q4) * pow(U2, 2) - 0.0042863206755 *
                                         cos(Q4) * pow((U2 + U3), 2) - 0.002197016561 * sin(Q4) * pow((U2 + U3 + U4), 2) - 0.002119799164 *
                                         sin(Q2 + Q3) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 *
                                             cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 +
                                                 Q3 + Q4)) * pow(U1, 2) - 7.24893818618E-05 * cos(Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) +
                                                     1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 *
                                                     sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2)) - 0.011 * sin(
                                       Q2 + Q3) * cos(Q2 + Q3) * pow(U1, 2) - 0.0013 * sin(Q2 + Q3 + Q4) * cos(Q2 + Q3 + Q4) * pow(U1, 2) -
                                     2.4718264E-05 * MF * sin(Q5) * (-25.27925669291339 + sin(Q4) - 56 * cos(Q4)) * (Q5DDD + KP * (
                                         Q5D - Q5) + 2 * pow(KP, 0.5) * (Q5DD - U5)) - 4.213094973272125E-08 * cos(0.0001404364991090709 *
                                             Q4) * (U4 + 7120.656 * sin(Q2 + Q3) * U1) * (sin(0.0001404364991090709 * Q4) * U2 + sin(
                                                 0.0001404364991090709 * Q4) * U3 + cos(0.0001404364991090709 * Q4) * cos(Q2 + Q3) * U1) -
                                     0.02 * cos(Q5) * (U5 + sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) *
                                         cos(Q2 + Q3 + Q4) * U1) - 4.213094973272125E-08 * sin(0.0001404364991090709 * Q4) * (cos(
                                             0.0001404364991090709 * Q4) * U4 * (U2 + U3) - sin(0.0001404364991090709 * Q4) * cos(Q2 +
                                                 Q3) * U1 * U4 - 7120.656 * sin(Q2 + Q3) * cos(0.0001404364991090709 * Q4) * U1 * (U2 + U3)) -
                                     6.666666666666667E-07 * cos(0.003333333333333333 * Q5) * (sin(0.003333333333333333 *
                                         Q5) * U5 * (U2 + U3 + U4) + cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 + 300 * sin(
                                             0.003333333333333333 * Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - sin(0.003333333333333333 *
                                                 Q5) * (3.333333333333334E-07 * U5 - 0.0001 * sin(Q2 + Q3 + Q4) * U1) * (cos(0.003333333333333333 *
                                                     Q5) * U2 + cos(0.003333333333333333 * Q5) * U3 + cos(0.003333333333333333 * Q5) * U4 - sin(
                                                         0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1) - 0.02 * sin(Q5) * (cos(Q5) * U4 * U5 + cos(
                                                             Q5) * U5 * (U2 + U3) - sin(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - cos(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 +
                                                                 U4)) - 3.333333333333334E-07 * sin(0.003333333333333333 * Q5) * (sin(0.003333333333333333 *
                                                                     Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - cos(0.003333333333333333 * Q5) * U5 * (U2 + U3 + U4) - 300 * cos(
                                                                         0.003333333333333333 * Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - MC * (0.07817978280468801 *
                                                                             sin(Q3) * pow(U2, 2) - 0.037286099612272 * cos(Q3) * pow(U2, 2) - 0.000571152862924 * sin(
                                                                                 Q2 + Q3) * (sin(Q2 + Q3) + 66.77606455220899 * sin(Q2) - 136.1479587760646 * cos(Q2) -
                                                                                     91.13639849804267 * cos(Q2 + Q3)) * pow(U1, 2) - 6.267011559999999E-06 * cos(Q2 + Q3) * (
                                                                                         sin(Q2 + Q3) + 66.77606455220899 * sin(Q2) - 136.1479587760646 * cos(Q2) - 91.13639849804267 *
                                                                                         cos(Q2 + Q3)) * pow(U1, 2)) - 0.0001612902540001 * MDD * (596.8176940408338 * sin(Q3) *
                                                                                             pow(U2, 2) - 259.4262066182751 * cos(Q3) * pow(U2, 2) - 21.74797578899545 * sin(Q2 + Q3) * (
                                                                                                 sin(Q2 + Q3) + 13.16276128916434 * sin(Q2) - 26.83720721479747 * cos(Q2) - 21.74797578899545 *
                                                                                                 cos(Q2 + Q3)) * pow(U1, 2) - cos(Q2 + Q3) * (sin(Q2 + Q3) + 13.16276128916434 * sin(Q2) -
                                                                                                     26.83720721479747 * cos(Q2) - 21.74797578899545 * cos(Q2 + Q3)) * pow(U1, 2)) - 1.51526461696E-05 * (
                                       15.5999016445218 * MC * (cos(Q2 + Q3) + 91.13639849804268 * sin(Q2 + Q3)) + 79.13993244565121 *
                                       MDD * (cos(Q2 + Q3) + 21.74797578899545 * sin(Q2 + Q3)) - 6.599507365296038 * sin(0.003333333333333333 *
                                           Q5) * cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) - MF * sin(Q5) * (cos(Q5) * cos(Q2 +
                                               Q3 + Q4) - 91.35188458218588 * sin(Q4) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q4) *
                                               cos(Q2 + Q3 + Q4) - 1.631283653253319 * sin(Q2 + Q3 + Q4) * (-25.27925669291339 + sin(Q4) -
                                                   56 * cos(Q4)))) * (Q1DDD + KP * (Q1D - Q1) + 2 * pow(KP, 0.5) * (Q1DD - U1)) - 1.515264616959965E-05 * (
                                       2204.235460008928 + 2460.698890143566 * MC * (1.396206159175881 + sin(Q3) + 2.096754115277765 *
                                           cos(Q3)) + 2761.426505404208 * MDD * (1.827010682911259 + sin(Q3) + 2.300529702918577 *
                                               cos(Q3)) - 6.599507365296192 * pow(sin(0.003333333333333333 * Q5), 2) - 38.00335562215631 *
                                       MFF * (-223.2305701993367 + cos(Q4) + 56.00000000000001 * sin(Q4) + 26.8372283464567 *
                                           sin(Q3 + Q4) - 208.6278365414622 * cos(Q3) - 106.9875743863997 * sin(Q3) - 13.16277165354331 *
                                           cos(Q3 + Q4)) - 65995.07365296193 * MD * (-0.1266750892366645 + 0.004394033122000001 *
                                               cos(Q4) + 0.008572641351000001 * sin(Q4) + 0.005141117272184 * sin(Q3 + Q4) - 0.12013863196 *
                                               cos(Q3) - 0.0616089446 * sin(Q3) - 1.020849752000116E-06 * cos(Q3 + Q4)) - MF * (-10049.37070817587 +
                                                   pow(sin(Q5), 2) + 134.5405702200109 * sin(Q4) + 3.262567306506715 * cos(Q4) * cos(Q5) +
                                                   182.703769164376 * sin(Q4) * cos(Q5) + 87.55826380040487 * cos(Q5) * sin(Q3 + Q4) -
                                                   7928.557864766286 * cos(Q3) - 7534.27193232061 * cos(Q4) - 4065.886836558251 * sin(Q3) -
                                                   3610.696004856924 * cos(Q3 + Q4) - 1770.926803943512 * sin(Q3 + Q4) - 42.94442845986372 *
                                                   cos(Q5) * cos(Q3 + Q4))) * (Q2DDD + KP * (Q2D - Q2) + 2 * pow(KP, 0.5) * (Q2DD - U2)) - 1.51526461696E-05 *
                                     MF * (7928.5578647661 * sin(Q3) * pow(U2, 2) + 3610.696004856839 * sin(Q3 + Q4) * pow(U2, 2) +
                                         67.27028511000385 * cos(Q4) * pow((U2 + U3), 2) + 3767.135966160216 * sin(Q4) * pow((U2 +
                                             U3), 2) + 42.94442845986271 * cos(Q5) * sin(Q3 + Q4) * pow(U2, 2) + 87.55826380040281 * cos(
                                                 Q5) * cos(Q3 + Q4) * pow(U2, 2) + 91.35188458218588 * cos(Q4) * cos(Q5) * pow((U2 + U3), 2) +
                                         17.92628909942867 * cos(Q5) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) + 1.631283653253319 *
                                         sin(Q4) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) +
                                         2.661086357371495 * cos(Q2 + Q3) * (26.32554330708661 * sin(Q2) - 53.6744566929134 *
                                             cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 +
                                                 Q4)) * pow(U1, 2) + 41.23763820954417 * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 *
                                                     sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                                     sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                         cos(Q2 + Q3 + Q4) * U1)) + 91.35188458218588 * cos(Q4) * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (
                                                             23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) -
                                                             23.3113491101155 * sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 +
                                                                 cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) + 1.631283653253319 * cos(Q4) * (sin(Q5) * U2 *
                                                                     U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 +
                                                                         Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                             U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 *
                                                                                 sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) *
                                                                                     cos(Q2 + Q3 + Q4) * U1)) + 91.35188458218588 * sin(Q4) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 +
                                                                                         sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 +
                                                                                             U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(
                                                                                                 Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) *
                                                                                                     U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 +
                                                                                                         Q4) * U1)) - 4065.886836558155 * cos(Q3) * pow(U2, 2) - 1770.926803943471 * cos(Q3 + Q4) *
                                         pow(U2, 2) - 1.631283653253319 * sin(Q4) * cos(Q5) * pow((U2 + U3), 2) - 1637.600292797904 *
                                         sin(Q4) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) - 29.24286237139114 * cos(Q4) * (
                                             U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 * U4) - 41.23763820954417 * U4 * (cos(Q5) * U2 + cos(
                                                 Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - 91.35188458218588 * cos(Q4) * U4 * (
                                                     cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - 149.0208360128037 *
                                         sin(Q2 + Q3) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 *
                                             cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 67.27028511000385 *
                                         sin(Q2 + Q3 + Q4) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 *
                                             cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 1.631283653253319 *
                                         cos(Q5) * cos(Q2 + Q3 + Q4) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) -
                                             56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) *
                                         pow(U1, 2) - 1.631283653253319 * sin(Q4) * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 *
                                             sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                             sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                 cos(Q2 + Q3 + Q4) * U1)) - cos(Q5) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(
                                                     Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                     cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (
                                                         23.3113491101155 * sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (
                                                             sin(Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 1.631283653253319 *
                                         sin(Q5) * (25.27925669291339 * cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) + 589.2935780127277 *
                                             cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) + 25.27925669291339 * sin(Q5) * (U2 + U3) * (
                                                 cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) + sin(Q4) * (23.3113491101155 *
                                                     cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) -
                                                     23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(
                                                         Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 589.2935780127277 *
                                             cos(Q5) * U2 * U5 - 589.2935780127277 * cos(Q5) * U3 * U5 - 56 * cos(Q4) * (23.3113491101155 *
                                                 cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) -
                                                 23.3113491101155 * cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(
                                                     Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1))))) / cos(Q3);
  return TAU;
}
float TAU4(float Q1, float Q2, float Q3, float Q4, float Q5, float U1, float U2, float U3, float U4, float U5, float Q1D, float Q2D, float Q3D, float Q4D, float Q5D) {
  float TAU = -7120.656000000001 * (0.00389264 * GRAV * (11.64829781330922 * MFF * sin(Q2 + Q3 +
                                    Q4) + 1.53140028361215 * MD * (cos(Q2 + Q3 + Q4) + 2.039898074042263 * sin(Q2 + Q3 + Q4)) - MF * (
                                      41.23763820954416 * cos(Q2 + Q3 + Q4) - cos(Q5) * sin(Q2 + Q3 + Q4))) + 0.0006248593406592 *
                                    MF * sin(Q5) * (Q5DDD + KP * (Q5D - Q5) + 2 * pow(KP, 0.5) * (Q5DD - U5)) + 0.02 * sin(Q5) * (U5 + sin(
                                          Q2 + Q3 + Q4) * U1) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) +
                                    1.51526461696E-05 * (-1458.491127730424 + 6.599507365296038 * pow(sin(0.003333333333333333 *
                                        Q5), 2) - 135.6828419475444 * MFF - 12.10394109462279 * MD - MF * (1700.542805101257 +
                                            pow(cos(Q5), 2))) * (Q4DDD + KP * (Q4D - Q4) + 2 * pow(KP, 0.5) * (Q4DD - U4)) + 0.02 * cos(Q5) * (
                                      sin(Q5) * U4 * U5 + sin(Q5) * U5 * (U2 + U3) + cos(Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 +
                                          Q3 + Q4) * U1 * (U2 + U3 + U4)) + 1.51526461696E-05 * (6.599507365296038 * sin(0.003333333333333333 *
                                              Q5) * cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) + MF * sin(Q5) * (41.23763820954417 *
                                                  sin(Q2 + Q3 + Q4) + cos(Q5) * cos(Q2 + Q3 + Q4))) * (Q1DDD + KP * (Q1D - Q1) + 2 * pow(KP, 0.5) * (
                                                      Q1DD - U1)) + 0.0002879257005 * MFF * (sin(Q4) * pow((U2 + U3), 2) - 53.6744566929134 * cos(
                                                          Q3 + Q4) * pow(U2, 2) - 26.32554330708661 * sin(Q3 + Q4) * pow(U2, 2) - 56.00000000000001 *
                                                          cos(Q4) * pow((U2 + U3), 2) - cos(Q2 + Q3 + Q4) * (sin(Q2 + Q3) + 53.6744566929134 * cos(Q2) +
                                                              56.00000000000001 * cos(Q2 + Q3) - 26.32554330708661 * sin(Q2) - 7.140571653543308 *
                                                              sin(Q2 + Q3 + Q4)) * pow(U1, 2)) + 1.515264616959965E-05 * (-1458.491127730458 + 6.599507365296192 *
                                                                  pow(sin(0.003333333333333333 * Q5), 2) + 19.00167781107816 * MFF * (-7.140571653543308 +
                                                                      cos(Q4) + 56.00000000000001 * sin(Q4)) - 12.10394109462307 * MD * (1 - 23.37057380471704 *
                                                                          sin(Q4) - 11.97893055051152 * cos(Q4)) - MF * (1701.542805101297 + 3767.135966160305 *
                                                                              cos(Q4) - 67.27028511000543 * sin(Q4) - pow(sin(Q5), 2) - 91.35188458218802 * sin(Q4) *
                                                                              cos(Q5) - 1.631283653253357 * cos(Q4) * cos(Q5))) * (Q3DDD + KP * (Q3D - Q3) + 2 * pow(KP, 0.5) * (
                                                                                  Q3DD - U3)) + MD * (0.002197016561 * sin(Q4) * pow((U2 + U3), 2) + 3.55357862161E-05 * sin(
                                                                                      Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) + 59.65251904401639 *
                                                                                          cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(Q2 + Q3 + Q4) - cos(Q2 +
                                                                                              Q3 + Q4)) * pow(U1, 2) - 0.005141117272184 * cos(Q3 + Q4) * pow(U2, 2) - 1.020849752000116E-06 *
                                                                                      sin(Q3 + Q4) * pow(U2, 2) - 0.0042863206755 * cos(Q4) * pow((U2 + U3), 2) - 7.24893818618E-05 *
                                                                                      cos(Q2 + Q3 + Q4) * (57.17529553662944 * cos(Q2) + 1.065223554357435 * sin(Q2 + Q3) +
                                                                                          59.65251904401639 * cos(Q2 + Q3) - 28.0425888119654 * sin(Q2) - 2.039898074042263 * sin(
                                                                                              Q2 + Q3 + Q4) - cos(Q2 + Q3 + Q4)) * pow(U1, 2)) + 1.515264616959965E-05 * (-1458.491127730458 +
                                                                                                  6.599507365296192 * pow(sin(0.003333333333333333 * Q5), 2) + 19.00167781107816 * MFF * (
                                                                                                      -7.140571653543308 + cos(Q4) + 56.00000000000001 * sin(Q4) + 53.6744566929134 * sin(
                                                                                                          Q3 + Q4) - 26.32554330708661 * cos(Q3 + Q4)) + 65995.07365296193 * MD * (-0.0001834067366645 +
                                                                                                              0.002197016561 * cos(Q4) + 0.0042863206755 * sin(Q4) + 0.005141117272184 * sin(Q3 + Q4) -
                                                                                                              1.020849752000116E-06 * cos(Q3 + Q4)) + MF * (-1701.542805101297 + pow(sin(Q5), 2) +
                                                                                                                  67.27028511000543 * sin(Q4) + 1.631283653253357 * cos(Q4) * cos(Q5) + 91.35188458218802 *
                                                                                                                  sin(Q4) * cos(Q5) + 87.55826380040487 * cos(Q5) * sin(Q3 + Q4) - 3767.135966160305 * cos(
                                                                                                                      Q4) - 3610.696004856923 * cos(Q3 + Q4) - 1770.926803943512 * sin(Q3 + Q4) - 42.94442845986372 *
                                                                                                                  cos(Q5) * cos(Q3 + Q4))) * (Q2DDD + KP * (Q2D - Q2) + 2 * pow(KP, 0.5) * (Q2DD - U2)) - 0.0013 * sin(
                                      Q2 + Q3 + Q4) * cos(Q2 + Q3 + Q4) * pow(U1, 2) - 0.02 * cos(Q5) * (U5 + sin(Q2 + Q3 + Q4) * U1) * (sin(
                                          Q5) * U2 + sin(Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1) - 6.666666666666667E-07 *
                                    cos(0.003333333333333333 * Q5) * (sin(0.003333333333333333 * Q5) * U5 * (U2 + U3 + U4) +
                                        cos(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 + 300 * sin(0.003333333333333333 *
                                            Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - sin(0.003333333333333333 * Q5) * (3.333333333333334E-07 *
                                                U5 - 0.0001 * sin(Q2 + Q3 + Q4) * U1) * (cos(0.003333333333333333 * Q5) * U2 + cos(0.003333333333333333 *
                                                    Q5) * U3 + cos(0.003333333333333333 * Q5) * U4 - sin(0.003333333333333333 * Q5) * cos(Q2 +
                                                        Q3 + Q4) * U1) - 0.02 * sin(Q5) * (cos(Q5) * U4 * U5 + cos(Q5) * U5 * (U2 + U3) - sin(Q5) * cos(Q2 + Q3 +
                                                            Q4) * U1 * U5 - cos(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - 3.333333333333334E-07 * sin(
                                      0.003333333333333333 * Q5) * (sin(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) * U1 * U5 -
                                          cos(0.003333333333333333 * Q5) * U5 * (U2 + U3 + U4) - 300 * cos(0.003333333333333333 * Q5) *
                                          sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4)) - 1.51526461696E-05 * MF * (3610.696004856839 * sin(Q3 +
                                              Q4) * pow(U2, 2) + 67.27028511000385 * cos(Q4) * pow((U2 + U3), 2) + 3767.135966160216 *
                                              sin(Q4) * pow((U2 + U3), 2) + 42.94442845986271 * cos(Q5) * sin(Q3 + Q4) * pow(U2, 2) +
                                              87.55826380040281 * cos(Q5) * cos(Q3 + Q4) * pow(U2, 2) + 91.35188458218588 * cos(Q4) *
                                              cos(Q5) * pow((U2 + U3), 2) + 17.92628909942867 * cos(Q5) * (U2 + U3 + U4) * (U2 + U3 + 2.30040015425493 *
                                                  U4) + 41.23763820954417 * cos(Q5) * ((U5 + sin(Q2 + Q3 + Q4) * U1) * (23.3113491101155 * sin(
                                                      Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) - 23.3113491101155 *
                                                      sin(Q5) * U5 * (U2 + U3) - cos(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                                          cos(Q2 + Q3 + Q4) * U1)) + 41.23763820954417 * sin(Q5) * (23.3113491101155 * cos(Q5) * U2 *
                                                              U5 + 23.3113491101155 * cos(Q5) * U3 * U5 - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 *
                                                              cos(Q5) * (U2 + U3) * (U5 + sin(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) *
                                                                  U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1)) - 1770.92680394347 * cos(Q3 + Q4) * pow(U2,
                                                                      2) - 1.631283653253319 * sin(Q4) * cos(Q5) * pow((U2 + U3), 2) - 41.23763820954417 * U4 * (
                                                                          cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - 67.27028511000385 *
                                              sin(Q2 + Q3 + Q4) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 *
                                                  cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - 1.631283653253319 *
                                              cos(Q5) * cos(Q2 + Q3 + Q4) * (26.32554330708661 * sin(Q2) - 53.6744566929134 * cos(Q2) -
                                                  56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) - 25.27925669291339 * cos(Q2 + Q3 + Q4)) *
                                              pow(U1, 2) - cos(Q5) * (sin(Q5) * U2 * U5 + sin(Q5) * U3 * U5 + sin(Q5) * U4 * U5 + cos(Q5) * cos(Q2 +
                                                  Q3 + Q4) * U1 * U5 - sin(Q5) * sin(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 23.3113491101155 * cos(Q5) * (
                                                      U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) * cos(Q2 + Q3 + Q4) * U1) - (23.3113491101155 *
                                                          sin(Q5) * U2 + 23.3113491101155 * sin(Q5) * U3 - U5 - sin(Q2 + Q3 + Q4) * U1) * (sin(Q5) * U2 + sin(
                                                              Q5) * U3 + sin(Q5) * U4 + cos(Q5) * cos(Q2 + Q3 + Q4) * U1)))) / cos(Q4);
  return TAU;
}

float TAU5(float Q1, float Q2, float Q3, float Q4, float Q5, float U1, float U2, float U3, float U4, float U5, float Q1D, float Q2D, float Q3D, float Q4D, float Q5D) {
  float TAU = 1.167792 * GRAV * MF * sin(Q5) * cos(Q2 + Q3 + Q4) + 9.999999999999999E-05 * cos(
                Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) + 0.18745780219776 * MF * sin(Q5) * (Q4DDD + KP * (Q4D - Q4) + 2 *
                    pow(KP, 0.5) * (Q4DD - U4)) + 0.0001 * (sin(0.003333333333333333 * Q5) * U2 + sin(0.003333333333333333 *
                        Q5) * U3 + sin(0.003333333333333333 * Q5) * U4 + cos(0.003333333333333333 * Q5) * cos(Q2 +
                            Q3 + Q4) * U1) * (cos(0.003333333333333333 * Q5) * U2 + cos(0.003333333333333333 * Q5) * U3 +
                                cos(0.003333333333333333 * Q5) * U4 - sin(0.003333333333333333 * Q5) * cos(Q2 + Q3 + Q4) *
                                U1) + 300 * (3.333333333333333E-07 * sin(Q2 + Q3 + Q4) - 1.51526461696E-05 * MF * (sin(Q2 +
                                    Q3 + Q4) + 42.94442845986271 * cos(Q5) * (sin(Q2) - 2.038873654640384 * cos(Q2)) - 41.23763820954417 *
                                    cos(Q5) * cos(Q2 + Q3 + Q4) - 1.631283653253319 * cos(Q5) * (sin(Q2 + Q3) + 56.00000000000001 *
                                        cos(Q2 + Q3)))) * (Q1DDD + KP * (Q1D - Q1) + 2 * pow(KP, 0.5) * (Q1DD - U1)) + 0.00454579385088 *
              MF * (23.3113491101155 * cos(Q5) * U2 * U5 + 23.3113491101155 * cos(Q5) * U3 * U5 + 87.55826380040281 *
                    sin(Q5) * sin(Q3 + Q4) * pow(U2, 2) + 1.631283653253319 * sin(Q5) * cos(Q4) * pow((U2 + U3),
                        2) + 91.35188458218588 * sin(Q4) * sin(Q5) * pow((U2 + U3), 2) - 42.94442845986271 * sin(
                      Q5) * cos(Q3 + Q4) * pow(U2, 2) - cos(Q2 + Q3 + Q4) * U1 * (U2 + U3 + U4) - 85.88885691972541 * cos(
                      Q5) * (cos(Q2) + 2.038873654640384 * sin(Q2)) * U1 * U2 - 35.85257819885733 * cos(Q5) * sin(
                      Q2 + Q3 + Q4) * U1 * (U2 + U3 + 1.650200077127465 * U4) - 23.3113491101155 * cos(Q5) * (U2 + U3) * (
                      U5 + sin(Q2 + Q3 + Q4) * U1) - 3.262567306506638 * cos(Q5) * (56.00000000000001 * sin(Q2 + Q3) -
                          cos(Q2 + Q3)) * U1 * (U2 + U3) - sin(Q5) * U4 * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                              cos(Q2 + Q3 + Q4) * U1) - sin(Q5) * (U2 + U3) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                  cos(Q2 + Q3 + Q4) * U1) - 1.631283653253319 * sin(Q5) * sin(Q2 + Q3 + Q4) * (26.32554330708661 *
                                      sin(Q2) - 53.6744566929134 * cos(Q2) - 56.00000000000001 * cos(Q2 + Q3) - sin(Q2 + Q3) -
                                      25.27925669291339 * cos(Q2 + Q3 + Q4)) * pow(U1, 2) - cos(Q5) * U1 * (23.3113491101155 * sin(
                                          Q2 + Q3 + Q4) * (U2 + U3 + U4) + cos(Q2 + Q3 + Q4) * (cos(Q5) * U2 + cos(Q5) * U3 + cos(Q5) * U4 - sin(Q5) *
                                              cos(Q2 + Q3 + Q4) * U1))) - 300 * (1.111111111111111E-09 + 1.51526461696E-05 * MF) * (
                Q5DDD + KP * (Q5D - Q5) + 2 * pow(KP, 0.5) * (Q5DD - U5)) - 0.007415479200000001 * MF * sin(Q5) * (
                -25.27925669291339 + sin(Q4) - 56 * cos(Q4)) * (Q3DDD + KP * (Q3D - Q3) + 2 * pow(KP, 0.5) * (
                      Q3DD - U3)) - 0.007415479200000001 * MF * sin(Q5) * (-25.27925669291339 + sin(Q4) - 56 *
                          cos(Q4) - 53.67445669291339 * cos(Q3 + Q4) - 26.32554330708661 * sin(Q3 + Q4)) * (Q2DDD +
                              KP * (Q2D - Q2) + 2 * pow(KP, 0.5) * (Q2DD - U2));
  return TAU;
}
#endif
