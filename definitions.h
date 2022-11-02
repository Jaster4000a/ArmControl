#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#define La1x 0.00523878
#define La1y 0.0510455
#define La1z 0.00591081
#define La2x 0.13353695
#define La2y 0.03416269
#define La2z 0.04140198
#define La3x 0
#define La3y 0.0436457
#define La3z 0

#define Lb1x 0.09528819
#define Lb1y 0.02826982
#define Lb1y2 0.03967518
#define Lb1z 0
#define Lb2x 0.3408328
#define Lb2y 0.00218397
#define Lb2z 0
#define Lb3x 0.1671672
#define Lb3y 0
#define Lb3z 0.0944235
#define Lb4x 0.07941319
#define Lb4y 0.01905
#define Lb4z 0

#define Lc1x 0.22815086
#define Lc1y 0.0025034
#define Lc1z 0
#define Lc2x 0.07940049
#define Lc2y 0.01270001
#define Lc2z 0
#define Lc3x 0.12744914
#define Lc3y 0.00635
#define Lc3z 0

#define Ld1x 0.06978059
#define Ld1y 0
#define Ld1z 0
#define Ld2x 0.00596119
#define Ld2y 0.01216022
#define Ld2z 0
#define Ld3x 0
#define Ld3y 0.04534263
#define Ld3z 0

#define Lf1x 0.09074269
#define Lf1y 0.00389264
#define Lf1z 0
#define Lf2x 0.11385116
#define Lf2y 0
#define Lf2z 0

#define La (La1y+La3y+Lb1y+Lb1y2)
#define Lb (Lb2x+Lb3x)
#define Lc sqrt(sq(Lc1x+Lc3x)+sq(Lc3y))
#define Ld (Ld1x+Lf1x+Lf2x)
#define Le 0 

#define GRAV 9.81
#define KP 1
#define KPgripper 10
#define MB 0.98
#define MBB 0.37
#define MC 0.693
#define MCC 0.37
#define MD 0.447
#define MDD 0.37
#define MF 5.72
#define MFF 0.37

#define Q1DD 0
#define Q1DDD 0
#define Q2DD 0
#define Q2DDD 0
#define Q3DD 0
#define Q3DDD 0
#define Q4DD 0
#define Q4DDD 0
#define Q5DD 0
#define Q5DDD 0
  
#define R 0.0254
#define ki 0.2229172306
#define kt 0.3677493751

#endif
