/* This file was automatically generated by CasADi.
   The CasADi copyright holders make no ownership claim of its contents. */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Translational_drone_impl_dae_jac_x_xdot_u_z_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s10 CASADI_PREFIX(s10)
#define casadi_s11 CASADI_PREFIX(s11)
#define casadi_s12 CASADI_PREFIX(s12)
#define casadi_s13 CASADI_PREFIX(s13)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
#define casadi_s9 CASADI_PREFIX(s9)
#define casadi_trans CASADI_PREFIX(trans)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

void casadi_trans(const casadi_real* x, const casadi_int* sp_x, casadi_real* y,
    const casadi_int* sp_y, casadi_int* tmp) {
  casadi_int ncol_x, nnz_x, ncol_y, k;
  const casadi_int* row_x, *colind_y;
  ncol_x = sp_x[1];
  nnz_x = sp_x[2 + ncol_x];
  row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2;
  for (k=0; k<ncol_y; ++k) tmp[k] = colind_y[k];
  for (k=0; k<nnz_x; ++k) {
    y[tmp[row_x[k]]++] = x[k];
  }
}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[8] = {0, 1, 2, 6, 9, 12, 15, 19};
static const casadi_int casadi_s1[6] = {3, 10, 13, 16, 20, 23};
static const casadi_int casadi_s2[6] = {4, 7, 14, 17, 21, 24};
static const casadi_int casadi_s3[5] = {5, 8, 11, 18, 22};
static const casadi_int casadi_s4[38] = {10, 10, 0, 0, 0, 0, 5, 11, 17, 22, 23, 24, 25, 4, 5, 6, 7, 8, 3, 5, 6, 7, 8, 9, 3, 4, 6, 7, 8, 9, 3, 4, 5, 7, 8, 0, 1, 2};
static const casadi_int casadi_s5[38] = {10, 10, 0, 1, 2, 3, 6, 9, 12, 15, 19, 23, 25, 7, 8, 9, 4, 5, 6, 3, 5, 6, 3, 4, 6, 3, 4, 5, 3, 4, 5, 6, 3, 4, 5, 6, 4, 5};
static const casadi_int casadi_s6[23] = {10, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s7[7] = {0, 3, 6, 9, 12, 13, 14};
static const casadi_int casadi_s8[22] = {10, 4, 0, 3, 7, 11, 15, 7, 8, 9, 3, 4, 5, 6, 3, 4, 5, 6, 3, 4, 5, 6};
static const casadi_int casadi_s9[28] = {4, 10, 0, 0, 0, 0, 3, 6, 9, 12, 13, 14, 15, 1, 2, 3, 1, 2, 3, 1, 2, 3, 1, 2, 3, 0, 0, 0};
static const casadi_int casadi_s10[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s11[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s12[3] = {0, 0, 0};
static const casadi_int casadi_s13[3] = {10, 0, 0};

/* Translational_drone_impl_dae_jac_x_xdot_u_z:(i0[10],i1[10],i2[4],i3[],i4[])->(o0[10x10,25nz],o1[10x10,10nz],o2[10x4,15nz],o3[10x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_int *cii;
  const casadi_real *cs;
  casadi_real *w0=w+0, *w1=w+25, w2, w3, w4, w5, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, *w23=w+51, *w24=w+59, w27, w28, w29, w30, *w31=w+71, *w32=w+77, w33, *w35=w+84, *w36=w+89, *w37=w+94, *w38=w+119, *w39=w+129, *w40=w+139, *w41=w+154, *w42=w+156, *w44=w+163, *w45=w+167, *w46=w+171;
  /* #0: @0 = zeros(10x10,25nz) */
  casadi_clear(w0, 25);
  /* #1: @1 = ones(10x1,7nz) */
  casadi_fill(w1, 7, 1.);
  /* #2: {NULL, NULL, NULL, @2, NULL, NULL, NULL, @3, @4, @5} = vertsplit(@1) */
  w2 = w1[3];
  w3 = w1[4];
  w4 = w1[5];
  w5 = w1[6];
  /* #3: @6 = 00 */
  /* #4: @7 = 0.5 */
  w7 = 5.0000000000000000e-01;
  /* #5: @8 = input[2][1] */
  w8 = arg[2] ? arg[2][1] : 0;
  /* #6: @9 = (@8*@2) */
  w9  = (w8*w2);
  /* #7: @9 = (@7*@9) */
  w9  = (w7*w9);
  /* #8: @10 = 0.5 */
  w10 = 5.0000000000000000e-01;
  /* #9: @11 = input[2][2] */
  w11 = arg[2] ? arg[2][2] : 0;
  /* #10: @12 = (@11*@2) */
  w12  = (w11*w2);
  /* #11: @12 = (@10*@12) */
  w12  = (w10*w12);
  /* #12: @13 = 0.5 */
  w13 = 5.0000000000000000e-01;
  /* #13: @14 = input[2][3] */
  w14 = arg[2] ? arg[2][3] : 0;
  /* #14: @15 = (@14*@2) */
  w15  = (w14*w2);
  /* #15: @15 = (@13*@15) */
  w15  = (w13*w15);
  /* #16: @16 = 30.303 */
  w16 = 3.0303030303030301e+01;
  /* #17: @17 = input[2][0] */
  w17 = arg[2] ? arg[2][0] : 0;
  /* #18: @18 = input[0][5] */
  w18 = arg[0] ? arg[0][5] : 0;
  /* #19: @19 = (@18*@2) */
  w19  = (w18*w2);
  /* #20: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #21: @19 = (@17*@19) */
  w19  = (w17*w19);
  /* #22: @19 = (@16*@19) */
  w19  = (w16*w19);
  /* #23: @20 = 30.303 */
  w20 = 3.0303030303030301e+01;
  /* #24: @21 = input[0][4] */
  w21 = arg[0] ? arg[0][4] : 0;
  /* #25: @2 = (@21*@2) */
  w2  = (w21*w2);
  /* #26: @2 = (-@2) */
  w2 = (- w2 );
  /* #27: @2 = (2.*@2) */
  w2 = (2.* w2 );
  /* #28: @2 = (@17*@2) */
  w2  = (w17*w2);
  /* #29: @2 = (@20*@2) */
  w2  = (w20*w2);
  /* #30: @22 = 00 */
  /* #31: @23 = vertcat(@3, @4, @5, @6, @9, @12, @15, @19, @2, @22) */
  rr=w23;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w15;
  *rr++ = w19;
  *rr++ = w2;
  /* #32: @23 = (-@23) */
  for (i=0, rr=w23, cs=w23; i<8; ++i) *rr++ = (- *cs++ );
  /* #33: @24 = @23[:8] */
  for (rr=w24, ss=w23+0; ss!=w23+8; ss+=1) *rr++ = *ss;
  /* #34: (@0[0, 1, 2, 6, 9, 12, 15, 19] = @24) */
  for (cii=casadi_s0, rr=w0, ss=w24; cii!=casadi_s0+8; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #35: @6 = 00 */
  /* #36: @22 = 00 */
  /* #37: @25 = 00 */
  /* #38: @3 = 0.5 */
  w3 = 5.0000000000000000e-01;
  /* #39: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #40: {NULL, NULL, NULL, NULL, @5, NULL, NULL, NULL, NULL, NULL} = vertsplit(@4) */
  w5 = w4;
  /* #41: @4 = (@8*@5) */
  w4  = (w8*w5);
  /* #42: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #43: @4 = (-@4) */
  w4 = (- w4 );
  /* #44: @26 = 00 */
  /* #45: @9 = (@14*@5) */
  w9  = (w14*w5);
  /* #46: @9 = (@10*@9) */
  w9  = (w10*w9);
  /* #47: @9 = (-@9) */
  w9 = (- w9 );
  /* #48: @12 = (@11*@5) */
  w12  = (w11*w5);
  /* #49: @12 = (@13*@12) */
  w12  = (w13*w12);
  /* #50: @15 = input[0][6] */
  w15 = arg[0] ? arg[0][6] : 0;
  /* #51: @19 = (@15*@5) */
  w19  = (w15*w5);
  /* #52: @19 = (2.*@19) */
  w19 = (2.* w19 );
  /* #53: @19 = (@17*@19) */
  w19  = (w17*w19);
  /* #54: @19 = (@16*@19) */
  w19  = (w16*w19);
  /* #55: @2 = input[0][3] */
  w2 = arg[0] ? arg[0][3] : 0;
  /* #56: @27 = (@2*@5) */
  w27  = (w2*w5);
  /* #57: @27 = (-@27) */
  w27 = (- w27 );
  /* #58: @27 = (2.*@27) */
  w27 = (2.* w27 );
  /* #59: @27 = (@17*@27) */
  w27  = (w17*w27);
  /* #60: @27 = (@20*@27) */
  w27  = (w20*w27);
  /* #61: @28 = 30.303 */
  w28 = 3.0303030303030301e+01;
  /* #62: @29 = (2.*@5) */
  w29 = (2.* w5 );
  /* #63: @29 = (@21*@29) */
  w29  = (w21*w29);
  /* #64: @30 = (2.*@21) */
  w30 = (2.* w21 );
  /* #65: @5 = (@30*@5) */
  w5  = (w30*w5);
  /* #66: @29 = (@29+@5) */
  w29 += w5;
  /* #67: @29 = (@17*@29) */
  w29  = (w17*w29);
  /* #68: @29 = (@28*@29) */
  w29  = (w28*w29);
  /* #69: @29 = (-@29) */
  w29 = (- w29 );
  /* #70: @31 = vertcat(@6, @22, @25, @4, @26, @9, @12, @19, @27, @29) */
  rr=w31;
  *rr++ = w4;
  *rr++ = w9;
  *rr++ = w12;
  *rr++ = w19;
  *rr++ = w27;
  *rr++ = w29;
  /* #71: @31 = (-@31) */
  for (i=0, rr=w31, cs=w31; i<6; ++i) *rr++ = (- *cs++ );
  /* #72: @32 = @31[:6] */
  for (rr=w32, ss=w31+0; ss!=w31+6; ss+=1) *rr++ = *ss;
  /* #73: (@0[3, 10, 13, 16, 20, 23] = @32) */
  for (cii=casadi_s1, rr=w0, ss=w32; cii!=casadi_s1+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #74: @6 = 00 */
  /* #75: @22 = 00 */
  /* #76: @25 = 00 */
  /* #77: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #78: {NULL, NULL, NULL, NULL, NULL, @9, NULL, NULL, NULL, NULL} = vertsplit(@4) */
  w9 = w4;
  /* #79: @4 = (@11*@9) */
  w4  = (w11*w9);
  /* #80: @4 = (@3*@4) */
  w4  = (w3*w4);
  /* #81: @4 = (-@4) */
  w4 = (- w4 );
  /* #82: @12 = (@14*@9) */
  w12  = (w14*w9);
  /* #83: @12 = (@7*@12) */
  w12  = (w7*w12);
  /* #84: @26 = 00 */
  /* #85: @19 = (@8*@9) */
  w19  = (w8*w9);
  /* #86: @19 = (@13*@19) */
  w19  = (w13*w19);
  /* #87: @19 = (-@19) */
  w19 = (- w19 );
  /* #88: @27 = (@2*@9) */
  w27  = (w2*w9);
  /* #89: @27 = (2.*@27) */
  w27 = (2.* w27 );
  /* #90: @27 = (@17*@27) */
  w27  = (w17*w27);
  /* #91: @27 = (@16*@27) */
  w27  = (w16*w27);
  /* #92: @29 = (@15*@9) */
  w29  = (w15*w9);
  /* #93: @29 = (2.*@29) */
  w29 = (2.* w29 );
  /* #94: @29 = (@17*@29) */
  w29  = (w17*w29);
  /* #95: @29 = (@20*@29) */
  w29  = (w20*w29);
  /* #96: @5 = (2.*@9) */
  w5 = (2.* w9 );
  /* #97: @5 = (@18*@5) */
  w5  = (w18*w5);
  /* #98: @33 = (2.*@18) */
  w33 = (2.* w18 );
  /* #99: @9 = (@33*@9) */
  w9  = (w33*w9);
  /* #100: @5 = (@5+@9) */
  w5 += w9;
  /* #101: @5 = (@17*@5) */
  w5  = (w17*w5);
  /* #102: @28 = (@28*@5) */
  w28 *= w5;
  /* #103: @28 = (-@28) */
  w28 = (- w28 );
  /* #104: @32 = vertcat(@6, @22, @25, @4, @12, @26, @19, @27, @29, @28) */
  rr=w32;
  *rr++ = w4;
  *rr++ = w12;
  *rr++ = w19;
  *rr++ = w27;
  *rr++ = w29;
  *rr++ = w28;
  /* #105: @32 = (-@32) */
  for (i=0, rr=w32, cs=w32; i<6; ++i) *rr++ = (- *cs++ );
  /* #106: @31 = @32[:6] */
  for (rr=w31, ss=w32+0; ss!=w32+6; ss+=1) *rr++ = *ss;
  /* #107: (@0[4, 7, 14, 17, 21, 24] = @31) */
  for (cii=casadi_s2, rr=w0, ss=w31; cii!=casadi_s2+6; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #108: @6 = 00 */
  /* #109: @22 = 00 */
  /* #110: @25 = 00 */
  /* #111: @4 = ones(10x1,1nz) */
  w4 = 1.;
  /* #112: {NULL, NULL, NULL, NULL, NULL, NULL, @12, NULL, NULL, NULL} = vertsplit(@4) */
  w12 = w4;
  /* #113: @14 = (@14*@12) */
  w14 *= w12;
  /* #114: @14 = (@3*@14) */
  w14  = (w3*w14);
  /* #115: @14 = (-@14) */
  w14 = (- w14 );
  /* #116: @11 = (@11*@12) */
  w11 *= w12;
  /* #117: @11 = (@7*@11) */
  w11  = (w7*w11);
  /* #118: @11 = (-@11) */
  w11 = (- w11 );
  /* #119: @8 = (@8*@12) */
  w8 *= w12;
  /* #120: @8 = (@10*@8) */
  w8  = (w10*w8);
  /* #121: @26 = 00 */
  /* #122: @4 = (@21*@12) */
  w4  = (w21*w12);
  /* #123: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #124: @4 = (@17*@4) */
  w4  = (w17*w4);
  /* #125: @16 = (@16*@4) */
  w16 *= w4;
  /* #126: @12 = (@18*@12) */
  w12  = (w18*w12);
  /* #127: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #128: @17 = (@17*@12) */
  w17 *= w12;
  /* #129: @20 = (@20*@17) */
  w20 *= w17;
  /* #130: @34 = 00 */
  /* #131: @35 = vertcat(@6, @22, @25, @14, @11, @8, @26, @16, @20, @34) */
  rr=w35;
  *rr++ = w14;
  *rr++ = w11;
  *rr++ = w8;
  *rr++ = w16;
  *rr++ = w20;
  /* #132: @35 = (-@35) */
  for (i=0, rr=w35, cs=w35; i<5; ++i) *rr++ = (- *cs++ );
  /* #133: @36 = @35[:5] */
  for (rr=w36, ss=w35+0; ss!=w35+5; ss+=1) *rr++ = *ss;
  /* #134: (@0[5, 8, 11, 18, 22] = @36) */
  for (cii=casadi_s3, rr=w0, ss=w36; cii!=casadi_s3+5; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #135: @37 = @0' */
  casadi_trans(w0,casadi_s5, w37, casadi_s4, iw);
  /* #136: output[0][0] = @37 */
  casadi_copy(w37, 25, res[0]);
  /* #137: @38 = zeros(10x10,10nz) */
  casadi_clear(w38, 10);
  /* #138: @39 = ones(10x1) */
  casadi_fill(w39, 10, 1.);
  /* #139: (@38[:10] = @39) */
  for (rr=w38+0, ss=w39; rr!=w38+10; rr+=1) *rr = *ss++;
  /* #140: @39 = @38' */
  casadi_trans(w38,casadi_s6, w39, casadi_s6, iw);
  /* #141: output[1][0] = @39 */
  casadi_copy(w39, 10, res[1]);
  /* #142: @40 = zeros(4x10,15nz) */
  casadi_clear(w40, 15);
  /* #143: @6 = 00 */
  /* #144: @22 = 00 */
  /* #145: @25 = 00 */
  /* #146: @41 = ones(4x1,2nz) */
  casadi_fill(w41, 2, 1.);
  /* #147: {@14, @11, NULL, NULL} = vertsplit(@41) */
  w14 = w41[0];
  w11 = w41[1];
  /* #148: @8 = (@21*@11) */
  w8  = (w21*w11);
  /* #149: @8 = (@3*@8) */
  w8  = (w3*w8);
  /* #150: @8 = (-@8) */
  w8 = (- w8 );
  /* #151: @16 = (@2*@11) */
  w16  = (w2*w11);
  /* #152: @16 = (@7*@16) */
  w16  = (w7*w16);
  /* #153: @20 = (@15*@11) */
  w20  = (w15*w11);
  /* #154: @20 = (@10*@20) */
  w20  = (w10*w20);
  /* #155: @11 = (@18*@11) */
  w11  = (w18*w11);
  /* #156: @11 = (@13*@11) */
  w11  = (w13*w11);
  /* #157: @11 = (-@11) */
  w11 = (- w11 );
  /* #158: @17 = 30.303 */
  w17 = 3.0303030303030301e+01;
  /* #159: @12 = (@2*@18) */
  w12  = (w2*w18);
  /* #160: @4 = (@21*@15) */
  w4  = (w21*w15);
  /* #161: @12 = (@12+@4) */
  w12 += w4;
  /* #162: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #163: @12 = (@12*@14) */
  w12 *= w14;
  /* #164: @17 = (@17*@12) */
  w17 *= w12;
  /* #165: @12 = 30.303 */
  w12 = 3.0303030303030301e+01;
  /* #166: @4 = (@18*@15) */
  w4  = (w18*w15);
  /* #167: @19 = (@2*@21) */
  w19  = (w2*w21);
  /* #168: @4 = (@4-@19) */
  w4 -= w19;
  /* #169: @4 = (2.*@4) */
  w4 = (2.* w4 );
  /* #170: @4 = (@4*@14) */
  w4 *= w14;
  /* #171: @12 = (@12*@4) */
  w12 *= w4;
  /* #172: @4 = 30.303 */
  w4 = 3.0303030303030301e+01;
  /* #173: @19 = 1 */
  w19 = 1.;
  /* #174: @30 = (@30*@21) */
  w30 *= w21;
  /* #175: @19 = (@19-@30) */
  w19 -= w30;
  /* #176: @33 = (@33*@18) */
  w33 *= w18;
  /* #177: @19 = (@19-@33) */
  w19 -= w33;
  /* #178: @19 = (@19*@14) */
  w19 *= w14;
  /* #179: @4 = (@4*@19) */
  w4 *= w19;
  /* #180: @1 = vertcat(@6, @22, @25, @8, @16, @20, @11, @17, @12, @4) */
  rr=w1;
  *rr++ = w8;
  *rr++ = w16;
  *rr++ = w20;
  *rr++ = w11;
  *rr++ = w17;
  *rr++ = w12;
  *rr++ = w4;
  /* #181: @1 = (-@1) */
  for (i=0, rr=w1, cs=w1; i<7; ++i) *rr++ = (- *cs++ );
  /* #182: @42 = @1[:7] */
  for (rr=w42, ss=w1+0; ss!=w1+7; ss+=1) *rr++ = *ss;
  /* #183: (@40[0, 3, 6, 9, 12, 13, 14] = @42) */
  for (cii=casadi_s7, rr=w40, ss=w42; cii!=casadi_s7+7; ++cii, ++ss) if (*cii>=0) rr[*cii] = *ss;
  /* #184: @6 = 00 */
  /* #185: @22 = 00 */
  /* #186: @25 = 00 */
  /* #187: @8 = ones(4x1,1nz) */
  w8 = 1.;
  /* #188: {NULL, NULL, @16, NULL} = vertsplit(@8) */
  w16 = w8;
  /* #189: @8 = (@18*@16) */
  w8  = (w18*w16);
  /* #190: @8 = (@3*@8) */
  w8  = (w3*w8);
  /* #191: @8 = (-@8) */
  w8 = (- w8 );
  /* #192: @20 = (@15*@16) */
  w20  = (w15*w16);
  /* #193: @20 = (@7*@20) */
  w20  = (w7*w20);
  /* #194: @20 = (-@20) */
  w20 = (- w20 );
  /* #195: @11 = (@2*@16) */
  w11  = (w2*w16);
  /* #196: @11 = (@10*@11) */
  w11  = (w10*w11);
  /* #197: @16 = (@21*@16) */
  w16  = (w21*w16);
  /* #198: @16 = (@13*@16) */
  w16  = (w13*w16);
  /* #199: @26 = 00 */
  /* #200: @34 = 00 */
  /* #201: @43 = 00 */
  /* #202: @44 = vertcat(@6, @22, @25, @8, @20, @11, @16, @26, @34, @43) */
  rr=w44;
  *rr++ = w8;
  *rr++ = w20;
  *rr++ = w11;
  *rr++ = w16;
  /* #203: @44 = (-@44) */
  for (i=0, rr=w44, cs=w44; i<4; ++i) *rr++ = (- *cs++ );
  /* #204: @45 = @44[:4] */
  for (rr=w45, ss=w44+0; ss!=w44+4; ss+=1) *rr++ = *ss;
  /* #205: (@40[1:13:3] = @45) */
  for (rr=w40+1, ss=w45; rr!=w40+13; rr+=3) *rr = *ss++;
  /* #206: @6 = 00 */
  /* #207: @22 = 00 */
  /* #208: @25 = 00 */
  /* #209: @8 = ones(4x1,1nz) */
  w8 = 1.;
  /* #210: {NULL, NULL, NULL, @20} = vertsplit(@8) */
  w20 = w8;
  /* #211: @15 = (@15*@20) */
  w15 *= w20;
  /* #212: @3 = (@3*@15) */
  w3 *= w15;
  /* #213: @3 = (-@3) */
  w3 = (- w3 );
  /* #214: @18 = (@18*@20) */
  w18 *= w20;
  /* #215: @7 = (@7*@18) */
  w7 *= w18;
  /* #216: @21 = (@21*@20) */
  w21 *= w20;
  /* #217: @10 = (@10*@21) */
  w10 *= w21;
  /* #218: @10 = (-@10) */
  w10 = (- w10 );
  /* #219: @2 = (@2*@20) */
  w2 *= w20;
  /* #220: @13 = (@13*@2) */
  w13 *= w2;
  /* #221: @26 = 00 */
  /* #222: @34 = 00 */
  /* #223: @43 = 00 */
  /* #224: @45 = vertcat(@6, @22, @25, @3, @7, @10, @13, @26, @34, @43) */
  rr=w45;
  *rr++ = w3;
  *rr++ = w7;
  *rr++ = w10;
  *rr++ = w13;
  /* #225: @45 = (-@45) */
  for (i=0, rr=w45, cs=w45; i<4; ++i) *rr++ = (- *cs++ );
  /* #226: @44 = @45[:4] */
  for (rr=w44, ss=w45+0; ss!=w45+4; ss+=1) *rr++ = *ss;
  /* #227: (@40[2:14:3] = @44) */
  for (rr=w40+2, ss=w44; rr!=w40+14; rr+=3) *rr = *ss++;
  /* #228: @46 = @40' */
  casadi_trans(w40,casadi_s9, w46, casadi_s8, iw);
  /* #229: output[2][0] = @46 */
  casadi_copy(w46, 15, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_jac_x_xdot_u_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_jac_x_xdot_u_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_impl_dae_jac_x_xdot_u_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_jac_x_xdot_u_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_jac_x_xdot_u_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s10;
    case 1: return casadi_s10;
    case 2: return casadi_s11;
    case 3: return casadi_s12;
    case 4: return casadi_s12;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s6;
    case 2: return casadi_s8;
    case 3: return casadi_s13;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 15;
  if (sz_res) *sz_res = 14;
  if (sz_iw) *sz_iw = 11;
  if (sz_w) *sz_w = 186;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
