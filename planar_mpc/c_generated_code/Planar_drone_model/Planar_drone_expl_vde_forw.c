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
  #define CASADI_PREFIX(ID) Planar_drone_expl_vde_forw_ ## ID
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
#define casadi_project CASADI_PREFIX(project)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
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

void casadi_project(const casadi_real* x, const casadi_int* sp_x, casadi_real* y, const casadi_int* sp_y, casadi_real* w) {
  casadi_int ncol_x, ncol_y, i, el;
  const casadi_int *colind_x, *row_x, *colind_y, *row_y;
  ncol_x = sp_x[1];
  colind_x = sp_x+2; row_x = sp_x + 2 + ncol_x+1;
  ncol_y = sp_y[1];
  colind_y = sp_y+2; row_y = sp_y + 2 + ncol_y+1;
  for (i=0; i<ncol_x; ++i) {
    for (el=colind_y[i]; el<colind_y[i+1]; ++el) w[row_y[el]] = 0;
    for (el=colind_x[i]; el<colind_x[i+1]; ++el) w[row_x[el]] = x[el];
    for (el=colind_y[i]; el<colind_y[i+1]; ++el) y[el] = w[row_y[el]];
  }
}

static const casadi_int casadi_s0[8] = {6, 2, 0, 2, 3, 3, 4, 5};
static const casadi_int casadi_s1[12] = {2, 6, 0, 0, 0, 0, 1, 2, 3, 0, 0, 1};
static const casadi_int casadi_s2[16] = {6, 2, 0, 5, 11, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[15] = {6, 2, 0, 5, 10, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};
static const casadi_int casadi_s4[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s5[45] = {6, 6, 0, 6, 12, 18, 24, 30, 36, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s6[17] = {6, 2, 0, 6, 12, 0, 1, 2, 3, 4, 5, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s7[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s8[3] = {0, 0, 0};
static const casadi_int casadi_s9[39] = {6, 6, 0, 5, 10, 15, 20, 25, 30, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4, 0, 1, 2, 3, 4};

/* Planar_drone_expl_vde_forw:(i0[6],i1[6x6],i2[6x2],i3[2],i4[])->(o0[6],o1[6x6,30nz],o2[6x2,11nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real w0, w1, w2, w3, w4, w5, w6, *w7=w+13, *w8=w+49, *w9=w+55, *w10=w+61, *w11=w+67, *w12=w+73, *w13=w+79, w14, w15, w16, w17, *w18=w+89, *w22=w+92, *w23=w+94, *w24=w+97, *w25=w+100, *w26=w+111, *w27=w+123, *w28=w+128, *w29=w+133, *w30=w+143;
  /* #0: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][4] */
  w0 = arg[0] ? arg[0][4] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = input[3][0] */
  w0 = arg[3] ? arg[3][0] : 0;
  /* #7: @1 = 0.0135 */
  w1 = 1.3500000000000000e-02;
  /* #8: @1 = (@0/@1) */
  w1  = (w0/w1);
  /* #9: @2 = input[0][2] */
  w2 = arg[0] ? arg[0][2] : 0;
  /* #10: @3 = sin(@2) */
  w3 = sin( w2 );
  /* #11: @4 = (@1*@3) */
  w4  = (w1*w3);
  /* #12: @4 = (-@4) */
  w4 = (- w4 );
  /* #13: output[0][3] = @4 */
  if (res[0]) res[0][3] = w4;
  /* #14: @4 = -9.81 */
  w4 = -9.8100000000000005e+00;
  /* #15: @5 = 0.0135 */
  w5 = 1.3500000000000000e-02;
  /* #16: @0 = (@0/@5) */
  w0 /= w5;
  /* #17: @5 = cos(@2) */
  w5 = cos( w2 );
  /* #18: @6 = (@0*@5) */
  w6  = (w0*w5);
  /* #19: @4 = (@4+@6) */
  w4 += w6;
  /* #20: output[0][4] = @4 */
  if (res[0]) res[0][4] = w4;
  /* #21: @4 = input[3][1] */
  w4 = arg[3] ? arg[3][1] : 0;
  /* #22: @6 = 1.65717e-05 */
  w6 = 1.6571710000000000e-05;
  /* #23: @4 = (@4/@6) */
  w4 /= w6;
  /* #24: output[0][5] = @4 */
  if (res[0]) res[0][5] = w4;
  /* #25: @7 = input[1][0] */
  casadi_copy(arg[1], 36, w7);
  /* #26: {@8, @9, @10, @11, @12, @13} = horzsplit(@7) */
  casadi_copy(w7, 6, w8);
  casadi_copy(w7+6, 6, w9);
  casadi_copy(w7+12, 6, w10);
  casadi_copy(w7+18, 6, w11);
  casadi_copy(w7+24, 6, w12);
  casadi_copy(w7+30, 6, w13);
  /* #27: {NULL, NULL, @4, @6, @14, @15} = vertsplit(@8) */
  w4 = w8[2];
  w6 = w8[3];
  w14 = w8[4];
  w15 = w8[5];
  /* #28: output[1][0] = @6 */
  if (res[1]) res[1][0] = w6;
  /* #29: output[1][1] = @14 */
  if (res[1]) res[1][1] = w14;
  /* #30: output[1][2] = @15 */
  if (res[1]) res[1][2] = w15;
  /* #31: @15 = cos(@2) */
  w15 = cos( w2 );
  /* #32: @14 = (@15*@4) */
  w14  = (w15*w4);
  /* #33: @14 = (@1*@14) */
  w14  = (w1*w14);
  /* #34: @14 = (-@14) */
  w14 = (- w14 );
  /* #35: output[1][3] = @14 */
  if (res[1]) res[1][3] = w14;
  /* #36: @14 = sin(@2) */
  w14 = sin( w2 );
  /* #37: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #38: @4 = (@0*@4) */
  w4  = (w0*w4);
  /* #39: @4 = (-@4) */
  w4 = (- w4 );
  /* #40: output[1][4] = @4 */
  if (res[1]) res[1][4] = w4;
  /* #41: {NULL, NULL, @4, @6, @16, @17} = vertsplit(@9) */
  w4 = w9[2];
  w6 = w9[3];
  w16 = w9[4];
  w17 = w9[5];
  /* #42: output[1][5] = @6 */
  if (res[1]) res[1][5] = w6;
  /* #43: output[1][6] = @16 */
  if (res[1]) res[1][6] = w16;
  /* #44: output[1][7] = @17 */
  if (res[1]) res[1][7] = w17;
  /* #45: @17 = (@15*@4) */
  w17  = (w15*w4);
  /* #46: @17 = (@1*@17) */
  w17  = (w1*w17);
  /* #47: @17 = (-@17) */
  w17 = (- w17 );
  /* #48: output[1][8] = @17 */
  if (res[1]) res[1][8] = w17;
  /* #49: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #50: @4 = (@0*@4) */
  w4  = (w0*w4);
  /* #51: @4 = (-@4) */
  w4 = (- w4 );
  /* #52: output[1][9] = @4 */
  if (res[1]) res[1][9] = w4;
  /* #53: {NULL, NULL, @4, @17, @16, @6} = vertsplit(@10) */
  w4 = w10[2];
  w17 = w10[3];
  w16 = w10[4];
  w6 = w10[5];
  /* #54: output[1][10] = @17 */
  if (res[1]) res[1][10] = w17;
  /* #55: output[1][11] = @16 */
  if (res[1]) res[1][11] = w16;
  /* #56: output[1][12] = @6 */
  if (res[1]) res[1][12] = w6;
  /* #57: @6 = (@15*@4) */
  w6  = (w15*w4);
  /* #58: @6 = (@1*@6) */
  w6  = (w1*w6);
  /* #59: @6 = (-@6) */
  w6 = (- w6 );
  /* #60: output[1][13] = @6 */
  if (res[1]) res[1][13] = w6;
  /* #61: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #62: @4 = (@0*@4) */
  w4  = (w0*w4);
  /* #63: @4 = (-@4) */
  w4 = (- w4 );
  /* #64: output[1][14] = @4 */
  if (res[1]) res[1][14] = w4;
  /* #65: {NULL, NULL, @4, @6, @16, @17} = vertsplit(@11) */
  w4 = w11[2];
  w6 = w11[3];
  w16 = w11[4];
  w17 = w11[5];
  /* #66: output[1][15] = @6 */
  if (res[1]) res[1][15] = w6;
  /* #67: output[1][16] = @16 */
  if (res[1]) res[1][16] = w16;
  /* #68: output[1][17] = @17 */
  if (res[1]) res[1][17] = w17;
  /* #69: @17 = (@15*@4) */
  w17  = (w15*w4);
  /* #70: @17 = (@1*@17) */
  w17  = (w1*w17);
  /* #71: @17 = (-@17) */
  w17 = (- w17 );
  /* #72: output[1][18] = @17 */
  if (res[1]) res[1][18] = w17;
  /* #73: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #74: @4 = (@0*@4) */
  w4  = (w0*w4);
  /* #75: @4 = (-@4) */
  w4 = (- w4 );
  /* #76: output[1][19] = @4 */
  if (res[1]) res[1][19] = w4;
  /* #77: {NULL, NULL, @4, @17, @16, @6} = vertsplit(@12) */
  w4 = w12[2];
  w17 = w12[3];
  w16 = w12[4];
  w6 = w12[5];
  /* #78: output[1][20] = @17 */
  if (res[1]) res[1][20] = w17;
  /* #79: output[1][21] = @16 */
  if (res[1]) res[1][21] = w16;
  /* #80: output[1][22] = @6 */
  if (res[1]) res[1][22] = w6;
  /* #81: @6 = (@15*@4) */
  w6  = (w15*w4);
  /* #82: @6 = (@1*@6) */
  w6  = (w1*w6);
  /* #83: @6 = (-@6) */
  w6 = (- w6 );
  /* #84: output[1][23] = @6 */
  if (res[1]) res[1][23] = w6;
  /* #85: @4 = (@14*@4) */
  w4  = (w14*w4);
  /* #86: @4 = (@0*@4) */
  w4  = (w0*w4);
  /* #87: @4 = (-@4) */
  w4 = (- w4 );
  /* #88: output[1][24] = @4 */
  if (res[1]) res[1][24] = w4;
  /* #89: {NULL, NULL, @4, @6, @16, @17} = vertsplit(@13) */
  w4 = w13[2];
  w6 = w13[3];
  w16 = w13[4];
  w17 = w13[5];
  /* #90: output[1][25] = @6 */
  if (res[1]) res[1][25] = w6;
  /* #91: output[1][26] = @16 */
  if (res[1]) res[1][26] = w16;
  /* #92: output[1][27] = @17 */
  if (res[1]) res[1][27] = w17;
  /* #93: @15 = (@15*@4) */
  w15 *= w4;
  /* #94: @15 = (@1*@15) */
  w15  = (w1*w15);
  /* #95: @15 = (-@15) */
  w15 = (- w15 );
  /* #96: output[1][28] = @15 */
  if (res[1]) res[1][28] = w15;
  /* #97: @14 = (@14*@4) */
  w14 *= w4;
  /* #98: @14 = (@0*@14) */
  w14  = (w0*w14);
  /* #99: @14 = (-@14) */
  w14 = (- w14 );
  /* #100: output[1][29] = @14 */
  if (res[1]) res[1][29] = w14;
  /* #101: @18 = zeros(2x6,3nz) */
  casadi_clear(w18, 3);
  /* #102: @19 = 00 */
  /* #103: @20 = 00 */
  /* #104: @21 = 00 */
  /* #105: @14 = 74.0741 */
  w14 = 7.4074074074074076e+01;
  /* #106: @22 = ones(2x1) */
  casadi_fill(w22, 2, 1.);
  /* #107: {@4, @15} = vertsplit(@22) */
  w4 = w22[0];
  w15 = w22[1];
  /* #108: @14 = (@14*@4) */
  w14 *= w4;
  /* #109: @3 = (@3*@14) */
  w3 *= w14;
  /* #110: @3 = (-@3) */
  w3 = (- w3 );
  /* #111: @14 = 74.0741 */
  w14 = 7.4074074074074076e+01;
  /* #112: @14 = (@14*@4) */
  w14 *= w4;
  /* #113: @5 = (@5*@14) */
  w5 *= w14;
  /* #114: @14 = 60343.8 */
  w14 = 6.0343802781969993e+04;
  /* #115: @14 = (@14*@15) */
  w14 *= w15;
  /* #116: @23 = vertcat(@19, @20, @21, @3, @5, @14) */
  rr=w23;
  *rr++ = w3;
  *rr++ = w5;
  *rr++ = w14;
  /* #117: @24 = @23[:3] */
  for (rr=w24, ss=w23+0; ss!=w23+3; ss+=1) *rr++ = *ss;
  /* #118: (@18[:3] = @24) */
  for (rr=w18+0, ss=w24; rr!=w18+3; rr+=1) *rr = *ss++;
  /* #119: @24 = @18' */
  casadi_trans(w18,casadi_s1, w24, casadi_s0, iw);
  /* #120: @25 = project(@24) */
  casadi_project(w24, casadi_s0, w25, casadi_s2, w);
  /* #121: @26 = input[2][0] */
  casadi_copy(arg[2], 12, w26);
  /* #122: {@13, @12} = horzsplit(@26) */
  casadi_copy(w26, 6, w13);
  casadi_copy(w26+6, 6, w12);
  /* #123: {NULL, NULL, @3, @5, @14, @15} = vertsplit(@13) */
  w3 = w13[2];
  w5 = w13[3];
  w14 = w13[4];
  w15 = w13[5];
  /* #124: @4 = cos(@2) */
  w4 = cos( w2 );
  /* #125: @17 = (@4*@3) */
  w17  = (w4*w3);
  /* #126: @17 = (@1*@17) */
  w17  = (w1*w17);
  /* #127: @17 = (-@17) */
  w17 = (- w17 );
  /* #128: @2 = sin(@2) */
  w2 = sin( w2 );
  /* #129: @3 = (@2*@3) */
  w3  = (w2*w3);
  /* #130: @3 = (@0*@3) */
  w3  = (w0*w3);
  /* #131: @3 = (-@3) */
  w3 = (- w3 );
  /* #132: @19 = 00 */
  /* #133: @27 = vertcat(@5, @14, @15, @17, @3, @19) */
  rr=w27;
  *rr++ = w5;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w17;
  *rr++ = w3;
  /* #134: {NULL, NULL, @5, @14, @15, @17} = vertsplit(@12) */
  w5 = w12[2];
  w14 = w12[3];
  w15 = w12[4];
  w17 = w12[5];
  /* #135: @4 = (@4*@5) */
  w4 *= w5;
  /* #136: @1 = (@1*@4) */
  w1 *= w4;
  /* #137: @1 = (-@1) */
  w1 = (- w1 );
  /* #138: @2 = (@2*@5) */
  w2 *= w5;
  /* #139: @0 = (@0*@2) */
  w0 *= w2;
  /* #140: @0 = (-@0) */
  w0 = (- w0 );
  /* #141: @19 = 00 */
  /* #142: @28 = vertcat(@14, @15, @17, @1, @0, @19) */
  rr=w28;
  *rr++ = w14;
  *rr++ = w15;
  *rr++ = w17;
  *rr++ = w1;
  *rr++ = w0;
  /* #143: @29 = horzcat(@27, @28) */
  rr=w29;
  for (i=0, cs=w27; i<5; ++i) *rr++ = *cs++;
  for (i=0, cs=w28; i<5; ++i) *rr++ = *cs++;
  /* #144: @30 = project(@29) */
  casadi_project(w29, casadi_s3, w30, casadi_s2, w);
  /* #145: @25 = (@25+@30) */
  for (i=0, rr=w25, cs=w30; i<11; ++i) (*rr++) += (*cs++);
  /* #146: output[2][0] = @25 */
  casadi_copy(w25, 11, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_expl_vde_forw(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Planar_drone_expl_vde_forw_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_expl_vde_forw_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_expl_vde_forw_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Planar_drone_expl_vde_forw_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_expl_vde_forw_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_expl_vde_forw_incref(void) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_expl_vde_forw_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_expl_vde_forw_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_expl_vde_forw_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real Planar_drone_expl_vde_forw_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_expl_vde_forw_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_expl_vde_forw_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_expl_vde_forw_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    case 3: return casadi_s7;
    case 4: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_expl_vde_forw_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s4;
    case 1: return casadi_s9;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Planar_drone_expl_vde_forw_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 9;
  if (sz_iw) *sz_iw = 3;
  if (sz_w) *sz_w = 154;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
