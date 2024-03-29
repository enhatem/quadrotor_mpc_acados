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
  #define CASADI_PREFIX(ID) Translational_drone_expl_vde_adj_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)

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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};
static const casadi_int casadi_s3[15] = {14, 1, 0, 11, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};

/* Translational_drone_expl_vde_adj:(i0[10],i1[10],i2[4],i3[])->(o0[14x1,11nz]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, *w3=w+3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, w20, w21, w22, w23;
  /* #0: @0 = input[0][5] */
  w0 = arg[0] ? arg[0][5] : 0;
  /* #1: @1 = input[2][0] */
  w1 = arg[2] ? arg[2][0] : 0;
  /* #2: @2 = 34.4828 */
  w2 = 3.4482758620689651e+01;
  /* #3: @3 = input[1][0] */
  casadi_copy(arg[1], 10, w3);
  /* #4: {@4, @5, @6, @7, @8, @9, @10, @11, @12, @13} = vertsplit(@3) */
  w4 = w3[0];
  w5 = w3[1];
  w6 = w3[2];
  w7 = w3[3];
  w8 = w3[4];
  w9 = w3[5];
  w10 = w3[6];
  w11 = w3[7];
  w12 = w3[8];
  w13 = w3[9];
  /* #5: @2 = (@2*@11) */
  w2 *= w11;
  /* #6: @11 = (@1*@2) */
  w11  = (w1*w2);
  /* #7: @11 = (2.*@11) */
  w11 = (2.* w11 );
  /* #8: @14 = (@0*@11) */
  w14  = (w0*w11);
  /* #9: @15 = input[0][4] */
  w15 = arg[0] ? arg[0][4] : 0;
  /* #10: @16 = 34.4828 */
  w16 = 3.4482758620689651e+01;
  /* #11: @16 = (@16*@12) */
  w16 *= w12;
  /* #12: @12 = (@1*@16) */
  w12  = (w1*w16);
  /* #13: @12 = (2.*@12) */
  w12 = (2.* w12 );
  /* #14: @17 = (@15*@12) */
  w17  = (w15*w12);
  /* #15: @14 = (@14-@17) */
  w14 -= w17;
  /* #16: @17 = input[2][3] */
  w17 = arg[2] ? arg[2][3] : 0;
  /* #17: @18 = 0.5 */
  w18 = 5.0000000000000000e-01;
  /* #18: @18 = (@18*@10) */
  w18 *= w10;
  /* #19: @10 = (@17*@18) */
  w10  = (w17*w18);
  /* #20: @14 = (@14+@10) */
  w14 += w10;
  /* #21: @10 = input[2][2] */
  w10 = arg[2] ? arg[2][2] : 0;
  /* #22: @19 = 0.5 */
  w19 = 5.0000000000000000e-01;
  /* #23: @19 = (@19*@9) */
  w19 *= w9;
  /* #24: @9 = (@10*@19) */
  w9  = (w10*w19);
  /* #25: @14 = (@14+@9) */
  w14 += w9;
  /* #26: @9 = input[2][1] */
  w9 = arg[2] ? arg[2][1] : 0;
  /* #27: @20 = 0.5 */
  w20 = 5.0000000000000000e-01;
  /* #28: @20 = (@20*@8) */
  w20 *= w8;
  /* #29: @8 = (@9*@20) */
  w8  = (w9*w20);
  /* #30: @14 = (@14+@8) */
  w14 += w8;
  /* #31: output[0][0] = @14 */
  if (res[0]) res[0][0] = w14;
  /* #32: @14 = 34.4828 */
  w14 = 3.4482758620689651e+01;
  /* #33: @14 = (@14*@13) */
  w14 *= w13;
  /* #34: @1 = (@1*@14) */
  w1 *= w14;
  /* #35: @13 = (@15*@1) */
  w13  = (w15*w1);
  /* #36: @13 = (-@13) */
  w13 = (- w13 );
  /* #37: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #38: @8 = (2.*@15) */
  w8 = (2.* w15 );
  /* #39: @21 = (@8*@1) */
  w21  = (w8*w1);
  /* #40: @13 = (@13-@21) */
  w13 -= w21;
  /* #41: @21 = input[0][3] */
  w21 = arg[0] ? arg[0][3] : 0;
  /* #42: @22 = (@21*@12) */
  w22  = (w21*w12);
  /* #43: @13 = (@13-@22) */
  w13 -= w22;
  /* #44: @22 = input[0][6] */
  w22 = arg[0] ? arg[0][6] : 0;
  /* #45: @23 = (@22*@11) */
  w23  = (w22*w11);
  /* #46: @13 = (@13+@23) */
  w13 += w23;
  /* #47: @23 = (@10*@18) */
  w23  = (w10*w18);
  /* #48: @13 = (@13+@23) */
  w13 += w23;
  /* #49: @23 = (@17*@19) */
  w23  = (w17*w19);
  /* #50: @13 = (@13-@23) */
  w13 -= w23;
  /* #51: @23 = 0.5 */
  w23 = 5.0000000000000000e-01;
  /* #52: @23 = (@23*@7) */
  w23 *= w7;
  /* #53: @7 = (@9*@23) */
  w7  = (w9*w23);
  /* #54: @13 = (@13-@7) */
  w13 -= w7;
  /* #55: output[0][1] = @13 */
  if (res[0]) res[0][1] = w13;
  /* #56: @13 = (@0*@1) */
  w13  = (w0*w1);
  /* #57: @13 = (-@13) */
  w13 = (- w13 );
  /* #58: @13 = (2.*@13) */
  w13 = (2.* w13 );
  /* #59: @7 = (2.*@0) */
  w7 = (2.* w0 );
  /* #60: @1 = (@7*@1) */
  w1  = (w7*w1);
  /* #61: @13 = (@13-@1) */
  w13 -= w1;
  /* #62: @1 = (@22*@12) */
  w1  = (w22*w12);
  /* #63: @13 = (@13+@1) */
  w13 += w1;
  /* #64: @1 = (@21*@11) */
  w1  = (w21*w11);
  /* #65: @13 = (@13+@1) */
  w13 += w1;
  /* #66: @1 = (@9*@18) */
  w1  = (w9*w18);
  /* #67: @13 = (@13-@1) */
  w13 -= w1;
  /* #68: @1 = (@17*@20) */
  w1  = (w17*w20);
  /* #69: @13 = (@13+@1) */
  w13 += w1;
  /* #70: @1 = (@10*@23) */
  w1  = (w10*w23);
  /* #71: @13 = (@13-@1) */
  w13 -= w1;
  /* #72: output[0][2] = @13 */
  if (res[0]) res[0][2] = w13;
  /* #73: @12 = (@0*@12) */
  w12  = (w0*w12);
  /* #74: @11 = (@15*@11) */
  w11  = (w15*w11);
  /* #75: @12 = (@12+@11) */
  w12 += w11;
  /* #76: @9 = (@9*@19) */
  w9 *= w19;
  /* #77: @12 = (@12+@9) */
  w12 += w9;
  /* #78: @10 = (@10*@20) */
  w10 *= w20;
  /* #79: @12 = (@12-@10) */
  w12 -= w10;
  /* #80: @17 = (@17*@23) */
  w17 *= w23;
  /* #81: @12 = (@12-@17) */
  w12 -= w17;
  /* #82: output[0][3] = @12 */
  if (res[0]) res[0][3] = w12;
  /* #83: output[0][4] = @4 */
  if (res[0]) res[0][4] = w4;
  /* #84: output[0][5] = @5 */
  if (res[0]) res[0][5] = w5;
  /* #85: output[0][6] = @6 */
  if (res[0]) res[0][6] = w6;
  /* #86: @6 = 1 */
  w6 = 1.;
  /* #87: @8 = (@8*@15) */
  w8 *= w15;
  /* #88: @6 = (@6-@8) */
  w6 -= w8;
  /* #89: @7 = (@7*@0) */
  w7 *= w0;
  /* #90: @6 = (@6-@7) */
  w6 -= w7;
  /* #91: @6 = (@6*@14) */
  w6 *= w14;
  /* #92: @14 = (@0*@22) */
  w14  = (w0*w22);
  /* #93: @7 = (@21*@15) */
  w7  = (w21*w15);
  /* #94: @14 = (@14-@7) */
  w14 -= w7;
  /* #95: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #96: @14 = (@14*@16) */
  w14 *= w16;
  /* #97: @6 = (@6+@14) */
  w6 += w14;
  /* #98: @14 = (@21*@0) */
  w14  = (w21*w0);
  /* #99: @16 = (@15*@22) */
  w16  = (w15*w22);
  /* #100: @14 = (@14+@16) */
  w14 += w16;
  /* #101: @14 = (2.*@14) */
  w14 = (2.* w14 );
  /* #102: @14 = (@14*@2) */
  w14 *= w2;
  /* #103: @6 = (@6+@14) */
  w6 += w14;
  /* #104: output[0][7] = @6 */
  if (res[0]) res[0][7] = w6;
  /* #105: @6 = (@22*@19) */
  w6  = (w22*w19);
  /* #106: @14 = (@0*@18) */
  w14  = (w0*w18);
  /* #107: @6 = (@6-@14) */
  w6 -= w14;
  /* #108: @14 = (@21*@20) */
  w14  = (w21*w20);
  /* #109: @6 = (@6+@14) */
  w6 += w14;
  /* #110: @14 = (@15*@23) */
  w14  = (w15*w23);
  /* #111: @6 = (@6-@14) */
  w6 -= w14;
  /* #112: output[0][8] = @6 */
  if (res[0]) res[0][8] = w6;
  /* #113: @6 = (@15*@18) */
  w6  = (w15*w18);
  /* #114: @14 = (@21*@19) */
  w14  = (w21*w19);
  /* #115: @6 = (@6+@14) */
  w6 += w14;
  /* #116: @14 = (@22*@20) */
  w14  = (w22*w20);
  /* #117: @6 = (@6-@14) */
  w6 -= w14;
  /* #118: @14 = (@0*@23) */
  w14  = (w0*w23);
  /* #119: @6 = (@6-@14) */
  w6 -= w14;
  /* #120: output[0][9] = @6 */
  if (res[0]) res[0][9] = w6;
  /* #121: @21 = (@21*@18) */
  w21 *= w18;
  /* #122: @15 = (@15*@19) */
  w15 *= w19;
  /* #123: @21 = (@21-@15) */
  w21 -= w15;
  /* #124: @0 = (@0*@20) */
  w0 *= w20;
  /* #125: @21 = (@21+@0) */
  w21 += w0;
  /* #126: @22 = (@22*@23) */
  w22 *= w23;
  /* #127: @21 = (@21-@22) */
  w21 -= w22;
  /* #128: output[0][10] = @21 */
  if (res[0]) res[0][10] = w21;
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_vde_adj(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_vde_adj_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_vde_adj_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_vde_adj_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_vde_adj_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_vde_adj_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_vde_adj_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_vde_adj_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_expl_vde_adj_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_expl_vde_adj_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_expl_vde_adj_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_expl_vde_adj_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_expl_vde_adj_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_expl_vde_adj_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    case 2: return casadi_s1;
    case 3: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_expl_vde_adj_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_vde_adj_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 6;
  if (sz_res) *sz_res = 11;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 33;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
