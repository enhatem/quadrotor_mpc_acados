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
  #define CASADI_PREFIX(ID) Translational_drone_expl_ode_fun_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)

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

static const casadi_int casadi_s0[14] = {10, 1, 0, 10, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
static const casadi_int casadi_s1[8] = {4, 1, 0, 4, 0, 1, 2, 3};
static const casadi_int casadi_s2[3] = {0, 0, 0};

/* Translational_drone_expl_ode_fun:(i0[10],i1[4],i2[])->(o0[10]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9;
  /* #0: @0 = input[0][7] */
  w0 = arg[0] ? arg[0][7] : 0;
  /* #1: output[0][0] = @0 */
  if (res[0]) res[0][0] = w0;
  /* #2: @0 = input[0][8] */
  w0 = arg[0] ? arg[0][8] : 0;
  /* #3: output[0][1] = @0 */
  if (res[0]) res[0][1] = w0;
  /* #4: @0 = input[0][9] */
  w0 = arg[0] ? arg[0][9] : 0;
  /* #5: output[0][2] = @0 */
  if (res[0]) res[0][2] = w0;
  /* #6: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #7: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #8: @2 = input[0][4] */
  w2 = arg[0] ? arg[0][4] : 0;
  /* #9: @3 = (@1*@2) */
  w3  = (w1*w2);
  /* #10: @3 = (-@3) */
  w3 = (- w3 );
  /* #11: @4 = input[1][2] */
  w4 = arg[1] ? arg[1][2] : 0;
  /* #12: @5 = input[0][5] */
  w5 = arg[0] ? arg[0][5] : 0;
  /* #13: @6 = (@4*@5) */
  w6  = (w4*w5);
  /* #14: @3 = (@3-@6) */
  w3 -= w6;
  /* #15: @6 = input[1][3] */
  w6 = arg[1] ? arg[1][3] : 0;
  /* #16: @7 = input[0][6] */
  w7 = arg[0] ? arg[0][6] : 0;
  /* #17: @8 = (@6*@7) */
  w8  = (w6*w7);
  /* #18: @3 = (@3-@8) */
  w3 -= w8;
  /* #19: @0 = (@0*@3) */
  w0 *= w3;
  /* #20: output[0][3] = @0 */
  if (res[0]) res[0][3] = w0;
  /* #21: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #22: @3 = input[0][3] */
  w3 = arg[0] ? arg[0][3] : 0;
  /* #23: @8 = (@1*@3) */
  w8  = (w1*w3);
  /* #24: @9 = (@6*@5) */
  w9  = (w6*w5);
  /* #25: @8 = (@8+@9) */
  w8 += w9;
  /* #26: @9 = (@4*@7) */
  w9  = (w4*w7);
  /* #27: @8 = (@8-@9) */
  w8 -= w9;
  /* #28: @0 = (@0*@8) */
  w0 *= w8;
  /* #29: output[0][4] = @0 */
  if (res[0]) res[0][4] = w0;
  /* #30: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #31: @8 = (@4*@3) */
  w8  = (w4*w3);
  /* #32: @9 = (@6*@2) */
  w9  = (w6*w2);
  /* #33: @8 = (@8-@9) */
  w8 -= w9;
  /* #34: @9 = (@1*@7) */
  w9  = (w1*w7);
  /* #35: @8 = (@8+@9) */
  w8 += w9;
  /* #36: @0 = (@0*@8) */
  w0 *= w8;
  /* #37: output[0][5] = @0 */
  if (res[0]) res[0][5] = w0;
  /* #38: @0 = 0.5 */
  w0 = 5.0000000000000000e-01;
  /* #39: @6 = (@6*@3) */
  w6 *= w3;
  /* #40: @4 = (@4*@2) */
  w4 *= w2;
  /* #41: @6 = (@6+@4) */
  w6 += w4;
  /* #42: @1 = (@1*@5) */
  w1 *= w5;
  /* #43: @6 = (@6-@1) */
  w6 -= w1;
  /* #44: @0 = (@0*@6) */
  w0 *= w6;
  /* #45: output[0][6] = @0 */
  if (res[0]) res[0][6] = w0;
  /* #46: @0 = (@3*@5) */
  w0  = (w3*w5);
  /* #47: @6 = (@2*@7) */
  w6  = (w2*w7);
  /* #48: @0 = (@0+@6) */
  w0 += w6;
  /* #49: @0 = (2.*@0) */
  w0 = (2.* w0 );
  /* #50: @6 = input[1][0] */
  w6 = arg[1] ? arg[1][0] : 0;
  /* #51: @0 = (@0*@6) */
  w0 *= w6;
  /* #52: @1 = 0.027 */
  w1 = 2.7000000000000000e-02;
  /* #53: @0 = (@0/@1) */
  w0 /= w1;
  /* #54: output[0][7] = @0 */
  if (res[0]) res[0][7] = w0;
  /* #55: @7 = (@5*@7) */
  w7  = (w5*w7);
  /* #56: @3 = (@3*@2) */
  w3 *= w2;
  /* #57: @7 = (@7-@3) */
  w7 -= w3;
  /* #58: @7 = (2.*@7) */
  w7 = (2.* w7 );
  /* #59: @7 = (@7*@6) */
  w7 *= w6;
  /* #60: @3 = 0.027 */
  w3 = 2.7000000000000000e-02;
  /* #61: @7 = (@7/@3) */
  w7 /= w3;
  /* #62: output[0][8] = @7 */
  if (res[0]) res[0][8] = w7;
  /* #63: @7 = 1 */
  w7 = 1.;
  /* #64: @3 = (2.*@2) */
  w3 = (2.* w2 );
  /* #65: @3 = (@3*@2) */
  w3 *= w2;
  /* #66: @7 = (@7-@3) */
  w7 -= w3;
  /* #67: @3 = (2.*@5) */
  w3 = (2.* w5 );
  /* #68: @3 = (@3*@5) */
  w3 *= w5;
  /* #69: @7 = (@7-@3) */
  w7 -= w3;
  /* #70: @7 = (@7*@6) */
  w7 *= w6;
  /* #71: @6 = 0.027 */
  w6 = 2.7000000000000000e-02;
  /* #72: @7 = (@7/@6) */
  w7 /= w6;
  /* #73: @6 = 9.81 */
  w6 = 9.8100000000000005e+00;
  /* #74: @7 = (@7-@6) */
  w7 -= w6;
  /* #75: output[0][9] = @7 */
  if (res[0]) res[0][9] = w7;
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_ode_fun(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_ode_fun_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_ode_fun_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_ode_fun_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_ode_fun_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_ode_fun_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_ode_fun_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_expl_ode_fun_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_expl_ode_fun_n_in(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_expl_ode_fun_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_expl_ode_fun_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_expl_ode_fun_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_expl_ode_fun_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_expl_ode_fun_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s1;
    case 2: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_expl_ode_fun_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_expl_ode_fun_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 5;
  if (sz_res) *sz_res = 2;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 10;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
