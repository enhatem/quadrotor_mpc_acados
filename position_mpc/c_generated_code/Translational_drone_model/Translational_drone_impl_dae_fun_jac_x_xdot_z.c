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
  #define CASADI_PREFIX(ID) Translational_drone_impl_dae_fun_jac_x_xdot_z_ ## ID
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
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
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

static const casadi_int casadi_s0[12] = {6, 6, 0, 0, 0, 0, 1, 2, 3, 0, 1, 2};
static const casadi_int casadi_s1[12] = {6, 6, 0, 1, 2, 3, 3, 3, 3, 3, 4, 5};
static const casadi_int casadi_s2[15] = {6, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s4[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s5[3] = {0, 0, 0};
static const casadi_int casadi_s6[3] = {6, 0, 0};

/* Translational_drone_impl_dae_fun_jac_x_xdot_z:(i0[6],i1[6],i2[5],i3[],i4[])->(o0[6],o1[6x6,3nz],o2[6x6,6nz],o3[6x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real w0, w1, w2, w3, w4, w5, *w6=w+6, w7, w8, w9, w10, *w11=w+16, *w12=w+22, *w16=w+25, *w17=w+28;
  /* #0: @0 = input[1][0] */
  w0 = arg[1] ? arg[1][0] : 0;
  /* #1: @1 = input[1][1] */
  w1 = arg[1] ? arg[1][1] : 0;
  /* #2: @2 = input[1][2] */
  w2 = arg[1] ? arg[1][2] : 0;
  /* #3: @3 = input[1][3] */
  w3 = arg[1] ? arg[1][3] : 0;
  /* #4: @4 = input[1][4] */
  w4 = arg[1] ? arg[1][4] : 0;
  /* #5: @5 = input[1][5] */
  w5 = arg[1] ? arg[1][5] : 0;
  /* #6: @6 = vertcat(@0, @1, @2, @3, @4, @5) */
  rr=w6;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  /* #7: @0 = input[0][3] */
  w0 = arg[0] ? arg[0][3] : 0;
  /* #8: @1 = input[0][4] */
  w1 = arg[0] ? arg[0][4] : 0;
  /* #9: @2 = input[0][5] */
  w2 = arg[0] ? arg[0][5] : 0;
  /* #10: @3 = input[2][1] */
  w3 = arg[2] ? arg[2][1] : 0;
  /* #11: @4 = input[2][3] */
  w4 = arg[2] ? arg[2][3] : 0;
  /* #12: @5 = (@3*@4) */
  w5  = (w3*w4);
  /* #13: @7 = input[2][2] */
  w7 = arg[2] ? arg[2][2] : 0;
  /* #14: @8 = input[2][4] */
  w8 = arg[2] ? arg[2][4] : 0;
  /* #15: @9 = (@7*@8) */
  w9  = (w7*w8);
  /* #16: @5 = (@5+@9) */
  w5 += w9;
  /* #17: @5 = (2.*@5) */
  w5 = (2.* w5 );
  /* #18: @9 = input[2][0] */
  w9 = arg[2] ? arg[2][0] : 0;
  /* #19: @5 = (@5*@9) */
  w5 *= w9;
  /* #20: @8 = (@4*@8) */
  w8  = (w4*w8);
  /* #21: @3 = (@3*@7) */
  w3 *= w7;
  /* #22: @8 = (@8-@3) */
  w8 -= w3;
  /* #23: @8 = (2.*@8) */
  w8 = (2.* w8 );
  /* #24: @8 = (@8*@9) */
  w8 *= w9;
  /* #25: @3 = 1 */
  w3 = 1.;
  /* #26: @10 = (2.*@7) */
  w10 = (2.* w7 );
  /* #27: @10 = (@10*@7) */
  w10 *= w7;
  /* #28: @3 = (@3-@10) */
  w3 -= w10;
  /* #29: @10 = (2.*@4) */
  w10 = (2.* w4 );
  /* #30: @10 = (@10*@4) */
  w10 *= w4;
  /* #31: @3 = (@3-@10) */
  w3 -= w10;
  /* #32: @3 = (@3*@9) */
  w3 *= w9;
  /* #33: @9 = 9.81 */
  w9 = 9.8100000000000005e+00;
  /* #34: @3 = (@3-@9) */
  w3 -= w9;
  /* #35: @11 = vertcat(@0, @1, @2, @5, @8, @3) */
  rr=w11;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  *rr++ = w5;
  *rr++ = w8;
  *rr++ = w3;
  /* #36: @6 = (@6-@11) */
  for (i=0, rr=w6, cs=w11; i<6; ++i) (*rr++) -= (*cs++);
  /* #37: output[0][0] = @6 */
  casadi_copy(w6, 6, res[0]);
  /* #38: @12 = zeros(6x6,3nz) */
  casadi_clear(w12, 3);
  /* #39: @6 = ones(6x1) */
  casadi_fill(w6, 6, 1.);
  /* #40: {NULL, NULL, NULL, @0, @1, @2} = vertsplit(@6) */
  w0 = w6[3];
  w1 = w6[4];
  w2 = w6[5];
  /* #41: @13 = 00 */
  /* #42: @14 = 00 */
  /* #43: @15 = 00 */
  /* #44: @16 = vertcat(@0, @1, @2, @13, @14, @15) */
  rr=w16;
  *rr++ = w0;
  *rr++ = w1;
  *rr++ = w2;
  /* #45: @16 = (-@16) */
  for (i=0, rr=w16, cs=w16; i<3; ++i) *rr++ = (- *cs++ );
  /* #46: @17 = @16[:3] */
  for (rr=w17, ss=w16+0; ss!=w16+3; ss+=1) *rr++ = *ss;
  /* #47: (@12[:3] = @17) */
  for (rr=w12+0, ss=w17; rr!=w12+3; rr+=1) *rr = *ss++;
  /* #48: @17 = @12' */
  casadi_trans(w12,casadi_s1, w17, casadi_s0, iw);
  /* #49: output[1][0] = @17 */
  casadi_copy(w17, 3, res[1]);
  /* #50: @6 = zeros(6x6,6nz) */
  casadi_clear(w6, 6);
  /* #51: @11 = ones(6x1) */
  casadi_fill(w11, 6, 1.);
  /* #52: (@6[:6] = @11) */
  for (rr=w6+0, ss=w11; rr!=w6+6; rr+=1) *rr = *ss++;
  /* #53: @11 = @6' */
  casadi_trans(w6,casadi_s2, w11, casadi_s2, iw);
  /* #54: output[2][0] = @11 */
  casadi_copy(w11, 6, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void Translational_drone_impl_dae_fun_jac_x_xdot_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_fun_jac_x_xdot_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Translational_drone_impl_dae_fun_jac_x_xdot_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real Translational_drone_impl_dae_fun_jac_x_xdot_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_fun_jac_x_xdot_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Translational_drone_impl_dae_fun_jac_x_xdot_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_fun_jac_x_xdot_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    case 3: return casadi_s5;
    case 4: return casadi_s5;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Translational_drone_impl_dae_fun_jac_x_xdot_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s3;
    case 1: return casadi_s0;
    case 2: return casadi_s2;
    case 3: return casadi_s6;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Translational_drone_impl_dae_fun_jac_x_xdot_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 10;
  if (sz_iw) *sz_iw = 7;
  if (sz_w) *sz_w = 31;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif