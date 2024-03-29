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
  #define CASADI_PREFIX(ID) Planar_drone_impl_dae_jac_x_xdot_u_z_ ## ID
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
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_s8 CASADI_PREFIX(s8)
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

static const casadi_int casadi_s0[14] = {6, 6, 0, 0, 0, 2, 3, 4, 5, 3, 4, 0, 1, 2};
static const casadi_int casadi_s1[14] = {6, 6, 0, 1, 2, 3, 4, 5, 5, 3, 4, 5, 2, 2};
static const casadi_int casadi_s2[15] = {6, 6, 0, 1, 2, 3, 4, 5, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[8] = {6, 2, 0, 2, 3, 3, 4, 5};
static const casadi_int casadi_s4[12] = {2, 6, 0, 0, 0, 0, 1, 2, 3, 0, 0, 1};
static const casadi_int casadi_s5[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s6[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s7[3] = {0, 0, 0};
static const casadi_int casadi_s8[3] = {6, 0, 0};

/* Planar_drone_impl_dae_jac_x_xdot_u_z:(i0[6],i1[6],i2[2],i3[],i4[])->(o0[6x6,5nz],o1[6x6,6nz],o2[6x2,3nz],o3[6x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_int i;
  casadi_real *rr, *ss;
  const casadi_real *cs;
  casadi_real *w0=w+0, *w1=w+5, w2, w3, w4, w5, w6, w7, w8, w9, *w11=w+19, *w12=w+24, *w13=w+29, *w14=w+35, *w17=w+38, *w18=w+40, *w19=w+43;
  /* #0: @0 = zeros(6x6,5nz) */
  casadi_clear(w0, 5);
  /* #1: @1 = ones(6x1) */
  casadi_fill(w1, 6, 1.);
  /* #2: {NULL, NULL, @2, @3, @4, @5} = vertsplit(@1) */
  w2 = w1[2];
  w3 = w1[3];
  w4 = w1[4];
  w5 = w1[5];
  /* #3: @6 = input[2][0] */
  w6 = arg[2] ? arg[2][0] : 0;
  /* #4: @7 = 0.01475 */
  w7 = 1.4749999999999999e-02;
  /* #5: @7 = (@6/@7) */
  w7  = (w6/w7);
  /* #6: @8 = input[0][2] */
  w8 = arg[0] ? arg[0][2] : 0;
  /* #7: @9 = cos(@8) */
  w9 = cos( w8 );
  /* #8: @9 = (@9*@2) */
  w9 *= w2;
  /* #9: @7 = (@7*@9) */
  w7 *= w9;
  /* #10: @7 = (-@7) */
  w7 = (- w7 );
  /* #11: @9 = 0.01475 */
  w9 = 1.4749999999999999e-02;
  /* #12: @6 = (@6/@9) */
  w6 /= w9;
  /* #13: @9 = sin(@8) */
  w9 = sin( w8 );
  /* #14: @9 = (@9*@2) */
  w9 *= w2;
  /* #15: @6 = (@6*@9) */
  w6 *= w9;
  /* #16: @6 = (-@6) */
  w6 = (- w6 );
  /* #17: @10 = 00 */
  /* #18: @11 = vertcat(@3, @4, @5, @7, @6, @10) */
  rr=w11;
  *rr++ = w3;
  *rr++ = w4;
  *rr++ = w5;
  *rr++ = w7;
  *rr++ = w6;
  /* #19: @11 = (-@11) */
  for (i=0, rr=w11, cs=w11; i<5; ++i) *rr++ = (- *cs++ );
  /* #20: @12 = @11[:5] */
  for (rr=w12, ss=w11+0; ss!=w11+5; ss+=1) *rr++ = *ss;
  /* #21: (@0[:5] = @12) */
  for (rr=w0+0, ss=w12; rr!=w0+5; rr+=1) *rr = *ss++;
  /* #22: @12 = @0' */
  casadi_trans(w0,casadi_s1, w12, casadi_s0, iw);
  /* #23: output[0][0] = @12 */
  casadi_copy(w12, 5, res[0]);
  /* #24: @1 = zeros(6x6,6nz) */
  casadi_clear(w1, 6);
  /* #25: @13 = ones(6x1) */
  casadi_fill(w13, 6, 1.);
  /* #26: (@1[:6] = @13) */
  for (rr=w1+0, ss=w13; rr!=w1+6; rr+=1) *rr = *ss++;
  /* #27: @13 = @1' */
  casadi_trans(w1,casadi_s2, w13, casadi_s2, iw);
  /* #28: output[1][0] = @13 */
  casadi_copy(w13, 6, res[1]);
  /* #29: @14 = zeros(2x6,3nz) */
  casadi_clear(w14, 3);
  /* #30: @10 = 00 */
  /* #31: @15 = 00 */
  /* #32: @16 = 00 */
  /* #33: @3 = sin(@8) */
  w3 = sin( w8 );
  /* #34: @4 = 67.7966 */
  w4 = 6.7796610169491530e+01;
  /* #35: @17 = ones(2x1) */
  casadi_fill(w17, 2, 1.);
  /* #36: {@5, @7} = vertsplit(@17) */
  w5 = w17[0];
  w7 = w17[1];
  /* #37: @4 = (@4*@5) */
  w4 *= w5;
  /* #38: @3 = (@3*@4) */
  w3 *= w4;
  /* #39: @3 = (-@3) */
  w3 = (- w3 );
  /* #40: @8 = cos(@8) */
  w8 = cos( w8 );
  /* #41: @4 = 67.7966 */
  w4 = 6.7796610169491530e+01;
  /* #42: @4 = (@4*@5) */
  w4 *= w5;
  /* #43: @8 = (@8*@4) */
  w8 *= w4;
  /* #44: @4 = 60343.8 */
  w4 = 6.0343802781969993e+04;
  /* #45: @4 = (@4*@7) */
  w4 *= w7;
  /* #46: @18 = vertcat(@10, @15, @16, @3, @8, @4) */
  rr=w18;
  *rr++ = w3;
  *rr++ = w8;
  *rr++ = w4;
  /* #47: @18 = (-@18) */
  for (i=0, rr=w18, cs=w18; i<3; ++i) *rr++ = (- *cs++ );
  /* #48: @19 = @18[:3] */
  for (rr=w19, ss=w18+0; ss!=w18+3; ss+=1) *rr++ = *ss;
  /* #49: (@14[:3] = @19) */
  for (rr=w14+0, ss=w19; rr!=w14+3; rr+=1) *rr = *ss++;
  /* #50: @19 = @14' */
  casadi_trans(w14,casadi_s4, w19, casadi_s3, iw);
  /* #51: output[2][0] = @19 */
  casadi_copy(w19, 3, res[2]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_jac_x_xdot_u_z(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_jac_x_xdot_u_z_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_jac_x_xdot_u_z_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_jac_x_xdot_u_z_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_jac_x_xdot_u_z_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_jac_x_xdot_u_z_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_jac_x_xdot_u_z_incref(void) {
}

CASADI_SYMBOL_EXPORT void Planar_drone_impl_dae_jac_x_xdot_u_z_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_impl_dae_jac_x_xdot_u_z_n_in(void) { return 5;}

CASADI_SYMBOL_EXPORT casadi_int Planar_drone_impl_dae_jac_x_xdot_u_z_n_out(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_real Planar_drone_impl_dae_jac_x_xdot_u_z_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_impl_dae_jac_x_xdot_u_z_name_in(casadi_int i){
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    case 4: return "i4";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Planar_drone_impl_dae_jac_x_xdot_u_z_name_out(casadi_int i){
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    case 3: return "o3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_impl_dae_jac_x_xdot_u_z_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s5;
    case 2: return casadi_s6;
    case 3: return casadi_s7;
    case 4: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Planar_drone_impl_dae_jac_x_xdot_u_z_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s2;
    case 2: return casadi_s3;
    case 3: return casadi_s8;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Planar_drone_impl_dae_jac_x_xdot_u_z_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 11;
  if (sz_res) *sz_res = 10;
  if (sz_iw) *sz_iw = 7;
  if (sz_w) *sz_w = 46;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
