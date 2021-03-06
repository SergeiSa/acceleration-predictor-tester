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
  #define CASADI_PREFIX(ID) g_Constraints_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int long long int
#endif

/* Add prefix to internal symbols */
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_f1 CASADI_PREFIX(f1)
#define casadi_f2 CASADI_PREFIX(f2)
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

static const casadi_int casadi_s0[7] = {3, 1, 0, 3, 0, 1, 2};
static const casadi_int casadi_s1[5] = {1, 1, 0, 1, 0};
static const casadi_int casadi_s2[9] = {1, 3, 0, 1, 2, 3, 0, 0, 0};

/* g_Constraint:(q[3])->(Task) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a2, a3, a4, a5, a6, a7;
  a0=5.0000000000000000e-01;
  a1=arg[0]? arg[0][0] : 0;
  a2=cos(a1);
  a3=(a0*a2);
  a4=arg[0]? arg[0][1] : 0;
  a5=cos(a4);
  a5=(a2*a5);
  a1=sin(a1);
  a6=sin(a4);
  a6=(a1*a6);
  a5=(a5-a6);
  a6=(a0*a5);
  a3=(a3+a6);
  a6=arg[0]? arg[0][2] : 0;
  a7=cos(a6);
  a5=(a5*a7);
  a7=cos(a4);
  a1=(a1*a7);
  a4=sin(a4);
  a2=(a2*a4);
  a1=(a1+a2);
  a6=sin(a6);
  a1=(a1*a6);
  a5=(a5-a1);
  a0=(a0*a5);
  a3=(a3+a0);
  if (res[0]!=0) res[0][0]=a3;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_name_out(casadi_int i){
  switch (i) {
    case 0: return "Task";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s1;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian:(q[3])->(TaskJacobian[1x3]) */
static int casadi_f1(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a2, a3, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][1] : 0;
  a3=cos(a2);
  a4=-5.0000000000000000e-01;
  a5=arg[0]? arg[0][2] : 0;
  a6=sin(a5);
  a6=(a4*a6);
  a7=(a3*a6);
  a8=sin(a2);
  a9=5.0000000000000000e-01;
  a10=cos(a5);
  a10=(a9*a10);
  a10=(a10+a9);
  a11=(a8*a10);
  a7=(a7-a11);
  a1=(a1*a7);
  a7=sin(a0);
  a11=sin(a2);
  a12=(a11*a6);
  a13=cos(a2);
  a14=(a13*a10);
  a12=(a12+a14);
  a12=(a12+a9);
  a7=(a7*a12);
  a1=(a1-a7);
  if (res[0]!=0) res[0][0]=a1;
  a1=cos(a2);
  a7=cos(a0);
  a12=(a7*a6);
  a1=(a1*a12);
  a12=sin(a2);
  a0=sin(a0);
  a6=(a0*a6);
  a12=(a12*a6);
  a1=(a1-a12);
  a12=cos(a2);
  a6=(a0*a10);
  a12=(a12*a6);
  a1=(a1-a12);
  a2=sin(a2);
  a10=(a7*a10);
  a2=(a2*a10);
  a1=(a1-a2);
  if (res[0]!=0) res[0][1]=a1;
  a1=cos(a5);
  a3=(a0*a3);
  a11=(a7*a11);
  a3=(a3+a11);
  a4=(a4*a3);
  a1=(a1*a4);
  a5=sin(a5);
  a7=(a7*a13);
  a0=(a0*a8);
  a7=(a7-a0);
  a9=(a9*a7);
  a5=(a5*a9);
  a1=(a1-a5);
  if (res[0]!=0) res[0][2]=a1;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f1(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_in(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 1;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}

/* g_Constraint_Jacobian_derivative:(q[3],v[3])->(TaskJacobian_derivative[1x3]) */
static int casadi_f2(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real a0, a1, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a2, a20, a21, a22, a23, a24, a25, a26, a27, a28, a29, a3, a30, a4, a5, a6, a7, a8, a9;
  a0=arg[0]? arg[0][0] : 0;
  a1=cos(a0);
  a2=arg[0]? arg[0][1] : 0;
  a3=cos(a2);
  a4=-5.0000000000000000e-01;
  a5=arg[0]? arg[0][2] : 0;
  a6=cos(a5);
  a6=(a4*a6);
  a7=(a3*a6);
  a8=sin(a2);
  a9=5.0000000000000000e-01;
  a10=sin(a5);
  a10=(a9*a10);
  a11=(a8*a10);
  a7=(a7+a11);
  a7=(a1*a7);
  a11=sin(a0);
  a12=sin(a2);
  a13=(a12*a6);
  a14=cos(a2);
  a15=(a14*a10);
  a13=(a13-a15);
  a13=(a11*a13);
  a7=(a7-a13);
  a13=arg[1]? arg[1][2] : 0;
  a7=(a7*a13);
  a15=sin(a5);
  a15=(a4*a15);
  a16=(a3*a15);
  a17=cos(a5);
  a17=(a9*a17);
  a17=(a17+a9);
  a18=(a8*a17);
  a16=(a16-a18);
  a18=sin(a0);
  a16=(a16*a18);
  a18=(a12*a15);
  a19=(a14*a17);
  a18=(a18+a19);
  a18=(a18+a9);
  a19=cos(a0);
  a18=(a18*a19);
  a16=(a16+a18);
  a18=arg[1]? arg[1][0] : 0;
  a16=(a16*a18);
  a19=sin(a2);
  a20=(a15*a19);
  a21=cos(a2);
  a22=(a17*a21);
  a20=(a20+a22);
  a1=(a1*a20);
  a20=cos(a2);
  a22=(a15*a20);
  a23=sin(a2);
  a24=(a17*a23);
  a22=(a22-a24);
  a11=(a11*a22);
  a1=(a1+a11);
  a11=arg[1]? arg[1][1] : 0;
  a1=(a1*a11);
  a16=(a16+a1);
  a7=(a7-a16);
  if (res[0]!=0) res[0][0]=a7;
  a7=sin(a2);
  a16=sin(a0);
  a1=(a17*a16);
  a1=(a7*a1);
  a22=cos(a2);
  a24=(a15*a16);
  a24=(a22*a24);
  a25=sin(a2);
  a26=cos(a0);
  a27=(a15*a26);
  a27=(a25*a27);
  a24=(a24+a27);
  a27=cos(a2);
  a28=(a17*a26);
  a28=(a27*a28);
  a24=(a24+a28);
  a1=(a1-a24);
  a1=(a1*a18);
  a24=sin(a0);
  a28=(a24*a17);
  a29=sin(a2);
  a28=(a28*a29);
  a0=cos(a0);
  a29=(a0*a15);
  a30=sin(a2);
  a29=(a29*a30);
  a15=(a24*a15);
  a30=cos(a2);
  a15=(a15*a30);
  a29=(a29+a15);
  a28=(a28-a29);
  a17=(a0*a17);
  a2=cos(a2);
  a17=(a17*a2);
  a28=(a28-a17);
  a28=(a28*a11);
  a1=(a1+a28);
  a28=(a0*a6);
  a22=(a22*a28);
  a6=(a24*a6);
  a25=(a25*a6);
  a22=(a22-a25);
  a25=(a24*a10);
  a27=(a27*a25);
  a22=(a22+a27);
  a10=(a0*a10);
  a7=(a7*a10);
  a22=(a22+a7);
  a22=(a22*a13);
  a1=(a1+a22);
  if (res[0]!=0) res[0][1]=a1;
  a1=cos(a5);
  a22=(a3*a26);
  a7=(a12*a16);
  a22=(a22-a7);
  a22=(a4*a22);
  a22=(a1*a22);
  a7=sin(a5);
  a16=(a14*a16);
  a26=(a8*a26);
  a16=(a16+a26);
  a16=(a9*a16);
  a16=(a7*a16);
  a22=(a22+a16);
  a22=(a22*a18);
  a20=(a0*a20);
  a19=(a24*a19);
  a20=(a20-a19);
  a20=(a4*a20);
  a1=(a1*a20);
  a23=(a0*a23);
  a21=(a24*a21);
  a23=(a23+a21);
  a23=(a9*a23);
  a7=(a7*a23);
  a1=(a1+a7);
  a1=(a1*a11);
  a22=(a22+a1);
  a3=(a24*a3);
  a12=(a0*a12);
  a3=(a3+a12);
  a4=(a4*a3);
  a3=sin(a5);
  a4=(a4*a3);
  a0=(a0*a14);
  a24=(a24*a8);
  a0=(a0-a24);
  a9=(a9*a0);
  a5=cos(a5);
  a9=(a9*a5);
  a4=(a4+a9);
  a4=(a4*a13);
  a22=(a22-a4);
  if (res[0]!=0) res[0][2]=a22;
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f2(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_release(int mem) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_incref(void) {
}

CASADI_SYMBOL_EXPORT void g_Constraint_Jacobian_derivative_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_in(void) { return 2;}

CASADI_SYMBOL_EXPORT casadi_int g_Constraint_Jacobian_derivative_n_out(void) { return 1;}

CASADI_SYMBOL_EXPORT casadi_real g_Constraint_Jacobian_derivative_default_in(casadi_int i){
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_in(casadi_int i){
  switch (i) {
    case 0: return "q";
    case 1: return "v";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* g_Constraint_Jacobian_derivative_name_out(casadi_int i){
  switch (i) {
    case 0: return "TaskJacobian_derivative";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s0;
    case 1: return casadi_s0;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* g_Constraint_Jacobian_derivative_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int g_Constraint_Jacobian_derivative_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 2;
  if (sz_res) *sz_res = 1;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 0;
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif
