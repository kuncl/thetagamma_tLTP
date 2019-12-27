/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__LTPLTD
#define _nrn_initial _nrn_initial__LTPLTD
#define nrn_cur _nrn_cur__LTPLTD
#define _nrn_current _nrn_current__LTPLTD
#define nrn_jacob _nrn_jacob__LTPLTD
#define nrn_state _nrn_state__LTPLTD
#define _net_receive _net_receive__LTPLTD 
#define calcBinAverage calcBinAverage__LTPLTD 
#define calcAverage calcAverage__LTPLTD 
#define state state__LTPLTD 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define carest _p[0]
#define pHC _p[1]
#define aHC _p[2]
#define theta_v _p[3]
#define theta_b _p[4]
#define theta_d _p[5]
#define cp _p[6]
#define cd _p[7]
#define low_p _p[8]
#define low_d _p[9]
#define alpha_w _p[10]
#define beta_w _p[11]
#define sigma_v _p[12]
#define sigma_d _p[13]
#define sigma_b _p[14]
#define Bin_interval _p[15]
#define pHN _p[16]
#define aHN _p[17]
#define kp _p[18]
#define kd _p[19]
#define tInit _p[20]
#define P_Fact _p[21]
#define D_Fact _p[22]
#define Avg_W _p[23]
#define MAX_W _p[24]
#define MIN_W _p[25]
#define SIGN_W _p[26]
#define Avg_Bin _p[27]
#define xspine _p[28]
#define A _p[29]
#define P _p[30]
#define V _p[31]
#define B _p[32]
#define D _p[33]
#define W _p[34]
#define cai _p[35]
#define DA _p[36]
#define DP _p[37]
#define DV _p[38]
#define DB _p[39]
#define DD _p[40]
#define DW _p[41]
#define tauw _p[42]
#define PREV_MAX_W _p[43]
#define PREV_MIN_W _p[44]
#define Top_Flag _p[45]
#define Bottom_Flag _p[46]
#define Bin_total _p[47]
#define Bin_count _p[48]
#define Bin_now_index _p[49]
#define a_deriv _p[50]
#define v_deriv _p[51]
#define p_deriv _p[52]
#define b_deriv _p[53]
#define d_deriv _p[54]
#define w_deriv _p[55]
#define v _p[56]
#define _g _p[57]
#define _nd_area  *_ppvar[0]._pval
#define _ion_cai	*_ppvar[2]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_a_sigma();
 static double _hoc_calcBinAverage();
 static double _hoc_calcAverage();
 static double _hoc_f_sigma();
 static double _hoc_myexp();
 static double _hoc_p_sigma();
 static double _hoc_trans();
 static double _hoc_unitconv();
 static double _hoc_v_sigma();
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "a_sigma", _hoc_a_sigma,
 "calcBinAverage", _hoc_calcBinAverage,
 "calcAverage", _hoc_calcAverage,
 "f_sigma", _hoc_f_sigma,
 "myexp", _hoc_myexp,
 "p_sigma", _hoc_p_sigma,
 "trans", _hoc_trans,
 "unitconv", _hoc_unitconv,
 "v_sigma", _hoc_v_sigma,
 0, 0
};
#define a_sigma a_sigma_LTPLTD
#define f_sigma f_sigma_LTPLTD
#define myexp myexp_LTPLTD
#define p_sigma p_sigma_LTPLTD
#define trans trans_LTPLTD
#define unitconv unitconv_LTPLTD
#define v_sigma v_sigma_LTPLTD
 extern double a_sigma( _threadargsprotocomma_ double );
 extern double f_sigma( _threadargsprotocomma_ double , double , double , double );
 extern double myexp( _threadargsprotocomma_ double );
 extern double p_sigma( _threadargsprotocomma_ double );
 extern double trans( _threadargsprotocomma_ double );
 extern double unitconv( _threadargsprotocomma_ double );
 extern double v_sigma( _threadargsprotocomma_ double );
 /* declare global and static user variables */
#define alpha_b alpha_b_LTPLTD
 double alpha_b = 5;
#define alpha_d alpha_d_LTPLTD
 double alpha_d = 1;
#define alpha_v alpha_v_LTPLTD
 double alpha_v = 1;
#define num_a num_a_LTPLTD
 double num_a = 1;
#define num_p num_p_LTPLTD
 double num_p = 10;
#define taub taub_LTPLTD
 double taub = 40;
#define taud taud_LTPLTD
 double taud = 250;
#define tauv tauv_LTPLTD
 double tauv = 10;
#define taua taua_LTPLTD
 double taua = 5;
#define taup taup_LTPLTD
 double taup = 500;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "taup_LTPLTD", "ms",
 "taua_LTPLTD", "ms",
 "tauv_LTPLTD", "ms",
 "taud_LTPLTD", "ms",
 "taub_LTPLTD", "ms",
 "carest", "mM",
 "pHC", "mM",
 "aHC", "mM",
 "theta_v", "mM",
 "tInit", "ms",
 "xspine", "uM",
 0,0
};
 static double A0 = 0;
 static double B0 = 0;
 static double D0 = 0;
 static double P0 = 0;
 static double V0 = 0;
 static double W0 = 0;
 static double delta_t = 1;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "num_p_LTPLTD", &num_p_LTPLTD,
 "num_a_LTPLTD", &num_a_LTPLTD,
 "taup_LTPLTD", &taup_LTPLTD,
 "taua_LTPLTD", &taua_LTPLTD,
 "tauv_LTPLTD", &tauv_LTPLTD,
 "taud_LTPLTD", &taud_LTPLTD,
 "taub_LTPLTD", &taub_LTPLTD,
 "alpha_v_LTPLTD", &alpha_v_LTPLTD,
 "alpha_d_LTPLTD", &alpha_d_LTPLTD,
 "alpha_b_LTPLTD", &alpha_b_LTPLTD,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[3]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"LTPLTD",
 "carest",
 "pHC",
 "aHC",
 "theta_v",
 "theta_b",
 "theta_d",
 "cp",
 "cd",
 "low_p",
 "low_d",
 "alpha_w",
 "beta_w",
 "sigma_v",
 "sigma_d",
 "sigma_b",
 "Bin_interval",
 "pHN",
 "aHN",
 "kp",
 "kd",
 "tInit",
 0,
 "P_Fact",
 "D_Fact",
 "Avg_W",
 "MAX_W",
 "MIN_W",
 "SIGN_W",
 "Avg_Bin",
 "xspine",
 0,
 "A",
 "P",
 "V",
 "B",
 "D",
 "W",
 0,
 0};
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 58, _prop);
 	/*initialize range parameters*/
 	carest = 0;
 	pHC = 0.004;
 	aHC = 0.0006;
 	theta_v = 0.002;
 	theta_b = 0.55;
 	theta_d = 2.6;
 	cp = 5;
 	cd = 4;
 	low_p = 0.3;
 	low_d = 0.01;
 	alpha_w = 0.8;
 	beta_w = 0.6;
 	sigma_v = -0.05;
 	sigma_d = -0.01;
 	sigma_b = -0.02;
 	Bin_interval = 100;
 	pHN = 4;
 	aHN = 3;
 	kp = -0.1;
 	kd = -0.002;
 	tInit = 0;
  }
 	_prop->param = _p;
 	_prop->param_size = 58;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 1, 0);
 	_ppvar[2]._pval = &prop_ion->param[1]; /* cai */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _plasticityModule_n_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("ca", -10000.);
 	_ca_sym = hoc_lookup("ca_ion");
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
  hoc_register_prop_size(_mechtype, 58, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 LTPLTD /home3/hjang/AD_OPTO_GAMMA/Evolution_Algorithm_Fit/neuron_code_tLTP_test_rev1/mod/x86_64/plasticityModule_n.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "plasticityModule";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int calcBinAverage(_threadargsproto_);
static int calcAverage(_threadargsproto_);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[6], _dlist1[6];
 static int state(_threadargsproto_);
 
static int  calcAverage ( _threadargsproto_ ) {
   if ( W >= 0.0 ) {
     SIGN_W = 1.0 ;
     }
   else {
     SIGN_W = - 1.0 ;
     }
   if ( W * SIGN_W >= MAX_W ) {
     MAX_W = W * SIGN_W ;
     Top_Flag = 0.0 ;
     }
   else {
     if ( Top_Flag  == 0.0 ) {
       PREV_MIN_W = MIN_W ;
       MIN_W = MAX_W ;
       }
     Top_Flag = 1.0 ;
     }
   if ( W * SIGN_W <= MIN_W ) {
     MIN_W = W * SIGN_W ;
     Bottom_Flag = 0.0 ;
     }
   else {
     if ( Bottom_Flag  == 0.0 ) {
       PREV_MAX_W = MAX_W ;
       MAX_W = MIN_W ;
       }
     Bottom_Flag = 1.0 ;
     }
   Avg_W = ( PREV_MAX_W + PREV_MIN_W ) / 2.0 * SIGN_W ;
   if ( Top_Flag  == 0.0  && Bottom_Flag  == 0.0 ) {
     }
    return 0; }
 
static double _hoc_calcAverage(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 calcAverage ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
static int  calcBinAverage ( _threadargsproto_ ) {
   double _lnow_bin , _lak ;
 Bin_total = Bin_total + W ;
   Bin_count = Bin_count + 1.0 ;
   _lnow_bin = floor ( ( t - tInit ) / Bin_interval ) ;
   if ( _lnow_bin > Bin_now_index ) {
     if ( Bin_now_index  != - 1.0 ) {
       Avg_Bin = Bin_total / Bin_count ;
       Bin_total = 0.0 ;
       Bin_count = 0.0 ;
       }
     Bin_now_index = _lnow_bin ;
     }
    return 0; }
 
static double _hoc_calcBinAverage(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r = 1.;
 calcBinAverage ( _p, _ppvar, _thread, _nt );
 return(_r);
}
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   xspine = trans ( _threadargscomma_ cai ) ;
   a_deriv = ( a_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - A ) / taua ;
   v_deriv = ( v_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - V ) / tauv ;
   p_deriv = ( p_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - cp * A * P ) / taup ;
   b_deriv = ( f_sigma ( _threadargscomma_ alpha_b , theta_b , sigma_b , A ) - B - cd * B * V ) / taub ;
   d_deriv = ( f_sigma ( _threadargscomma_ alpha_d , theta_d , sigma_d , B ) - D ) / taud ;
   w_deriv = ( alpha_w / ( 1.0 + myexp ( _threadargscomma_ ( P - low_p ) / kp ) ) - beta_w / ( 1.0 + myexp ( _threadargscomma_ ( D - low_d ) / kd ) ) - W ) / tauw ;
   if ( t < tInit ) {
     a_deriv = 0.0 ;
     v_deriv = 0.0 ;
     p_deriv = 0.0 ;
     b_deriv = 0.0 ;
     d_deriv = 0.0 ;
     w_deriv = 0.0 ;
     }
   else {
     }
   DA = a_deriv ;
   DV = v_deriv ;
   DP = p_deriv ;
   DB = b_deriv ;
   DD = d_deriv ;
   DW = w_deriv ;
   P_Fact = alpha_w / ( 1.0 + myexp ( _threadargscomma_ ( P - low_p ) / kp ) ) ;
   D_Fact = beta_w / ( 1.0 + myexp ( _threadargscomma_ ( D - low_d ) / kd ) ) ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 xspine = trans ( _threadargscomma_ cai ) ;
 a_deriv = ( a_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - A ) / taua ;
 v_deriv = ( v_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - V ) / tauv ;
 p_deriv = ( p_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - cp * A * P ) / taup ;
 b_deriv = ( f_sigma ( _threadargscomma_ alpha_b , theta_b , sigma_b , A ) - B - cd * B * V ) / taub ;
 d_deriv = ( f_sigma ( _threadargscomma_ alpha_d , theta_d , sigma_d , B ) - D ) / taud ;
 w_deriv = ( alpha_w / ( 1.0 + myexp ( _threadargscomma_ ( P - low_p ) / kp ) ) - beta_w / ( 1.0 + myexp ( _threadargscomma_ ( D - low_d ) / kd ) ) - W ) / tauw ;
 if ( t < tInit ) {
   a_deriv = 0.0 ;
   v_deriv = 0.0 ;
   p_deriv = 0.0 ;
   b_deriv = 0.0 ;
   d_deriv = 0.0 ;
   w_deriv = 0.0 ;
   }
 else {
   }
 DA = DA  / (1. - dt*( 0.0 )) ;
 DV = DV  / (1. - dt*( 0.0 )) ;
 DP = DP  / (1. - dt*( 0.0 )) ;
 DB = DB  / (1. - dt*( 0.0 )) ;
 DD = DD  / (1. - dt*( 0.0 )) ;
 DW = DW  / (1. - dt*( 0.0 )) ;
 return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   xspine = trans ( _threadargscomma_ cai ) ;
   a_deriv = ( a_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - A ) / taua ;
   v_deriv = ( v_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - V ) / tauv ;
   p_deriv = ( p_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - cp * A * P ) / taup ;
   b_deriv = ( f_sigma ( _threadargscomma_ alpha_b , theta_b , sigma_b , A ) - B - cd * B * V ) / taub ;
   d_deriv = ( f_sigma ( _threadargscomma_ alpha_d , theta_d , sigma_d , B ) - D ) / taud ;
   w_deriv = ( alpha_w / ( 1.0 + myexp ( _threadargscomma_ ( P - low_p ) / kp ) ) - beta_w / ( 1.0 + myexp ( _threadargscomma_ ( D - low_d ) / kd ) ) - W ) / tauw ;
   if ( t < tInit ) {
     a_deriv = 0.0 ;
     v_deriv = 0.0 ;
     p_deriv = 0.0 ;
     b_deriv = 0.0 ;
     d_deriv = 0.0 ;
     w_deriv = 0.0 ;
     }
   else {
     }
    A = A - dt*(- ( a_deriv ) ) ;
    V = V - dt*(- ( v_deriv ) ) ;
    P = P - dt*(- ( p_deriv ) ) ;
    B = B - dt*(- ( b_deriv ) ) ;
    D = D - dt*(- ( d_deriv ) ) ;
    W = W - dt*(- ( w_deriv ) ) ;
   P_Fact = alpha_w / ( 1.0 + myexp ( _threadargscomma_ ( P - low_p ) / kp ) ) ;
   D_Fact = beta_w / ( 1.0 + myexp ( _threadargscomma_ ( D - low_d ) / kd ) ) ;
   }
  return 0;
}
 
double myexp ( _threadargsprotocomma_ double _lx ) {
   double _lmyexp;
 if ( _lx > 700.0 ) {
     _lx = 700.0 ;
     }
   _lmyexp = exp ( _lx ) ;
   
return _lmyexp;
 }
 
static double _hoc_myexp(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  myexp ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double trans ( _threadargsprotocomma_ double _lx ) {
   double _ltrans;
 _ltrans = ( _lx ) ;
   
return _ltrans;
 }
 
static double _hoc_trans(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  trans ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double unitconv ( _threadargsprotocomma_ double _lx ) {
   double _lunitconv;
 _lunitconv = _lx * 1e3 ;
   
return _lunitconv;
 }
 
static double _hoc_unitconv(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  unitconv ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double p_sigma ( _threadargsprotocomma_ double _lx ) {
   double _lp_sigma;
 _lp_sigma = ( num_p * ( pow( ( _lx / unitconv ( _threadargscomma_ pHC ) ) , pHN ) ) ) / ( 1.0 + ( pow( ( _lx / unitconv ( _threadargscomma_ pHC ) ) , pHN ) ) ) ;
   
return _lp_sigma;
 }
 
static double _hoc_p_sigma(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  p_sigma ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double a_sigma ( _threadargsprotocomma_ double _lx ) {
   double _la_sigma;
 _la_sigma = ( num_a * ( pow( ( _lx / unitconv ( _threadargscomma_ aHC ) ) , aHN ) ) ) / ( 1.0 + ( pow( ( _lx / unitconv ( _threadargscomma_ aHC ) ) , aHN ) ) ) ;
   
return _la_sigma;
 }
 
static double _hoc_a_sigma(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  a_sigma ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double v_sigma ( _threadargsprotocomma_ double _lx ) {
   double _lv_sigma;
 _lv_sigma = alpha_v / ( 1.0 + myexp ( _threadargscomma_ ( _lx - unitconv ( _threadargscomma_ theta_v ) ) / sigma_v ) ) ;
   
return _lv_sigma;
 }
 
static double _hoc_v_sigma(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  v_sigma ( _p, _ppvar, _thread, _nt, *getarg(1) );
 return(_r);
}
 
double f_sigma ( _threadargsprotocomma_ double _lalpha_f , double _ltheta_f , double _lsigma_f , double _lx ) {
   double _lf_sigma;
 _lf_sigma = _lalpha_f / ( 1.0 + myexp ( _threadargscomma_ ( _lx - _ltheta_f ) / _lsigma_f ) ) ;
   
return _lf_sigma;
 }
 
static double _hoc_f_sigma(void* _vptr) {
 double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   _p = ((Point_process*)_vptr)->_prop->param;
  _ppvar = ((Point_process*)_vptr)->_prop->dparam;
  _thread = _extcall_thread;
  _nt = (_NrnThread*)((Point_process*)_vptr)->_vnt;
 _r =  f_sigma ( _p, _ppvar, _thread, _nt, *getarg(1) , *getarg(2) , *getarg(3) , *getarg(4) );
 return(_r);
}
 
static int _ode_count(int _type){ return 6;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 6; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  cai = _ion_cai;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_ca_sym, _ppvar, 2, 1);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  A = A0;
  B = B0;
  D = D0;
  P = P0;
  V = V0;
  W = W0;
 {
   tauw = 500.0 ;
   Avg_W = 0.0 ;
   MAX_W = 0.0 ;
   MIN_W = 0.0 ;
   PREV_MAX_W = 0.0 ;
   PREV_MIN_W = 0.0 ;
   SIGN_W = 1.0 ;
   Top_Flag = 0.0 ;
   Bottom_Flag = 0.0 ;
   Bin_total = 0.0 ;
   Bin_count = 0.0 ;
   Bin_now_index = - 1.0 ;
   Avg_Bin = 0.0 ;
   xspine = trans ( _threadargscomma_ cai ) ;
   p_deriv = 0.0 ;
   A = ( a_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - A ) / taua ;
   V = ( v_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - V ) / tauv ;
   P = ( p_sigma ( _threadargscomma_ unitconv ( _threadargscomma_ xspine ) ) - cp * A * P ) / taup ;
   B = ( f_sigma ( _threadargscomma_ alpha_b , theta_b , sigma_b , A ) - B - cd * B * V ) / taub ;
   D = ( f_sigma ( _threadargscomma_ alpha_d , theta_d , sigma_d , B ) - D ) / taud ;
   P_Fact = 0.0 ;
   D_Fact = 0.0 ;
   W = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  cai = _ion_cai;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{
} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  cai = _ion_cai;
 {   state(_p, _ppvar, _thread, _nt);
  } {
   if ( t > tInit ) {
     calcBinAverage ( _threadargs_ ) ;
     }
   Avg_W = Avg_Bin ;
   }
}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(V) - _p;  _dlist1[1] = &(DV) - _p;
 _slist1[2] = &(P) - _p;  _dlist1[2] = &(DP) - _p;
 _slist1[3] = &(B) - _p;  _dlist1[3] = &(DB) - _p;
 _slist1[4] = &(D) - _p;  _dlist1[4] = &(DD) - _p;
 _slist1[5] = &(W) - _p;  _dlist1[5] = &(DW) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
