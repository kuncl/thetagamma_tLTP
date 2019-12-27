/* Created by Language version: 6.2.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__MyAMPA
#define _nrn_initial _nrn_initial__MyAMPA
#define nrn_cur _nrn_cur__MyAMPA
#define _nrn_current _nrn_current__MyAMPA
#define nrn_jacob _nrn_jacob__MyAMPA
#define nrn_state _nrn_state__MyAMPA
#define _net_receive _net_receive__MyAMPA 
#define rates rates__MyAMPA 
#define state state__MyAMPA 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define tau _p[0]
#define e _p[1]
#define gbar _p[2]
#define STDPfactor _p[3]
#define temp _p[4]
#define i _p[5]
#define firstepsc _p[6]
#define lastepsc _p[7]
#define diffepsc _p[8]
#define A _p[9]
#define g _p[10]
#define tadj _p[11]
#define ntau _p[12]
#define STDPmod _p[13]
#define epscflag _p[14]
#define nowepsc _p[15]
#define epsccnt _p[16]
#define DA _p[17]
#define Dg _p[18]
#define _g _p[19]
#define _tsav _p[20]
#define _nd_area  *_ppvar[0]._pval
#define STDPweight	*_ppvar[2]._pval
#define _p_STDPweight	_ppvar[2]._pval
 
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
 static int hoc_nrnpointerindex =  2;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static double _hoc_rates();
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
 _p = _prop->param; _ppvar = _prop->dparam;
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
 "rates", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
#define q10 q10_MyAMPA
 double q10 = 2.7;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 "tau", 1e-09, 1e+09,
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "tau", "ms",
 "e", "mV",
 "gbar", "uS",
 "temp", "degC",
 "A", "uS",
 "g", "uS",
 "i", "nA",
 0,0
};
 static double A0 = 0;
 static double delta_t = 0.01;
 static double g0 = 0;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "q10_MyAMPA", &q10_MyAMPA,
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
"MyAMPA",
 "tau",
 "e",
 "gbar",
 "STDPfactor",
 "temp",
 0,
 "i",
 "firstepsc",
 "lastepsc",
 "diffepsc",
 0,
 "A",
 "g",
 0,
 "STDPweight",
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 21, _prop);
 	/*initialize range parameters*/
 	tau = 7.9;
 	e = 0;
 	gbar = 0.00025;
 	STDPfactor = 1e-05;
 	temp = 26.5;
  }
 	_prop->param = _p;
 	_prop->param_size = 21;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 4, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _net_receive(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _MyAMPA_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 21, 4);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
  hoc_register_dparam_semantics(_mechtype, 2, "pointer");
  hoc_register_dparam_semantics(_mechtype, 3, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_size[_mechtype] = 1;
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 MyAMPA /home2/hjang/Research/AD_OPTO_GAMMA/Evolution_Algorithm_Fit/neuron_code_tLTP/mod/x86_64/MyAMPA.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int rates(double);
 extern double *_getelm();
 
#define _MATELM1(_row,_col)	*(_getelm(_row + 1, _col + 1))
 
#define _RHS1(_arg) _coef1[_arg + 1]
 static double *_coef1;
 
#define _linmat1  1
 static void* _sparseobj1;
 static void* _cvsparseobj1;
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[2], _dlist1[2]; static double *_temp1;
 static int state();
 
static int  rates (  double _lv ) {
   tadj = pow( q10 , ( ( temp - celsius ) / 10.0 ) ) ;
   ntau = tau * tadj ;
    return 0; }
 
static double _hoc_rates(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 rates (  *getarg(1) );
 return(_r);
}
 
static int state ()
 {_reset=0;
 {
   double b_flux, f_flux, _term; int _i;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<2;_i++){
  	_RHS1(_i) = -_dt1*(_p[_slist1[_i]] - _p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 /* ~ A <-> g ( 1.0 / ntau , 0.0 )*/
 f_flux =  1.0 / ntau * A ;
 b_flux =  0.0 * g ;
 _RHS1( 0) -= (f_flux - b_flux);
 _RHS1( 1) += (f_flux - b_flux);
 
 _term =  1.0 / ntau ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  0.0 ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ g - > ( 1.0 / ntau )*/
 f_flux =  ( 1.0 / ntau ) * g ;
 b_flux = 0. ;
 _RHS1( 1) -= (f_flux - b_flux);
 
 _term =  ( 1.0 / ntau ) ;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
    } return _reset;
 }
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{    _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t; {
     if (nrn_netrec_state_adjust && !cvode_active_){
    /* discon state adjustment for general derivimplicit and KINETIC case */
    int __i, __neq = 2;
    double __state = A;
    double __primary_delta = (A + _args[0] * ntau * 1.0) - __state;
    double __dtsav = dt;
    for (__i = 0; __i < __neq; ++__i) {
      _p[_dlist1[__i]] = 0.0;
    }
    _p[_dlist1[0]] = __primary_delta;
    dt *= 0.5;
    v = NODEV(_pnt->node);
    _ode_matsol_instance1(_threadargs_);
    dt = __dtsav;
    for (__i = 0; __i < __neq; ++__i) {
      _p[_slist1[__i]] += _p[_dlist1[__i]];
    }
  } else {
 A = A + _args[0] * ntau * 1.0 ;
     }
 } }
 
/*CVODE ode begin*/
 static int _ode_spec1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
 {int _i; for(_i=0;_i<2;_i++) _p[_dlist1[_i]] = 0.0;}
 /* ~ A <-> g ( 1.0 / ntau , 0.0 )*/
 f_flux =  1.0 / ntau * A ;
 b_flux =  0.0 * g ;
 DA -= (f_flux - b_flux);
 Dg += (f_flux - b_flux);
 
 /*REACTION*/
  /* ~ g - > ( 1.0 / ntau )*/
 f_flux =  ( 1.0 / ntau ) * g ;
 b_flux = 0. ;
 Dg -= (f_flux - b_flux);
 
 /*REACTION*/
    } return _reset;
 }
 
/*CVODE matsol*/
 static int _ode_matsol1() {_reset=0;{
 double b_flux, f_flux, _term; int _i;
   b_flux = f_flux = 0.;
 {int _i; double _dt1 = 1.0/dt;
for(_i=0;_i<2;_i++){
  	_RHS1(_i) = _dt1*(_p[_dlist1[_i]]);
	_MATELM1(_i, _i) = _dt1;
      
} }
 /* ~ A <-> g ( 1.0 / ntau , 0.0 )*/
 _term =  1.0 / ntau ;
 _MATELM1( 0 ,0)  += _term;
 _MATELM1( 1 ,0)  -= _term;
 _term =  0.0 ;
 _MATELM1( 0 ,1)  -= _term;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
  /* ~ g - > ( 1.0 / ntau )*/
 _term =  ( 1.0 / ntau ) ;
 _MATELM1( 1 ,1)  += _term;
 /*REACTION*/
    } return _reset;
 }
 
/*CVODE end*/
 
static int _ode_count(int _type){ return 2;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 ();
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 2; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _cvode_sparse(&_cvsparseobj1, 2, _dlist1, _p, _ode_matsol1, &_coef1);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
  A = A0;
  g = g0;
 {
   g = 0.0 ;
   rates ( _threadargscomma_ v ) ;
   epscflag = 0.0 ;
   firstepsc = 0.0 ;
   lastepsc = 0.0 ;
   epsccnt = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
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
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   STDPmod = STDPfactor * STDPweight ;
   if ( STDPmod < - 1.0 ) {
     STDPmod = - 1.0 ;
     }
   i = gbar * g * ( v - e ) * ( 1.0 + STDPmod ) ;
   if ( g > 0.000000001  && epscflag  == 0.0 ) {
     epscflag = 1.0 ;
     epsccnt = epsccnt + 1.0 ;
     nowepsc = 0.0 ;
     }
   else if ( g > 0.000000001  && epscflag  == 1.0 ) {
     if ( i < nowepsc ) {
       nowepsc = i ;
       }
     }
   else if ( g < 0.00000001  && epscflag  == 1.0 ) {
     epscflag = 0.0 ;
     if ( epsccnt  == 1.0 ) {
       firstepsc = nowepsc ;
       }
     lastepsc = nowepsc ;
     diffepsc = lastepsc - firstepsc ;
     }
   else {
     }
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
double _dtsav = dt;
if (secondorder) { dt *= 0.5; }
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
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
 { error = sparse(&_sparseobj1, 2, _slist1, _dlist1, _p, &t, dt, state,&_coef1, _linmat1);
 if(error){fprintf(stderr,"at line 75 in file MyAMPA.mod:\n	SOLVE state METHOD sparse\n"); nrn_complain(_p); abort_run(error);}
    if (secondorder) {
    int _i;
    for (_i = 0; _i < 2; ++_i) {
      _p[_slist1[_i]] += dt*_p[_dlist1[_i]];
    }}
 }}}
 dt = _dtsav;
}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(A) - _p;  _dlist1[0] = &(DA) - _p;
 _slist1[1] = &(g) - _p;  _dlist1[1] = &(Dg) - _p;
_first = 0;
}
