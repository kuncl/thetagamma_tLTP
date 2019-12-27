NEURON {
	POINT_PROCESS EPSCCount
	RANGE n, thresh, time, firing, lastepsc, firstepsc, diffepsc
	THREADSAFE : if APCount.record uses distinct instances of Vector
	POINTER curr

}

UNITS {
	(nA) = (nanoamp)
}

PARAMETER {
	n
	thresh = -20 (nA)
	time (ms)
	lastepsc (nA)
	firstepsc (nA)
	diffepsc (nA)
}

ASSIGNED {
	firing
	space
	curr (nA)
	mycurr (nA)
	maxcurr 
	maxtime (ms)
}

VERBATIM
extern void vector_resize();
extern double* vector_vec();
extern void* vector_arg();
ENDVERBATIM

INITIAL {
	n = 0
	firing = 0
	mycurr = 0
	maxcurr = thresh
	maxtime = 0
VERBATIM
	{ void* vv;
		vv = *((void**)(&space));
		if (vv) {
			vector_resize(vv, 0);
		}
	}
ENDVERBATIM
	check()
}

BREAKPOINT {
	SOLVE check METHOD after_cvode
}

PROCEDURE check() {
	mycurr = curr * -1
	
VERBATIM
	if (mycurr >= thresh && !firing) {
		
		int size; double* px; void* vv;
		firing = 1;
		time = t;
		
		n += 1.;
		
		
		vv = *((void**)(&space));
		if (vv) {
			size = (int)n;
			vector_resize(vv, size);
			px = vector_vec(vv);
			px[size-1] = time;
		}
		
		maxcurr = mycurr;
		maxtime = time;
	}
	if (firing && mycurr >= maxcurr) {
		maxcurr = mycurr;
		maxtime = t;
		//printf("%f ## %f ## Peak : %f  ## Diff : %f \n", time, n, maxcurr, mycurr );		
	}
	
	
	if (firing && mycurr < thresh && t > time) {
		
		lastepsc =  maxcurr;
		if (n==1.0) {
			firstepsc = maxcurr;			
		}
		diffepsc = lastepsc - firstepsc;
		

		firing = 0;
		
		//printf("%f ## %f ## Peak : %f  ## Diff : %f \n", maxtime, n, maxcurr, diffepsc );	
		
	}
ENDVERBATIM
}

PROCEDURE record() {
VERBATIM
	extern void* vector_arg();
	void** vv;
	vv = (void**)(&space);
	*vv = (void*)0;
	if (ifarg(1)) {
		*vv = vector_arg(1);
	}
ENDVERBATIM
}
