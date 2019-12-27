TITLE ...just to store peak membrane voltage
: M.Migliore June 2001

UNITS {
	(mA) = (milliamp)
	(mV) = (millivolt)

}


NEURON {
	SUFFIX anPSP
    RANGE vpeak, vrest
}



PARAMETER {
	v (mV)
	vrest = -60 (mV)	
	vdir = 1 : EPSP 
}

ASSIGNED {
	
	tstart (ms)
	tpeak (ms)
	tend (ms)
	
	vpeak (mV)
}


INITIAL {
	vpeak = v
	
	tstart = 0
	tpeak = 0
	tend = 0
}


BREAKPOINT {
	: if ( tstart == 0 && v < vrest) {
		: tstart = t
	: }
	if (vdir == 1 ) {	
		if (v > vpeak) {
			vpeak = v
			tpeak = t
		}
	} else {
		if (v < vpeak) {
			vpeak = v
			tpeak = t
		}	
	}
	: if (tend == 0 && v > vrest) {
		: tend = t
	: }
}
