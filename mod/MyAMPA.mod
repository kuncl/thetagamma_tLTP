COMMENT
	The AMPA conductance can be derived from  Koch
	Single exponential alpha
	
	at 35 celcius
		
	
ENDCOMMENT


NEURON {
	POINT_PROCESS MyAMPA
	RANGE tau,  e, i, g, gbar, STDPfactor, temp, lastepsc, firstepsc, diffepsc
	POINTER STDPweight
	NONSPECIFIC_CURRENT i	
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(uS) = (microsiemens)
}

PARAMETER {
	tau = 7.9 (ms) <1e-9,1e9>
	
	e=0	(mV)
	gbar = 0.00025 (uS) : maximal conductance

	STDPfactor = 0.00001
	STDPweight = 0

	celsius		(degC)
	temp = 26.5	(degC)		: original temp 		
	q10  = 2.7			: temperature sensitivity (Hestrin, Sah, Nicoll (1990b)
	
}


ASSIGNED {
	v (mV)			: postsynaptic voltage
	i (nA)			: nonspecific current = g*(v - Erev)
	tadj
	ntau (ms)
	STDPmod 
	
	firstepsc
	lastepsc
	diffepsc
	epscflag
	nowepsc
	epsccnt 
}


STATE { A (uS) g (uS) }

INITIAL {
	g = 0 	
	rates(v)
	
	epscflag = 0
	firstepsc = 0
	lastepsc = 0
	epsccnt = 0
}

PROCEDURE rates(v) {     :Computes rate and other constants at current v.
                      :Call once from HOC to initialize inf at resting v.
	
	tadj = q10^((temp - celsius)/10)  :temperature adjastment 
	ntau = tau * tadj
}
BREAKPOINT { 
	SOLVE state METHOD sparse
	STDPmod = STDPfactor * STDPweight
	if (STDPmod < -1) {
		STDPmod = -1
	}
	i = gbar * g * (v - e) * ( 1+ STDPmod)
	:printf("%f\n",STDPfactor * STDPweight)
	: printf("t = %f, epsc = %f", t, i)
	
	if (g > 0.000000001 && epscflag == 0 ) {
		
		epscflag = 1
		epsccnt = epsccnt + 1.0
		nowepsc = 0		
		: printf("# %f ; EPSC on \n ", epsccnt)
		
	} else if (g > 0.000000001 && epscflag == 1 ) {
		if (i < nowepsc) {
			nowepsc = i
		}
		: now on
		
	} else if ( g < 0.00000001 && epscflag == 1) {
		epscflag = 0
		if (epsccnt == 1.0) {
			firstepsc = nowepsc
		}
		lastepsc = nowepsc
		diffepsc = lastepsc - firstepsc
		: printf("# %f ; %f ms; EPSC off ; %f nA\n",epsccnt, t, nowepsc)
		
	} else {
	
	}
	
	
}

KINETIC state {
	~ A <-> g (1/ntau, 0)
	~ g -> (1/ntau)
}
	
NET_RECEIVE(weight (uS)) {
	A = A + weight*ntau*1  (/ms)

}
