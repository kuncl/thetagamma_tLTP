COMMENT
	The NMDA conductance can be derived from 
	a model in which the binding rate constant of Mg2+ varies
	as an exponential function of voltage.
	
	
	
	Ascher and Nowak, 1998
	Jahr and Stevens,	1990
	at 35 celcius
		
	gNMDAca / gNMDAna = 0.11 (Mayer and Westbrook, 1987)
ENDCOMMENT


NEURON {
	POINT_PROCESS MyNMDA
	RANGE tau1, tau2, e, i, g, gbar, mg, caratio,camethod, ica, temp, rsyn, rca, r, gbarsyn, gbarca, revsyn, revca
	NONSPECIFIC_CURRENT i
	USEION ca WRITE ica VALENCE 2
	
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
	(uS) = (microsiemens)
	(mM) = (milli/liter)
}

PARAMETER {
	tau1= 80 (ms) <1e-9,1e9>
	tau2 = 0.67 (ms) <1e-9,1e9>
	temp = 35	(degC)		: original temp 		
	: tau1= 141.5 (ms) <1e-9,1e9>
	: tau2 = 4.1 (ms) <1e-9,1e9>
	: temp = 23	(degC)		: original temp 			
	
	e=0	(mV)
	gbar = 0.0002 (uS) : maximal conductance
	
	
	gbarsyn = 0.0003 (uS) : maximal conductance
	gbarca = 0.025 (uS) : maximal conductance
	mg = 1 (mM)
	caratio = 0.11 : Mayer and Westbrook
	
	camethod = 1 : 1 - 0.11 / 2 - Shouval / 3 - Rubin
	
	
	
	n = 3.57
	r = 0.062 : EXP

	
	celsius		(degC)
	q10d1 = 3.5
	q10r2  = 2.2		: temperature sensitivity (Hestrin, Sah, Nicoll (1990b)
	: Feldmeyer et al., 2002; Leszkiewicz, 2004; Erreger et al, 2004; Popescu and Auerbach, 2004)
	
	revca = 140
	revsyn = 0
	
	rsyn = 0.062
	rca = 0.124
}


ASSIGNED {
	v (mV)			: postsynaptic voltage
	i (nA)			: nonspecific current = g*(v - Erev)
	ica (nA)		: calcium current through NMDA synapse (Carter/Sabatini)	
	isyn (nA)
	
	g (uS)
	
	
	tadj1
	tadj2
	ntau1 (ms)	
	ntau2 (ms)	
	
}
STATE {
	A (uS)
	B (uS)
}
INITIAL {

	A= 0 
	B= 0
	
	myrates(v)

}

PROCEDURE myrates(v) {     :Computes rate and other constants at current v.
                      :Call once from HOC to initialize inf at resting v.

	tadj1 = q10d1^((temp - celsius)/10)  :temperature adjastment 
	tadj2 = q10r2^((temp - celsius)/10)  :temperature adjastment 
	
	ntau1 = tau1 * tadj1
	ntau2 = tau2 * tadj2
	
	: printf("Adjusted Tau1 = %f ms, Tau2 = %f\n", ntau1, ntau2)
	
}

BREAKPOINT { 
	SOLVE state METHOD cnexp

	g = A - B
	
	if (camethod == 1) {
		:Shouval 
		: printf("Method1")
		n = 3.57
		r = 0.062
		i = ( 1- caratio ) * gbar * g * (v - e)/ (1+(mg/n)*exp(-r*v ) )	
		ica = caratio * gbar * g * (v - e)/ (1+(mg/n)*exp(-r*v) )
		
	} else if (camethod == 2) {
		: printf("Method2")
		isyn = gbarsyn * g * (v - revsyn) / (1.0 + (mg/3.57)*exp(-rsyn*v))
		ica = gbarca * g * (v - revca)  / (1.0 + (mg/3.57)*exp(-rca*v))
		
		i = isyn + ica
	} else {
		: printf("Method3")
		n = 0.3
		r = 0.062
		i = gbarsyn * g * (v - e)/ (1+n*mg*exp(-r*v) )	
		ica = gbarca * g * (v - e)/ (1+n*mg*exp(-r*2*v) ) 	
	}
	
	
}


DERIVATIVE state {
	A' = -A / ntau1
	B' = -B / ntau2
}

NET_RECEIVE(weight (uS)) {
	state_discontinuity(A, A + weight)
	state_discontinuity(B, B + weight)
}
