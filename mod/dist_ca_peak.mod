TITLE ...just to store peak calcium 
: H. Jang, 2011

UNITS {
	(mA) = (milliamp)
	(mV) = (millivolt)
	(molar) = (1/liter)
	(mM) =	(millimolar)
}

PARAMETER {
	cai (mM)
	catimerest = 0
}


NEURON {
	SUFFIX dsca
	USEION ca READ cai
    RANGE camax, carest, catimerest
}

ASSIGNED {
	camax
	carest
	flag
}

INITIAL {
	camax = cai
	flag = 0 
}


BREAKPOINT {
	if (cai>camax) {camax=cai}
	if (t > catimerest && flag==0) {
		flag= 1
		carest = cai	
	}
}
