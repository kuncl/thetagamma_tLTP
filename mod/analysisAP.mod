TITLE ...just to store peak membrane voltage
: M.Migliore June 2001

UNITS {
	(mA) = (milliamp)
	(mV) = (millivolt)

}


NEURON {
	SUFFIX anAP
    RANGE PeakV, PeakT, MinV, Vamp, Vthreshold, tRange1, tRange2
}


PARAMETER {
	v (mV)
	
	tRange1 = 0 (ms)
	tRange2 = 0 (ms)
}


ASSIGNED {
	prev (mV)
	pret (ms)
	
	PeakV
	PeakT
	Vamp
	MinV
	DV
	
	Vthreshold 
	
	APthreshold
}

INITIAL {
	prev = v
	pret = 0
	PeakV = v
	PeakT = 0
	MinV = v
	Vthreshold = 0
	
	APthreshold = 20
}


BREAKPOINT {
	if (t >= tRange1 && (t <= tRange2 || tRange2 <= tRange1) ) {		
		if (v>PeakV) {
			PeakV=v
			PeakT=t
			
			DV = (v-prev) / (t-pret)
			if (Vthreshold == 0 && DV > 40) {
				Vthreshold = v
				:printf("t = %f ms, PreV = %fmV , AP threshold = %fmV \n" ,t,  prev , Vthreshold)
			}
			
			Vamp = PeakV - Vthreshold		
			
		}
		
		if (v<MinV) {
			MinV=v
		}
		
		if (v > APthreshold && prev == PeakV) {
			: printf("AP Peak! , t = %f, V = %f, MinV = %f, \n", pret, prev, MinV)
			:MinV = v
		}
		
		prev = v
		pret = t
	}
}
