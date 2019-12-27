COMMENT
Since this is an electrode current, positive values of i depolarize the cell
and in the presence of the extracellular mechanism there will be a change
in vext since i is not a transmembrane current but a current injected
directly to the inside of the cell.
ENDCOMMENT

NEURON {
	POINT_PROCESS IClampISI
	RANGE  del, dur, amp, interval, i, num
	ELECTRODE_CURRENT i
}
UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
}

PARAMETER {
	del (ms)
	dur (ms)	<0,1e9>
	amp (nA)
	interval	(ms) <1e-9,1e9>: time between spikes (msec)
	num = 10000 : maximal
	
}

ASSIGNED { 
	i(nA)
	on
	flag
	cnt	
}

INITIAL { 
	i = 0 
	on = del  :first spike timing
	flag = 0 : off state
	cnt = 0
}

BREAKPOINT {
	at_time(del)
	at_time(del+dur)
	if (cnt < num) {
		if (t < on + dur && t >= on) {
			i = amp
			flag = 1 : on state
		} else if (t > on + dur && flag == 1) {		
			on = on + interval
			i = 0
			flag = 0 
			cnt = cnt + 1
		}
		else{
			i = 0
			flag = 0
		}
	}
	else {
		i = 0
		flag= 0
	}
}
