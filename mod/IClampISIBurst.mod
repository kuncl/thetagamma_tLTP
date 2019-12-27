COMMENT

Since this is an electrode current, positive values of i depolarize the cell
and in the presence of the extracellular mechanism there will be a change
in vext since i is not a transmembrane current but a current injected
directly to the inside of the cell.

ENDCOMMENT

NEURON {
	POINT_PROCESS IClampISIBurst
	RANGE  del, amp, single_dur, burst_isi, burst_num, i, isi, num
	ELECTRODE_CURRENT i
}

UNITS {
	(nA) = (nanoamp)
	(mV) = (millivolt)
}

PARAMETER {
	del (ms)
	amp (nA)
	single_dur (ms)	<0,1e9>	
	burst_isi = 10	(ms) <1e-9,1e9>: time between spikes (msec)  100Hz
	burst_num = 1 : five stimulations
		
	isi	= 1000 (ms) <1e-9,1e9>: time between train (msec)
	num = 1 : maximal	
}

ASSIGNED { 
	i(nA)
	train_on
	train_flag
	cnt
	
	burst_on
	burst_flag
	burst_cnt 
	
	train_dur 
}

INITIAL { 
	i = 0 
	train_on = del  :first spike timing
	train_flag = 0 : off state
	cnt = 0
	
	burst_on = train_on : count for burst
	burst_flag = 0 : flag for burst
	burst_cnt = 0
	
	train_dur = single_dur * burst_num + burst_isi * (burst_num -1)
}

BREAKPOINT {
		
	at_time(train_on)
	at_time(train_on+isi)
	at_time(burst_on)
	at_time(burst_on+burst_isi)
	
	if (cnt < num) {
		if (t >= train_on && t < train_on + train_dur) {
			train_flag = 1
			
			if (burst_cnt < burst_num) {
				if (t >= burst_on && t < burst_on + single_dur) {
					burst_flag = 1
					i = amp
					
				} else if (burst_flag == 1 && t > burst_on + single_dur ) {
					burst_on = burst_on + burst_isi
					burst_cnt = burst_cnt + 1
					burst_flag = 0
					i = 0
					
				} else {
					i = 0
				}
			} else {
				i = 0
			}
		} else if (train_flag == 1 && t > train_on + train_dur) {
			train_on = train_on + isi
			burst_on = train_on
			
			i = 0
			train_flag = 0
			burst_cnt = 0
			burst_flag = 0
			
			cnt = cnt + 1
		} else {
			i = 0
			train_flag = 0
			burst_flag = 0
			burst_cnt = 0
		}
	} else {
		i = 0
	}
		
		
	
}
