TITLE plasticityModule
:


INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}

NEURON {
	POINT_PROCESS LTPLTD
	USEION ca READ cai
	RANGE carest, W, pHC, aHC, theta_v, theta_b, theta_d,  Avg_W, MAX_W, MIN_W, SIGN_W, Avg_Bin, Bin_interval, xspine    ,low_p,low_d,kp,kd, alpha_w, beta_w, cp, cd, P_Fact, D_Fact, sigma_v, sigma_d, sigma_b, pHN, aHN, tInit
}

UNITS {
	(molar) = (1/liter)			: moles do not appear in units
	(mM)	= (millimolar)
	(mA)	= (milliamp)
}


PARAMETER {

	cai		(mM)		: Ca concentration inside
	carest  (mM) 		: Baseline Calcium concentration
	
	pHC = 0.004  (mM)
	aHC = 0.0006  (mM)
	theta_v = 0.002  (mM)
	
	theta_b = 0.55
	theta_d = 2.6
	
	cp = 5
	cd = 4 
	
	low_p = 0.3 
	low_d = 0.01
	
	alpha_w = 0.8 
	beta_w = 0.6 		
	
	num_p = 10
	num_a = 1
	
	
	taup = 500 (ms)
	taua = 5 (ms)
	tauv = 10 (ms)
	
	taud = 250 (ms)
	taub = 40 (ms)
	
	
	alpha_v = 1.0
	alpha_d = 1.0
	alpha_b = 5.0
	
	sigma_v = -0.05
	sigma_d = -0.01
	sigma_b = -0.02
	
	:Bin_interval = 1000
	Bin_interval = 100

	pHN = 4
	aHN = 3
	
	kp = -0.1
	kd = -0.002
	
	tInit = 0 (ms)
	
}

STATE {
	A P V B D W
}

INITIAL {					

	
	tauw = 500 (ms)


	Avg_W = 0
	
	MAX_W = 0
	MIN_W = 0
	
	PREV_MAX_W = 0
	PREV_MIN_W = 0 
	
	SIGN_W = 1
	
	Top_Flag = 0
	Bottom_Flag = 0
	
	Bin_total = 0
	Bin_count = 0
	Bin_now_index = -1
	Avg_Bin = 0


	xspine = trans(cai)
	p_deriv = 0
	A = (a_sigma( unitconv(xspine) ) - A ) / taua
	V = (v_sigma( unitconv(xspine) ) - V ) / tauv	
	P = (p_sigma( unitconv(xspine) ) - cp * A * P ) / taup
	B = (f_sigma(alpha_b,theta_b,sigma_b, A) - B - cd * B * V ) / taub
	D = (f_sigma(alpha_d,theta_d,sigma_d, B) - D ) / taud
	
	
	:if (t <0.1) { printf("Initial | P = %f \t P' = %f \n", P,  cp * A * P) }
	P_Fact = 0
	D_Fact = 0 
	W = 0:(alpha_w / ( 1 + myexp( ( P - low_p) / kp ) ) - beta_w / ( 1 + myexp ( ( D - low_d ) / kd) ) - W ) / tauw	 
}

ASSIGNED {

	P_Fact
	D_Fact
	
	
	tauw  (ms)
	
	
		
	Avg_W
	
	MAX_W
	MIN_W
	
	PREV_MAX_W
	PREV_MIN_W
	
	SIGN_W
	
	Top_Flag
	Bottom_Flag
	
	Bin_total
	Bin_count
	Bin_now_index
	Avg_Bin
	
	xspine (uM)
	
	a_deriv
	v_deriv
	p_deriv
	
	b_deriv
	d_deriv
	w_deriv
	
}


PROCEDURE calcAverage() {

	if (W >= 0) {
		SIGN_W = 1
	} else {
		SIGN_W = -1		
	}
	
		if (W * SIGN_W >= MAX_W) {
			MAX_W = W * SIGN_W
			
			Top_Flag = 0
		}
		else {
			if (Top_Flag == 0) { 
				PREV_MIN_W = MIN_W	
				
				MIN_W = MAX_W
				
			}
			Top_Flag = 1
		}
		if (W * SIGN_W <= MIN_W) { 
			MIN_W = W * SIGN_W
			
			Bottom_Flag = 0
		}else {
			if (Bottom_Flag == 0 ){
				PREV_MAX_W = MAX_W
				MAX_W = MIN_W				
			}
			Bottom_Flag = 1
		}
		Avg_W = (PREV_MAX_W + PREV_MIN_W)/2		 * SIGN_W
		if (Top_Flag ==0 && Bottom_Flag == 0){
			
			
		}
}


PROCEDURE calcBinAverage() { LOCAL now_bin, ak
	Bin_total = Bin_total + W
	Bin_count = Bin_count + 1
		
	now_bin = floor((t - tInit)/ Bin_interval)	
	
	if (now_bin > Bin_now_index)  {
		if (Bin_now_index != -1) {
		
			Avg_Bin = Bin_total / Bin_count
			
			Bin_total = 0
			Bin_count = 0
			
			
		}
		Bin_now_index = now_bin
	}
}


BREAKPOINT {

	SOLVE state METHOD cnexp	
	
	if (t > tInit )  {
		calcBinAverage()
	}
	Avg_W = Avg_Bin		
	:if (t <= 0.1) {printf("BREAKPOINT | t=%f \t P = %f \t P' = %f \n",t, P, p_deriv)}	
}


DERIVATIVE state { 
	xspine = trans(cai)
	a_deriv = (a_sigma( unitconv(xspine)  ) - A ) / taua
	v_deriv =  (v_sigma( unitconv(xspine)  ) - V ) / tauv	
	p_deriv = (p_sigma( unitconv(xspine) ) - cp * A * P ) / taup	
	b_deriv = (f_sigma(alpha_b,theta_b,sigma_b, A) - B - cd * B * V) / taub
	d_deriv =  (f_sigma(alpha_d,theta_d,sigma_d, B) - D ) / taud
	w_deriv =  (alpha_w / ( 1 + myexp( ( P - low_p) / kp ) ) - beta_w / ( 1 + myexp ( ( D - low_d ) / kd) ) - W ) / tauw
	
	if (t < tInit ) {
		: printf("t = %f, tInit = %f\n",t,tInit)
		a_deriv = 0
		v_deriv = 0
		p_deriv = 0
		b_deriv = 0
		d_deriv = 0
		w_deriv = 0
		:low_p = P
		:low_d = D		
	} else{ 
		
	}
	
	A' = a_deriv
	V' = v_deriv	
	P' = p_deriv
	B' = b_deriv
	D' = d_deriv
	W' = w_deriv
	
	P_Fact = alpha_w / ( 1 + myexp( ( P - low_p) / kp ) )
	D_Fact = beta_w / ( 1 + myexp ( ( D - low_d ) / kd) )
		:printf("%f \t %f \n",alpha_w / ( 1 + myexp( ( P - low_p) / kp ) ) ,beta_w / ( 1 + myexp ( ( D - low_d ) / kd) ) )
}

FUNCTION myexp(x) { 
	if ( x > 700) {
		x = 700
	}
	myexp = exp( x ) 
	:printf("Calcium influx%f \n", trans)
}

FUNCTION trans(x) { 
	trans = ( x ) 
	:printf("Calcium influx%f \n", trans)
}

FUNCTION unitconv(x) { : mM -> uM
	unitconv = x * 1e3
}

FUNCTION p_sigma(x) {
	p_sigma = ( num_p * ( ( x / unitconv(pHC) ) ^ pHN) ) / ( 1 + ( (x / unitconv(pHC) ) ^ pHN) )	
		:printf("X : %f \t p_Sigma : %f \n",x, p_sigma)	
}	

FUNCTION a_sigma(x) {
	a_sigma = ( num_a * ( ( x / unitconv(aHC) ) ^ aHN) ) / ( 1 + ((x / unitconv(aHC) ) ^ aHN) )
		:printf("X : %f \t a_Sigma : %f \n",x, a_sigma)
}

FUNCTION v_sigma(x) {
	v_sigma = alpha_v / ( 1.0 + myexp ( ( x - unitconv(theta_v) ) / sigma_v ) )
	:printf("X : %f \t V_Sigma : %f \n",x, v_sigma)
}

FUNCTION f_sigma(alpha_f, theta_f, sigma_f, x) {
	f_sigma = alpha_f / ( 1.0 + myexp ( ( x - theta_f) / sigma_f ) ) 	
		:printf("X : %f \t f_Sigma : %f \n",x, f_sigma)	
}
