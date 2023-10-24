# program to obtain cyclic-voltammetry [ I(V) ] data from tvi from bc
# usage: awk -f tvi2cv.awk file.tvi >file.cv

BEGIN{ vstep=0.005; Q=0.00; pts=0; lastNR=0;}

{
	time = $1;
	volts = $2;
	current = $3;
	if(NR==1){lastvolts=volts;lasttime=time;}

	Q += current * (time-lasttime);
	dV = volts-lastvolts; if(dV<0){dV=lastvolts-volts;}

	if(dV>vstep){	# moved the required voltage step
		Vinterim[pts]=volts;
		Qinterim[pts]=Q;
		tinterim[pts]=time-lasttime;
		dlines[pts]=NR-lastNR;
		pts++;
		lastvolts=volts;
		lastNR=NR;
		Q=0;
		lasttime = time;
	}
}

END{
	tsum=0;
	for(i=1;i<pts;i++){
		tsum+=tinterim[i];
	}
	tscale = tsum/pts;	# find mean interval
	for(i=1;i<pts;i++){
		i_cv = Qinterim[i]/tinterim[i];
		printf("%e %e %d \r\n", Vinterim[i], i_cv, dlines[i] )	
	}
}

