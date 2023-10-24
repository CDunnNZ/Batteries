# Takes a .tvi file with regular cycle waveform and calculates u for each cycle and U across all cycles, and reports with a variety of other parameters including number of cycles, times, voltages, charge moved, energy and loss, and u variance
# Also outputs automatically a .tu file <filename>.tu, where <filename>.tvi is the input file
# Usage: awk -f getURegCycle.awk <filename>.tvi > output.txt (choose name of file, holds breakdown of data for each cycle, otherwise outputs to terminal; .tu file is generated either way) 
BEGIN{time=0.00; lasttime=0+0.00; volts=0+0.00; curr=0.00; dt=0.00; energy=0.00; dQ=0.00; cumdQ=0.00; lastdQ=0.00; lastdQ2=0.00; loss=0.00; timePeriod=0.00; count=0; usum=0.00; uusum=0.00; umean=0.00; uVar=0.00; E=0.00; L=0.00; totQ=0.00}
{
    time=$1+0;
    volts=$2+0;
    curr=$3+0;
    dt=time-lasttime;
    lasttime=time;
    energy += dt*volts*curr;
    lastdQ = dQ;
    dQ = curr*dt;
    cumdQ += dQ;
    loss += dt*curr*curr;

    if( (sign(dQ)!=sign(lastdQ)) && (sign(dQ)==sign(lastdQ2)) ){
    	u = energy/loss;
	if(count>0){
	    printf("%.3f \tV=%.3f dQ=%f \tE=%f \tI2=%f \tu=%f \tT %.1fh Cycle %d\n", timePeriod/3600, volts, cumdQ, energy, loss, u, (time-timePeriod)/3600, count);
	    split(FILENAME,array,".");
	    printf("%.3f %f\n", timePeriod/3600, u)>array[1]"RegCyc.tu";
	    uArray[count]=u;
	    usum+=u;
	    E+=energy;
	    L+=loss;
	    totQ+=cumdQ;
	}
	timePeriod = time;
	lastdQ2=dQ;
	energy=0.00;
	loss=0.00;
	cumdQ=0.00;
	count=count+1;
    }
}
END{
    ucount=count-1;
    umean=usum/ucount;
    for(i=1; i<=ucount; i++)
	uusum+=(uArray[i]-umean)*(uArray[i]-umean);    
    uVar=uusum/(ucount-1);
    printf("Mean u = %.6f, variance = %.6f or %.3f%%, %s cycles\n", umean, uVar, 100.00*uVar/umean, ucount);
    printf("Whole file U = %.3f, total dQ = %.3f (%.3fAh)\n", E/L, totQ, totQ/3600);
}

function abs(x){
    return x<0?-x:x
}

function sign(x){
    if(x<=0) return -1;
    else return 1;
}

function near(x,y,z){if(x>y){
	if(x-y<z){
	    return 1;
	}else{return 0;}
	}else{
	if(y-x<z){
	    return 1;
	}else{
	    return 0;
	}
    }
}
