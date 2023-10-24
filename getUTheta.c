#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#define PI 3.14159265358979323846

int sign(double x){
  if(x<=0) return -1;
  else return 1;
}

int main(int argc, char *argv[]) //*argv[] is an array of pointers
{
  double time=0.00, volts=0.00, curr=0.00, origv=0.00, vmax=0.00, vmin=100.00;
  double current=0.00, voltage=0.00, V0=0.00, Va=0.00, CPEvoltage=0.00, currentMean=0.00, currentRMS=0.00, C_K=0.00;
  double dt=0.00, lasttime=0.00, timePeriod=0.00, dtmin=0.00, dtmax=0.00, accumtime=0.00, origtime=0.00, totalTime=0.00;
  double dQ=0.00, lastdQ=0.00, cumdQ=0.00, u=0.00, uLast=0.001, lastdQ2=0.00, allQ=0.00, allQSq=0.00, wholeMeanQ=0.00, wholeMeanQSq=0.00;
  double Ein=0.00, Eout=0.00, totQ=0.00, totIn=0.00, totOut=0.00, Rs=0.00;
  int count=0, ucount=0;
  double usum=0.00, uusum=0.00, umean=0.00, uVar=0.00, costheta=0.00, theta=0.00, alpha=0.00;
  //double uArray[12];
  FILE *fileptr; //points to a random address in memory; initialised later with &time, &volts, etc.
  char *sline;
  size_t li;

  //float version=1.1f; //dynamic memory allocation added
  //float version=1.2f; //addition of prevoltages and currents to calculate u; increase similarity to getSoH
  //float version=1.3f; //reworking of when to start recording u in order to eliminate early outliers
  //float version=1.4f; //addition of 'if' statement to catch non-monotonic timestamps
  //float version=1.5f; //reinstatement of Eout/Ein calculation and addition of C_K correction factor
  //float version=1.0f; //first version of getUTheta
  float version=1.1f; //Addition of Rs as a parameter to be optionally passed in

  char year[20]="July 2023";

  if(argc<2 || argc>3){
    fprintf(stderr, "\ngetUTheta version %.2f Chris Dunn %s\n\n", version, year);
    fprintf(stderr, "Usage: getUTheta inputfile.tvi [Rs] >outputfile.tu 2>resultsfile.txt\n");
    fprintf(stderr, "Rs (optional) = series resistance.\n");
    fprintf(stderr, "Takes a .tvi (3-column ascii) file for a regular waveform,\n");
    fprintf(stderr, "works out 'u' for each period and overall 'U' across all periods,\n");
    fprintf(stderr, "and writes period start times and u values to stdout.\n");
    fprintf(stderr, "getUTheta also calculates and writes out the phase angle for the CPE,\n");
    fprintf(stderr, "from which we may infer alpha, and adds theta (in radians) and alpha to stdout.\n");
    fprintf(stderr, "Note: suitable only for regular waveforms.\n");
    fprintf(stderr, "For irregular and self-similar cycles use 'getSoH'.\n\n");
    exit(1);
  }

  fileptr=fopen(argv[1], "rb");

  if(fileptr==NULL){
    fprintf(stderr, "Cannot open .tvi file %s!\n", argv[1]); //check for the .tvi file
    exit(1);
    }
    if(argc==2){
      fprintf(stderr, "Rs set to zero\n");
    }
    if(argc==3){
      Rs = atof(argv[2]);
      fprintf(stderr,"Rs set to %s\n",argv[2]);
    }
    // while loop initialises *fileptr line by line
    //while(fscanf(fileptr, "%f %f %f %f %d", &time, &volts, &curr, &dump, &cyc)!=EOF)
    while(getline(&sline, &li, fileptr)>0 && sscanf(sline,"%le %le %le", &time, &volts, &curr)==3){
      time+=accumtime;
      if(time<=lasttime){
	accumtime = lasttime;
	time = lasttime+dt;
	fprintf(stderr,"Non-monotonic time: accumulated time = %.1lf; dtmax=%.2lf, dtmin=%.2lf\n", accumtime, dtmax, dtmin);
      }
      dt=time-lasttime;
      if(dt>dtmax){dtmax=dt;}
      if(dt<dtmin){dtmin=dt;}
      if(volts>vmax){vmax=volts;} //get vmax for theta calculation
      if(volts<vmin){vmin=volts;} //get vmin for theta calculation
      lasttime=time;
      voltage=volts;
      current=curr;
      CPEvoltage=voltage+Rs*current;
      if(current>0){
	Ein+=dt*CPEvoltage*current;
      }else{
	Eout-=dt*CPEvoltage*current;
      }
      lastdQ=dQ;
      dQ=current*dt;
      cumdQ+=dQ;
      allQ+=dt*fabs(current);
      allQSq+=dt*current*current;

      if(sign(dQ)!=sign(lastdQ) && sign(dQ)==sign(lastdQ2) && dQ!=-0.00){
	if(count>0){
	  u=Eout/Ein;
	  Va=(vmax-vmin)/2;
	  V0=Va+vmin;
          costheta=(2*V0*(1-u))/(PI*Va);
          theta=acos(costheta);
          alpha=theta*2/PI;
          vmax=0; //reset vmax
          vmin=100; //reset vmin

	  if(fabs((u-uLast)/uLast)<0.1){//wait for 'u' to settle down
	    if(ucount<1){origv=voltage; origtime=timePeriod;}
	    ucount++;
	    fprintf(stderr, "%.3lf \tV=%.3lf \tdQ=%.3lf \tu=%.6lf \tTheta=%.2lf degrees \tAlpha=%.3lf \tT %.1lfh Cycle %d\n", timePeriod/3600, volts, cumdQ, u, theta*180/PI, alpha, (time-timePeriod)/3600, count);
	    fprintf(stdout, "%.3lf %lf %lf %lf\n", timePeriod/3600, u, theta, alpha);
	    //fprintf(stderr, "vmax=%.3lf vmin=%.3lf Va=%.3lf V0=%.3lf\n", vmax, vmin, Va, V0);
	    usum+=u;
	    uusum+=u*u;
	    totQ+=cumdQ;
	    totIn+=Ein;
	    totOut+=Eout;
	    wholeMeanQ+=allQ;
	    wholeMeanQSq+=allQSq;
	  }
	}
	uLast=u;
        Ein=Eout=0.00;
        timePeriod=time;
        lastdQ2=dQ;
        cumdQ=0.0f;
        allQ=0.00;
        allQSq=0.00;
        count++;
      }
    } //end of while loop

    umean=usum/ucount;
    uVar=(uusum - usum*usum/ucount)/(ucount-1);
    totalTime=timePeriod-origtime;
    
    //For future use
    //currentMean=wholeMeanQ/totalTime;
    //currentRMS=sqrt(wholeMeanQSq/totalTime);
    //C_K=1+(currentMean/currentRMS-1)/50.00;

    fprintf(stderr, "Mean u = %.6lf, variance = %.3e (%.3e%%), SD = %.3e, %d cycles\n", umean, uVar, 100*uVar/umean, sqrt(uVar), ucount);
    fprintf(stderr, "Whole file U = %.3lf, adjusted U = %.3lf, total dQ = %.3lf (%.3fAh), start time = %.3lf (%.3lfh), starting voltage = %.3lf\n", totOut/totIn, C_K*totOut/totIn, totQ, totQ/3600, origtime, origtime/3600, origv);
    //fprintf(stderr, "usum=%.3lf uusum=%.3lf ucount=%d \n", usum, uusum, ucount);
}
