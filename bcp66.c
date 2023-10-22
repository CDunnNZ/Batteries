// Program to cycle a battery using an HP/Agilent/Keysight 66332A on Raspberry Pi via Prologix 
// JBS Nov 2020

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>
#include    <math.h>
#include    <fcntl.h>
#include    <errno.h>
#include 	<unistd.h> // write(), read(), close()
#include <termios.h>

#define MAX(A,B) (((A)>(B))?(A):(B))
#define MIN(A,B) (((A)<(B))?(A):(B))
#define TRUE 1
#define FALSE 0

#define DIAG 0

FILE *logfile;				// to log errors
#include "prologix.h"

int main(int argc, char* argv[])
{
	FILE *tvi;
	int hp;
	int i;
	char USBpath[32];
	char rbuf[256],wbuf[256];
	char message[128];
	time_t tstart,tnow,tmark;
	struct timespec ts, tn;
	double deltat=0.00,meastime,lastmeastime;
    double Ich, Idis, Ich_end, Idis_end, iset;
    double Vmin, Vmax, vnow=0.00, inow=0.00;
    double batQ=0.00, Qmax=0.00;
    float Qfinal, fsmax=0.94, Tsmin, Tsincesec;
    int tdwellplus,tdwellminus,ncyc,tfinal,cycle=0;
	int dwell;
	int ccmodeCounter;
    int state, finished=0, CCmode;
    char *statename;
    long int npts=0, ets, nlines=0;
    int gpibaddr=5, argcnt=0;
    char baseName[64], logfname[128];
	struct termios spset;
	int restplus=0, restminus=0;

	// version 1.0: adjusted for 66332A
	// version 1.01: fixed ets/meastime check
	// version 1.03: removed ets/meastime check, fixed SET phase bug
	// version 1.04: add port setup code to defeat minicom preconditioning requirement
	// version 1.05: reset dwell at state change to prevent skipping
	// version 1.06: add check for crazy current value
	// version 1.07: init inow and deltat to zero to stop (?) silly batQ values at start
	// version 1.08: add option for rest (I=0) during tdwell
    float version = 1.08;

    if (argc<13+1 || argc>15+1) { // ??
        fprintf(stderr,"bcp66 V%.2f jbs Nov 2020, June 2021, Sep 2021\n", version);
        fprintf(stderr,"Battery Cycler via Prologix gpib to HP/Agilent 66332A.\n");
        fprintf(stderr,"%d parameters is illegal.\n", argc-1);
        fprintf(stderr,"Usage: bcp66 USB Vmax Vmin Ich Idis I+end I-end tdwell+ tdwell- ncyc Qfinal tfinal baseName [fsmax [Addr]]\n");
		fprintf(stderr,"where-  USB is the rPi USB address (/dev/ttyUSB0, etc);\n");
		fprintf(stderr,"        Vmax/Vmin are charge/discharge 'CV' voltages;\n");
        fprintf(stderr,"        Ich/Idis are the charge and discharge 'CC' currents;\n");
        fprintf(stderr,"        I+end/I-end are currents at which to end the CV phases;\n");
        fprintf(stderr,"        tdwell+ is the max period (s) to wait at CV for I+end;\n");
        fprintf(stderr,"        tdwell- is the max period (s) to wait at CV for I-end;\n");
        fprintf(stderr,"        Qfinal is the %% charge in Ah at the end of cycling;\n");
        fprintf(stderr,"        ncyc is the # cycles to perform;\n");
        fprintf(stderr,"        tfinal is the period (s) to digitise after current is zero;\n");
        fprintf(stderr,"        baseName is the file string to be used;\n");
        fprintf(stderr,"        optionally Addr is the GPIB bus address, def=%d.\n",gpibaddr);
        fprintf(stderr,"        optionally fsmax is the max sample frequency, def=%.2f.\n",fsmax);
        fprintf(stderr,"Cycles the battery by the CCCV method, while measuring V & I,\n");
        fprintf(stderr,"~8 samples/second, tvi file entries limited by fsmax.\n");
        fprintf(stderr,"If tdwell+/- is <0, the period is set and current is set to zero (CV->rest).\n");
        fprintf(stderr,"Assuming a 66332A instrument on Prologix/Fenrir USB-GPIB interface.\n");
        fprintf(stderr,"Requires no drivers, controls prologix using ++cmd protocol.\n");
        fprintf(stderr,"Creates basename.log & baseName.tvi with time-volts-amps-dQ-cyc quintuples.\n");
        fprintf(stderr,"Displays: #points, elapsed time, V, I, cycle, Tsample, CV time, dQ, and CC/CV mode.\n");
        fprintf(stderr,"\n");
        exit(1);
    }

    // process input arguments
	strcpy(USBpath,argv[++argcnt]);								// /dev/ttyUSBx
	if(strstr(USBpath,"tty")==NULL) err("Bad USB address?");	// Raspbian check
	if(strstr(USBpath,"dev")==NULL) err("Bad USB address?");	// Raspbian check

    Vmax = atof(argv[++argcnt]);
	if(Vmax<0.9) err("Vmax is too small");
	if(Vmax>20.0) err("Vmax is too large");

    Vmin = atof(argv[++argcnt]);
	if(Vmin<0.25) err("Vmin is too small");
	if(Vmin>15.0) err("Vmin is too large");

    Ich = atof(argv[++argcnt]);
	if(Ich<0.001) err("Ich too small");
	if(Ich>5.10) err("Ich is too large");

    Idis = atof(argv[++argcnt]);
	if(Idis<0.001) err("Idis too small");
	if(Idis>5.10) err("Idis is too large");

    Ich_end = atof(argv[++argcnt]);
	if(Ich_end<1e-3) err("Ich_end too small");
	if(Ich_end>5.0) err("Ich_end is too large");
	if(Ich_end>=Ich) err("Ich_end not less than Ich");

    Idis_end = atof(argv[++argcnt]);
	if(Idis_end<1e-3) err("Idis_end too small");
	if(Idis_end>5.0) err("Idis_end is too large");
	if(Idis_end>=Idis) err("Idis_end not less than Idis");

	tdwellplus = atoi(argv[++argcnt]);
	if(tdwellplus<0){tdwellplus=-tdwellplus;restplus=1;}
	if(tdwellplus<5) err("tdwellplus must be at least 5 seconds.");
	if(tdwellplus>605000) err("tdwellplus must be less than 1 week, 604ksec.");

	tdwellminus = atoi(argv[++argcnt]);
	if(tdwellminus<0){tdwellminus=-tdwellminus;restminus=1;}
	if(tdwellminus<5) err("tdwellminus must be at least 5 seconds.");
	if(tdwellminus>87000) err("tdwellminus must be less than 1 day, 87ksec.");

	ncyc = atoi(argv[++argcnt]);
	if(ncyc<1) err("Bad number of cycles.");
	if(ncyc>1000) err("Count exceeds 1000 cycles.");

	Qfinal = atof(argv[++argcnt]);
	if(Qfinal<0.1) err("Qfinal less than 0.1\%.");
	if(Qfinal>99.9) err("Qfinal is more than 99.9\%.");

	tfinal = atoi(argv[++argcnt]);
	if(tfinal<1) err("tfinal must be at least 1 second.");
	if(tfinal>605000) err("tfinal must be less than 1 week, 604ksec.");

	strcpy(baseName,argv[++argcnt]);

	if(argc>argcnt+1) fsmax = atof(argv[++argcnt]);
	if(fsmax>10.0 || fsmax<0.01) err("fsmax must be 10>fsmax>1/100.\n");
	Tsmin=1.0/fsmax;	// shortest reporting period for tvi file

	if(argc>argcnt+1) gpibaddr = atoi(argv[++argcnt]);
	if(gpibaddr<1 || gpibaddr>30) err("Bad GPIB_Address given.\n");

	// now open a log file for problem/progress reports
	strcpy(logfname,baseName);
	strcat(logfname,".log");
	logfile = fopen(logfname,"w+");					// log file open 

	time(&tstart);									// note the time of start
	sprintf(wbuf,"%s started, logfile opened, at %s",argv[0],ctime(&tstart));
	progress(wbuf);
	for(wbuf[0]='\0',i=0;i<argc;i++){strcat(wbuf,argv[i]);strcat(wbuf," ");}
	progress(wbuf);

	// find interface
	hp = open(USBpath,O_RDWR|O_NONBLOCK);	// open read & write ASCII, without hanging
	if(hp<0) {
		fprintf(stderr,"Error %i from open: %s\n", errno, strerror(errno));
		err("Cannot open the device.");
	}
	progress("Handle opened.");
	if (tcgetattr(hp, &spset) < 0) {
		err("Cannot get port attributes.");
	}
	cfmakeraw(&spset); // added in 1.04 to fix port control
	if (tcsetattr(hp, TCSANOW, &spset) < 0) {	// raw port now
		err("Cannot set port attributes.");
	}

	msg("Setting up prologix interface... ");
	initPrologix(hp);								// set ip inteface
	sprintf(wbuf,"++addr %d\n", gpibaddr);
	wrtstr(hp,wbuf);								// point to our instr


	// grab bus, terminate communications, ID instrument
	msg("Clearing GPIB bus... ");
	sprintf(wbuf,"++ifc\n");wrtstr(hp,wbuf);		// send INTERFACE CLEAR
	sprintf(wbuf,"++clr\n");wrtstr(hp,wbuf);		// send CLEAR to hp itself
	msg("Waiting while bus clears... ");
	tickle(1000);									// allow stuff to happen

	msg("Check ID of Instrument... ");
	wrtstr(hp,"*IDN?\n");						// ask for IDN
	getmsg(hp,rbuf);							// get message from addr
	if(strstr(rbuf,"66332")==NULL){
		fprintf(stderr,"Instrument at %d identifies as:'%s' (%d chars)",gpibaddr,rbuf,strlen(rbuf));
		err("Bad instrument ID");
	}else{
		progress(rbuf);
	}

	// initialize HP function
	msg("Setting up hp... ");
	sprintf(wbuf,"*RST;\n");wrtstr(hp,wbuf); 		// init ADC parameters
	tickle(2500);
	if(Ich>0.02 || Idis>0.02){										// big currents
		sprintf(wbuf,"SENSe:CURRent:RANGe MAX\n");wrtstr(hp,wbuf); 	// 5A range
	}else{
		sprintf(wbuf,"SENSe:CURRent:RANGe MIN\n");wrtstr(hp,wbuf); 	// 20mA range
	}

	msg("Checking for instrument errors... ");
	do{
		i=0;
		sprintf(wbuf,"SYST:ERR?;\n");wrtstr(hp,wbuf); 	// check for errors
		getmsg(hp,rbuf);
		sscanf(rbuf,"%d",&i);
		if(i){progress(rbuf);msg(rbuf);}
	}while(i);

	// now open tvi file
	strcpy(logfname,baseName);
	strcat(logfname,".tvi");
	tvi = fopen(logfname,"w+");							// tvi file open 
	if(tvi==NULL) err("Cannot open tvi file");


#define CHARGE 1
#define DISCHARGE 2
#define PRECHARGE 3
#define POSTSET 4
#define EQUILIBRATE 5
#define GIG 1000000000.00
	state=CHARGE;									// start going up to Vmax
	CCmode=TRUE;ccmodeCounter=0;					// assume in CC mode to start
	inow=Ich;										// assume I large (not decayed)
	time(&tmark);									// time in seconds for dwells
	clock_gettime(CLOCK_REALTIME, &ts);				// present into ts(tart) structure
	clock_gettime(CLOCK_REALTIME, &tn);				// present into tn(ow) structure
	meastime = (tn.tv_sec-ts.tv_sec)+(tn.tv_nsec-ts.tv_nsec)/GIG; // init meastime
	wrtstr(hp,"OUTP ON;\n"); 						// enable output
	Tsincesec=0.0;									// no time elapsed since last tvi file entry
	msg("Commencing main state-machine loop... ");
	while(!finished){

		// in Raspbian, use clock_gettime()
		lastmeastime = meastime;					// deal with time
		clock_gettime(CLOCK_REALTIME, &tn);			// present into tn(ow) structure
		meastime = (tn.tv_sec-ts.tv_sec)+(tn.tv_nsec-ts.tv_nsec)/GIG; // init meastime
		deltat=meastime-lastmeastime;				// time since last meas
		Tsincesec+=deltat;							// time since last tvi log
		time(&tnow);
		ets = (long)tnow-tstart;					// total Elapsed Time in Secs

		do{
			wrtstr(hp,"MEAS:VOLT?\n"); 			// request the terminal voltage
			getmsg(hp,rbuf);					// read it...
			sscanf(rbuf,"%lf",&vnow);  			// scanf it...
			wrtstr(hp,"MEAS:CURR?\n"); 			// request the terminal CURRENT ch1
			getmsg(hp,rbuf);  					// read it...
			sscanf(rbuf,"%lf",&inow);    		// scanf it...
		}while(inow>100.0 || inow<-100.0);		// crazy result

		if(npts%64==0){						// every so many cycles
			do{
				i=0;
				sprintf(wbuf,"SYST:ERR?\n");wrtstr(hp,wbuf); 	// check for errors
				getmsg(hp,rbuf);
				sscanf(rbuf,"%d",&i);
				if(i){progress(rbuf);msg(rbuf);}
			}while(i);
		}

		switch(state){									// chg/dischg/etc state machine
			default:
			case CHARGE:
				statename="CHG";
				if(restplus && !CCmode){ iset=0.00; }else{ iset=fabs(Ich); }
				sprintf(wbuf,"VOLT %.4lf;CURR %.4lf;\n",Vmax,iset); // set V & I
				wrtstr(hp,wbuf);									// send	
				time(&tnow);dwell=tnow-tmark;
				if(CCmode){time(&tmark);}
				else{											// out of CC
					if(dwell>tdwellplus || (!restplus && inow<Ich_end) ){ // charge done
						if(cycle!=0){							// just done ch/dis cycle
							fprintf(logfile,"# Cycle=%d, dQ=%.1lfC, %sAh\n",cycle,batQ,sengstr(batQ/3600.0,3));
							//Qmax=MAX(Qmax,fabs(batQ));// dont keep chg capacity
						}
						progress("Completed a cycle.");
						fprintf(logfile,"Charge transferred %sC, %sAh\n",sengstr(batQ,3),sengstr(batQ/3600.0,3));
						cycle++;								// inc # cycle
						batQ=0.00;								// reset charge counter
						dwell=0;				// no dwell any more
						if(cycle>ncyc){							// done cycling
							state=POSTSET;						// go to Qset phase
							time(&tmark);						// reset dwell time
							msg("Moving to charge setting phase...");
							progress("Moving to charge setting phase...");
							CCmode=TRUE;ccmodeCounter=0;		// set safe for next state
						}
						else{									// start next cycle
							state=DISCHARGE;
							msg("Moving to DISCHARGE...");
							progress("Moving to DISCHARGE...");
						}
						CCmode=TRUE;ccmodeCounter=0;			// set safe for next state
					}
				}
			break;
			case DISCHARGE:
				statename="DIS";
				if(restminus && !CCmode){ iset=0.00; }else{ iset=fabs(Idis); }
				sprintf(wbuf,"VOLT %.4lf;CURR %.4lf;\n",Vmin,iset);		// set V & I
				wrtstr(hp,wbuf);											// send
				time(&tnow);dwell=tnow-tmark;
				if(CCmode){time(&tmark);}							// reset dwelltime
				else{
					if(dwell>tdwellminus || (!restminus && fabs(inow)<Idis_end)){ // discharge done
						fprintf(logfile,"Charge transferred %sC, %sAh\n",sengstr(batQ,3),sengstr(batQ/3600,3));
						Qmax=MAX(Qmax,fabs(batQ));					// keep capacity
						batQ=0.00;									// reset charge counter
						state=CHARGE;								// start next cycle
						dwell=0;				// no dwell any more
						time(&tmark);
						msg("Moving to CHARGE...");
						progress("Moving to CHARGE...");
						CCmode=TRUE;ccmodeCounter=0;				// safe for next state
					}
				}
			break;
			case POSTSET:										// on way to prescribed Q
				statename="SET";
				iset=fabs(Idis);
				sprintf(wbuf,"VOLT %.4lf;CURR %.4lf;\n",Vmin,iset); // set V & I
				wrtstr(hp,wbuf);										// send	
				if(Qmax<0.001){
					wrtstr(hp,"OUTP OFF;\n"); 						// disable output
					err("Qmax too small... aborting SET phase");
				}
				if(fabs(batQ)/Qmax>(1-Qfinal/100.0)){					// Q low enough
					time(&tmark);
					state=EQUILIBRATE;
					msg("Moving to final settling phase...");
					progress("Moving to final settling phase...");
				}
			break;
			case EQUILIBRATE:
				statename="EQU";
				time(&tnow);dwell=tnow-tmark;
				sprintf(wbuf,"VOLT %.4lf;CURR %.4lf;\n",(Vmax+Vmin)/2.0,0.00); // set V & I
				wrtstr(hp,wbuf);											// send			
				if(dwell>tfinal)finished=TRUE;
			break;
		}

		// ------ in CC or CV? ------
		if(relerr(iset,fabs(inow))<0.05){		// see if current limit is close to actual current
			ccmodeCounter+=1;					// increase confidence in CC state
			if(ccmodeCounter>5)ccmodeCounter=5;	// limit value so confidence does not last too long
		}else{
			ccmodeCounter-=1;					// decrease confidence in CC state
			if(ccmodeCounter<-5)ccmodeCounter=-5; // to sensible limit
		}
		if(ccmodeCounter>=4){CCmode=TRUE;}		// confident hp in CC mode?
		if(ccmodeCounter<=-4){CCmode=FALSE;}	// confident hp in CV mode?
		// ------ CC or CV ------

		batQ += inow*deltat;					// sum charge moved

		if(npts++ && Tsincesec>Tsmin){					// not first point, OK to log
			Tsincesec=0.0;
			nlines+=1;
			fprintf(tvi, "%.3lf %.3lf %.3lf  %s %d\n", meastime,vnow,inow,engstr(batQ/3600.0,3),cycle);fflush(tvi);
		}
		sprintf(wbuf, "pt%ld/%ld: %lds, V=%.3lf I=%s cyc=%d dt=%.2lfs dwell=%d Q=%sAh C%c %s", 
					nlines,npts,ets,vnow,sengstr(inow,3),cycle,deltat,dwell,
						sengstr(batQ/3600.0,3),CCmode?'C':'V',statename);
		msg(wbuf);								// screen display
		if(ccmodeCounter>-2 && ccmodeCounter<2){
			progress(wbuf);						// log a display line before mode changes
		}
	}
	wrtstr(hp,"OUTP OFF;\n"); 						// disable output

	time(&tnow);
	sprintf(wbuf,"bcp66 done (took %ld secs, %.1f hours).\n",tnow-tstart,(tnow-tstart)/3600.00);	// display we are finished
	msg(wbuf);
	progress(wbuf);

	fclose(tvi);							// tvi file intact, so close
	fclose(logfile);						// close log file as well
}

