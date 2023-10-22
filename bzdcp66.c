// Program to measure battery impedance using HP66332 and Prologix/Fenrir GPIB on Raspberry Pi
// with added dc current
// optionally includes frequency/z analysis by system calls to dftp & ff
// JBS Dec 24, 2020, cloned from bzp66.c 11 Aug 2021 by JBS & CJD
// CJD & JBS 31 Oct 2021, special update to correct drifting Q caused by DAC resolution problem

//*************Source/range instructions from bzdcpk****************************
// sprintf(wbuf,":SOURce:VOLTage:RANGe %f;\n",1.011*Vmax); // set V range (plus a bit)(v6)
//******************************************************************************

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>
#include    <math.h>
#include    <fcntl.h>
#include    <errno.h>
#include    <complex.h>
#include 	<unistd.h> // write(), read(), close()
#include 	<termios.h>


#define MAX(A,B) (((A)>(B))?(A):(B))
#define MIN(A,B) (((A)<(B))?(A):(B))
#define TRUE 1
#define FALSE 0

#define GIG 1000000000
#define NFREQS 32
#define PI 3.141592654
#define TWOPI 2*3.141592654
#define MAX(A,B) (((A)>(B))?(A):(B))
#define MIN(A,B) (((A)<(B))?(A):(B))
#define TRUE 1
#define FALSE 0
#define SIGN(X) (((X)>(0))?(1):(-1))

FILE *logfile;										// to log errors
double meastime;

#include "prologix.h"

int main(int argc, char* argv[])
{
	FILE *tvi;
	int hp,skip=FALSE;
	char USBpath[64];
	char rbuf[256], wbuf[128];
 	time_t tstart,tnow,tmark;
	struct timespec ts, tn;
    double lastmeastime, lastvb, lastib;
    int gpibaddr=5;
    char baseName[64], logfname[128];
    float ncyc,fmin,fmax,Vmin,Vmax,Imax,ftmp,Xcyc;
    double deltaQ,ib,vb,Istim,dQ,dt,dtmp;
    double freq, f[NFREQS], period;
    double a[NFREQS], ph[NFREQS], iqf;
    double Ibiggest=-100.0, Ismallest=100.0;
    double dQbiggest=-1000.0, dQsmallest=1000.0;
    int i,j, nf=0, narg=0, npts=0, datvoid,scpi;
    double mag,pha,fcheck,discard;
    double imag[NFREQS],vmag[NFREQS],ipha[NFREQS],vpha[NFREQS];
    int getz=FALSE,refine=FALSE,readFreqs=FALSE;
	struct termios spset;
	double actualImax,Imultiplier;
	int qloops, eqI=FALSE;
	double Idc, fdc, dQdc, fracycle, tdc;
	unsigned char fase; // phase of Idc cycle
	double errpc;
	int sink=FALSE;		// flag for using only negative current
	//***********************************************************************
	double itrim=0.00, i_error=0.00;	// DAC resolution trim out current, apparent error current (v6)
	int midcycle=FALSE, last_midcycle=FALSE; // to track switches in squarewave
	double dQtarget, last_dQtarget, Qerror, actualdQ=0.0;	 // expected delta charge (v6)
	//************************************************************************

	double deltaf,minDeltaf,badf;

	FILE *fmp, *ffz, *bat, *frq, *gs, *ff;
	char dftname[256],ffname[256], cmd[256];
    complex double cf[NFREQS], vf[NFREQS], z[NFREQS];

	// version 2.01: fixed bug in processing .frq file (non-numeric lines ignored)
	// version 2.02: updated instructions, cleaning up error checking, prologix.h
	// version 2.03: fix Imax bug. project finish time
	// version 2.04: put version # into logfile, adjust limits of freq & cycles
	// version 2.05: fix frequency limits bug 
	// version 2.06: fix dQ bug
	// version 2.07: Schroeder phase distribution for flat spectrum
	// version 2.08: expand v limit diagnostic message and turn OP off if err
	// version 2.09: fix qf bug
	// version 2.10: extend fmin
	// version 2.11: add serial control parts to set up port without minicom
	// version 2.12: fix ambiguities in instructions
	// version 2.13: Optimisation of I_max distribution between tones (29 Mar 2021)
	// version 2.14: fixed infinite loop in q-scrunching/sharing algorithm (8 Jun 2021)
	// version 2.15: fixed inability to reach below 1uHz with automatic frequency setting
	// version 2.16: added equal-I tone amplitudes option
	// version 4.00: cloned from bzp66
	// version 4.01: fix I & Q bugs
	// version 4.02: fix dQ checking
	// version 4.03: check many more harmonics of fdc for collision
	// version 4.04: bug on rPi regarding #fundamentals spaced out?
	// version 4.05: added 5th and 7th harmonics of fdc to .bat file
	// version 4.06: DAC resolution bug fixed - prevents drifting Q during measurement
	// version 6.00: minor changes to what is displayed while running, Vmax/Vmin targets
	// version 6.01: current trim acceleration factor = 1.2
	// version 6.02: current trim acceleration factor = 1.1
	// version 6.03: change readout/logfile data format to more friendly
	// version 6.10: changed feedback on itrim to fix should-never-happen errors
	// version 6.11: changed -- to == in log output to make search easier with grep... duh.
	// version 6.12: add sink-only option
	// version 6.13: phase of lowest tone set to cos to minimise dQ
	// version 6.14: clean up progress reporting text
	// version 6.15: increase itrim multiplier
	// version 6.16: change log info for dQdc, initialise itrim
	// version 6.17: fix bug in equal-I tone sizing
	// version 6.18: multiplier back to 1.2, allow 1% dQ overshoot
	// version 6.19: attempt to fix dQ drift, allow == updates periodically without zero cross
	// version 6.20: rework application of itrim, improve logfile diagnostics
	// version 6.21: allow zero Idc
    float version = 6.21; 
    if (argc<12+1 || argc>14+1) { // ??
        fprintf(stderr,"bzdcp66 ------------  V%.2f jbs Dec 2020 -> Nov 2021\n", version);
        fprintf(stderr,"Battery Z measurement with dc, via Prologix/Fenrir GPIB-USB & 66332A, optional DFT.\n");
        fprintf(stderr,"Usage: bzdcp66 USB Vmin Vmax Imax dQmax ncyc fmin fmax Xcyc Idc fdc baseName [Addr [dftp]]\n");
        fprintf(stderr,"where-  USB is the rPi USB address (/dev/ttyUSB0, /dev/ttyACM0, etc);\n");
	fprintf(stderr,"        Vmin/Vmax are voltage limits (aborts outside this range);\n");
        fprintf(stderr,"        Imax is the maximum multitone current in the cell (if <0 only sinks I);\n");
        fprintf(stderr,"        dQmax is the total charge in Ah that can be sourced/sunk (-val => equal I tones);\n");
        fprintf(stderr,"        ncyc is the # cycles at fmin (typically 2.01-6.00);\n");
        fprintf(stderr,"        fmin/fmax are the lowest and highest freqs;\n");
        fprintf(stderr,"        Xcyc is the # cycles at fmin of data to discard before logging;\n");
        fprintf(stderr,"        Idc is the magnitude of the added dc current component, and\n");
        fprintf(stderr,"        fdc is the frequency of the squarewave at Idc;\n");
        fprintf(stderr,"        baseName is the file string to be used;\n");
        fprintf(stderr,"        Addr is the optional GPIB bus address, def=%d.\n",gpibaddr);
        fprintf(stderr,"        dftp is the [path]name of Scott/Farrow dft program (dftp,dvtv,etc);\n");
        fprintf(stderr,"Makes a multitone tvi/Z measurement by sourcing current, measuring V & I.\n");
        fprintf(stderr,"If the USB parameter is set to \"skip\" the tvi measurement is skipped.\n");
        fprintf(stderr,"NB: If Imax<0 battery is discharging, so dQmax boundary checking is ignored.\n");
        fprintf(stderr,"Creates baseName.tvi, basename.log, [.bat, .fmp] files.\n");
        fprintf(stderr,"Z optionally computed by calls to dftp at each frequency.\n");
        fprintf(stderr,"The .bat file is dft script, fmp has dft's z values.\n");
        fprintf(stderr,"Frequencies are a 1-2-5 sequence between fmin and fmax;\n");
        fprintf(stderr,"if fmax<0, frequencies are read from baseName.frq file, up to %d freqs.\n",NFREQS);
        fprintf(stderr,"fdc should be chosen so that none of its harmonics clash with multitones.\n");
        fprintf(stderr,"Requires no drivers, communicates using ++cmd protocol.\n");
        fprintf(stderr,"Measures for (ncyc+Xcyc)/fmin seconds, then does dft calls.\n");
        fprintf(stderr,"Corrects for 1/2 LSB DAC error in 66332.\n");
        fprintf(stderr,"\n");
        exit(1);
    }
    
    // process input arguments
	strcpy(USBpath,argv[++narg]);									// /dev/tty????
	if(strstr(USBpath,"skip")!=NULL){ 
		skip=TRUE; 
	}else{
		if(strstr(USBpath,"tty")==NULL) err("Bad USB address?");
		if(strstr(USBpath,"dev")==NULL) err("Bad USB address?");
	}
	
    Vmin = atof(argv[++narg]);
	if(Vmin<0.2) err("Vmin is too small");
	if(Vmin>12.0) err("Vmin is too large");

    Vmax = atof(argv[++narg]);
	if(Vmax<=Vmin) err("Vmin exceeds/equals Vmax");
	if(Vmax>20.0) err("Vmax is too large");

    Imax = atof(argv[++narg]);
    if(Imax<0){
		Imax = -Imax;
		sink = TRUE;
	}
	if(Imax<5e-3) err("Imax too small");
	if(Imax>5.10) err("Imax is too large");	// max actually 5.12A

    deltaQ = atof(argv[++narg]);
    if(deltaQ<0){
    	deltaQ = fabs(deltaQ);
    	eqI=TRUE;
    }
	if(deltaQ<0.001) err("deltaQ is less than 1mAh");
	if(deltaQ>30) err("deltaQ is more than 30Ah");
	deltaQ = deltaQ*3600.0;								// convert to Amp-seconds

	ncyc = atof(argv[++narg]);
	if(ncyc<1.1) err("Too few cycles requested.");
	
    fmin = atof(argv[++narg]);
	if(fmin<0.1e-6) err("fmin is too small");
	if(fmin>0.5) err("fmin is too large");

    fmax = atof(argv[++narg]);
    if(fmax<0){fmax=fabs(fmax);readFreqs=TRUE;}
	if(fmax<1e-6) err("fmax is too small");
	if(fmax>2.5) err("fmax is too large");
	if(fmin>=fmax) err("Fmin>=Fmax");
	
    Xcyc = atof(argv[++narg]);
	if(Xcyc<0) err("Xcyc must be >=0.");
	if(Xcyc>6) err("Xcyc must be <6.");
	
    Idc = atof(argv[++narg]);
	if(Idc<0) err("Idc must be >=0mA.");
	if(Idc+Imax>5.0) err("Idc plus Imax must be less than 5A.");
	
    fdc = atof(argv[++narg]);
	if(fdc<0.75*Idc/deltaQ/2.0) err("fdc is too small given the Idc and dQmax.");

	strcpy(baseName,argv[++narg]);

	if(argc>++narg) gpibaddr = atoi(argv[narg]);
	if(gpibaddr<1 || gpibaddr>30) err("Bad GPIB_Address given.\n");

	// open a log file for problem/progress reports
	strcpy(logfname,baseName);
	strcat(logfname,".log");
	logfile = fopen(logfname,"w+");						// log file open 
	if(logfile==NULL) err("Cannot open log file.");
	// start logging
	time(&tstart);										// note the time of start
	sprintf(wbuf,"%s v%.2f started, logfile opened, at %s",argv[0],version,ctime(&tstart));
	wbuf[strlen(wbuf)-1]='\0';	// clip off newline
	progress(wbuf);
	for(wbuf[0]='\0',i=0;i<argc;i++){strcat(wbuf,argv[i]);strcat(wbuf," ");}
	progress(wbuf);

	// optional requests
	if(argc>++narg) {						// this param means we do the dft 
		getz=TRUE;
		strcpy(dftname,argv[narg]);			// program to call
		strcpy(logfname,baseName); strcat(logfname,".fmp");
		fmp = fopen(logfname,"w+");			// fmp file open
		if(fmp==NULL) err("Cannot open fmp file");
		progress(".fmp file open.");
		strcpy(logfname,baseName); strcat(logfname,".bat");
		bat = fopen(logfname,"w+");			// batch file open 
		if(bat==NULL) err("Cannot open bat file");
		progress(".bat file open.");
	}

// 	if(argc>++narg) {						// this param means we do ff
// 		refine=TRUE;
// 		strcpy(ffname,argv[narg]);			// ff program name to call
// 		progress("Opening .ffz file...");	// for refined fmp results
// 		strcpy(logfname,baseName); strcat(logfname,".ffz");
// 		ffz = fopen(logfname,"w+");			// ffz file open 
// 		if(ffz==NULL) err("Cannot open ffz file");
// 		progress(".ffz open.");
// 	}
	
	strcpy(logfname,baseName); strcat(logfname,".frq");
	if(readFreqs){							// from where to read f[]
		frq = fopen(logfname,"r");			// frq file open 
		if(frq==NULL) err("Cannot open frq file to read.");
	}else{
		frq = fopen(logfname,"w+");			// frq file open 
		if(frq==NULL) err("Cannot open frq file to write.");		
	}

	// compute frequencies
	for(i=0;i<NFREQS;i++){
		switch(i%3){							// is a 1-2-5 sequence/decade
			default:
			case 0: freq=1.0e-7; break;
			case 1: freq=2.0e-7; break;
			case 2: freq=5.0e-7; break;
		}
		freq *= pow(10.0,(i/3));
		if(freq>=fmin && freq<=fmax){
			f[nf++]=freq;
		}
		if(nf>=NFREQS)err("Requested too many frequencies");
	}
	if(readFreqs){							// overwrite f[] from file
		nf=0;
		while(nf<NFREQS && NULL!=fgets(rbuf,255,frq) ){	// for each line in frq file
			f[nf]=0; i=0;
			i=sscanf(rbuf,"%le",&f[nf]);
			if(i==1 && f[nf]>=fmin && f[nf]<=fmax){nf++;}
		}
	}else{									// write freqs to file
		for(i=0;i<nf;i++){
			fprintf(frq,"%s\n",engstr(f[i],6));
		}
	}fclose(frq);							// won't need this again
	if(nf>NFREQS)err("frq file contained too many frequencies");
	if(nf<1)err("frq file contained no acceptable frequencies");
	sprintf(rbuf,"Using %d frequencies.",nf);
	progress(rbuf);
	msg(rbuf);
	
	// check that fdc is a multiple of f[0]
	if( fabs((fdc/f[0]) - (int)((fdc+1e-9)/f[0])) > 1e-9 ) {
		sprintf(rbuf,"Squarewave frequency %s is not a multiple of fundamental %s (%s). \n",
			engstr(fdc,6),engstr(f[0],6),engstr((fdc/f[0])-(int)((fdc+1e-9)/f[0]),3) );
		progress(rbuf);
		err(rbuf);
	}
	sprintf(rbuf,"Squarewave frequency %s is %d * fundamental %s. ",
		engstr(fdc,6),(int)((fdc+1e-9)/f[0]),engstr(f[0],6) );
	progress(rbuf);

	// check for collisions between tones & squarewave
	minDeltaf=1000;
	badf=0.00;
	if(Idc!=0.00){
		for(i=0;i<nf;i++){		// for each tone frequency
			for(j=1;j*fdc<=f[nf-1];j++){	// for all harmonics of sq wave up to fmax
				deltaf = fabs(f[i]-j*fdc)/f[0];	// in multiples of fmin
				if(deltaf<minDeltaf){
					minDeltaf=deltaf;
					badf=j*fdc;
				}
			}
		}
		sprintf(rbuf,"Min frequency spacing is %.1lf fundamentals around %.9lf. ",minDeltaf,badf);
		progress(rbuf);
		msg(rbuf);
		if(minDeltaf<5.0){
			err("Less than 5x will pollute data. Aim for >20. \n");
		}
	}

	// compute amplitudes & phases
	progress("Finding mag & phases of tones... ");
	actualImax=0.00;
	Imultiplier=1.00;
	qloops=0;
	dQdc = Idc/(2.0*fdc);
	if(dQdc>0.95*deltaQ)err("Q consumed by dc squarewave would exceed 95% of deltaQ permitted!");
	sprintf(wbuf,"dQdc = %s Ah",engstr(dQdc/3600.0,3)); progress(wbuf);
	while(++qloops<3000 && actualImax<(Imax) && actualdQ<(deltaQ-dQdc) ){		// push up I, if 3k pushes, give up
		for(actualImax=0.00, actualdQ=0.00,i=0;i<nf;i++){
			iqf=((deltaQ-dQdc)/nf)*f[i]*PI;				// scrunch Q value at this freq
			a[i]=MIN((Imax)*Imultiplier/nf,iqf);	// I is (1/nf) of max or scrunched value
			if(eqI==TRUE){
				a[i] = ((deltaQ-dQdc)/nf)*f[0]*PI*Imultiplier;
			}
			actualdQ += a[i]/(f[i]*PI); 
			actualImax += a[i];
			ph[i] = -PI*i*i/nf+PI/2.0;	// compute Schroeder phase, assumes flat spectrum; f[0] set to cos
		}
		Imultiplier *= 1.004;
	}
	sprintf(rbuf,"Current multiplier = %s.", engstr(Imultiplier,4));
	progress(rbuf);
		fmin=1000; fmax=1e-7;
	for(i=0;i<nf;i++){		// write out freq/mag/pha for each tone
		fprintf(logfile,"f[%d]=%s a=%s, ph=%.2lf dQ=%.3lfAh\n",
			i, sengstr(f[i],3), sengstr(a[i],3), 180*ph[i]/PI, a[i]/(f[i]*PI*3600.0) );
		fmin=MIN(fmin,f[i]);
		fmax=MAX(fmax,f[i]);
	}
	period = 1.0/fmin;					// period of multitone "cycle"


	if(skip==FALSE){				// execute actual measurement
		// now open tvi file
		strcpy(logfname,baseName);
		strcat(logfname,".tvi");
		tvi = fopen(logfname,"w+");						// tvi file open 
		if(tvi==NULL) err("Cannot open tvi file");

		// open interface, NOTRANS apparently not supported
		hp = open(USBpath, O_RDWR | O_NOCTTY | O_NONBLOCK); // open port, no hanging
		if(hp<0) {	// O_NOTRANS added in 2.11 to fix port control
			fprintf(stderr,"Error %i from open: %s\n", errno, strerror(errno));
			err("Cannot open the device.");
		}
		progress("Handle opened.");
		if (tcgetattr(hp, &spset) < 0) {
			err("Cannot get port attributes.");
		}
		cfmakeraw(&spset); // added in 2.11 to fix port control
		if (tcsetattr(hp, TCSANOW, &spset) < 0) {	// raw port now
			err("Cannot set port attributes.");
		}


		// set up the prologix for 66332
		msg("Setting up prologix interface for 66332... ");
		initPrologix(hp);							// set up inteface
		sprintf(wbuf,"++addr %d\n", gpibaddr);
		wrtstr(hp,wbuf);								// point to our instr
		progress("Prologix set up.");

		// grab bus, terminate instrument communications, ID instrument
		msg("Checking GPIB bus... ");
		sprintf(wbuf,"++ifc\n");wrtstr(hp,wbuf);		// send INTERFACE CLEAR
		tickle(500);
		sprintf(wbuf,"++clr\n");wrtstr(hp,wbuf);		// send CLEAR to instrument itself
		msg("Waiting while bus clears... ");
		tickle(2500);
		progress("Bus cleared.");

		// initialize HP function
		msg("Check ID of Instrument... ");
		wrtstr(hp,"*IDN?\n");						// ask for IDN
		tickle(300);
		rbuf[0]='\0';								// clear string
		getmsg(hp,rbuf);							// get message from addr
		if(strstr(rbuf,"66332")==NULL){
			fprintf(stderr,"Instrument at %d identifies as:'%s' (%ld chars)",gpibaddr,rbuf,strlen(rbuf));
			err("Bad instrument ID");
		}else{
			for(i=0;i<strlen(rbuf);i++){if(rbuf[i]=='\r'||rbuf[i]=='\n')rbuf[i]='\0';}
			progress(rbuf);
		}
		progress("Instrument ID checked OK; initialising...");
		
		//****************************************************************
		
		sprintf(wbuf,"*RST;\n");wrtstr(hp,wbuf); 		// reset
		tickle(2500);
		sprintf(wbuf,"SENSe:CURRent:RANGe MAX;\n"); 	// 5A range
		wrtstr(hp,wbuf);

		msg("Checking for instrument errors... ");
		j=0;
		do{
			sprintf(wbuf,"SYST:ERR?;\n");wrtstr(hp,wbuf); 	// ask for errors
			tickle(300);
			rbuf[0]='\0';
			getmsg(hp,rbuf);								// Tx
			i=sscanf(rbuf,"%d",&scpi);						// Rx
			sprintf(wbuf,"err check (%d): %s",j++,rbuf);
			progress(wbuf);msg(wbuf);
		}while(i!=1 || scpi!=0);
		
		//********************************************************************


		// now iterate set-read loop until required time has elapsed
		wrtstr(hp,"OUTP ON\n");tickle(500);				// enable outputs
		time(&tmark);									// time in seconds for dwells
		clock_gettime(CLOCK_REALTIME, &ts);				// present into ts(tart) structure
		clock_gettime(CLOCK_REALTIME, &tn);				// present into tn(ow) structure
		lastmeastime = meastime = (tn.tv_sec-ts.tv_sec)+(tn.tv_nsec-ts.tv_nsec)/GIG; // init meastime
		dt=0.00;
		dQ=0.00;
		
		//New code!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
		dQtarget=0.00;	// (v6)
		i_error=0.00;	// (v6)
		ib=0.00;
		vb=0.00;
		//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!	
		
		msg("Commencing main measurement loop... ");
		progress("Commencing main measurement loop... ");
		while(meastime<=period*ncyc+Xcyc*period+dt+1.0){		// not covered discard+window+margin yet

			// TIME: in Raspbian, use clock_gettime()
			clock_gettime(CLOCK_REALTIME, &tn);			// present into tn(ow) structure
			meastime = (tn.tv_sec-ts.tv_sec)+(double)((tn.tv_nsec-ts.tv_nsec))/GIG; // update meastime

			// calculate STIMULUS ***********************************************
			for(Istim=0.0,i=0;i<nf;i++){				// sum Istim over each tone
				Istim += a[i] * sin(2.0*PI*f[i]*meastime+ph[i]);
			}
			// add in squarewave 
			tdc = 1.0/fdc;
			fracycle = fmod(meastime,tdc)/tdc;			// this is the 0<= fraction <1 of a dc current cycle
			if( fracycle>=0.25 && fracycle<0.75 ){		// if in second or third quarter
				midcycle = TRUE;						// Idc switch flag (v6)
				Istim += Idc;							// add dc
			}else{										// otherwise... (1st or 4th)
				midcycle = FALSE;						// Idc switch flag (v6)
				Istim -= Idc;							// subtract dc
			}							// using 2 & 3 ensures dQ is equally + and -
			// allow for sink-only mode
			if(sink){							// ordered to discharge battery
				Istim -= Imax;					// now Istim<0
			}
			// STIMULUS calculated ***********************************************

			Ibiggest = MAX(Ibiggest,Istim);
			Ismallest = MIN(Ismallest,Istim);

			// set required V & I, change heading towards Vmin/Vmax, include trim correction
			sprintf(wbuf,"VOLT %.6lf;CURR %.6lf\n",(Istim+itrim<0.00)?(0.99*Vmin):(1.01*Vmax),fabs(Istim+itrim)); // set V & I
			wrtstr(hp,wbuf);							// send	

			// READOUT V & I
			datvoid=FALSE;						// reset warning, retry measurement
			wrtstr(hp,"MEAS:VOLT?\n");			// request the terminal voltage
			getmsg(hp,rbuf);
			i=sscanf(rbuf,"%lf",&vb);
			wrtstr(hp,"MEAS:CURR?\n");
			getmsg(hp,rbuf);
			i+=sscanf(rbuf,"%lf",&ib);
			if(i!=2)datvoid=TRUE;			// something went wrong
			
			if(datvoid==FALSE && vb>=Vmax){		// hit hi voltage limit!
				wrtstr(hp,"OUTP OFF\n");					// disable outputs
				progress("Hit high voltage limit... aborting run!");
				err("Hit high voltage limit... aborting run!");
			}
			if(datvoid==FALSE && vb<=Vmin){		// hit lo voltage limit!
				wrtstr(hp,"OUTP OFF\n");					// disable outputs
				progress("Hit low voltage limit... aborting run!");
				err("Hit low voltage limit... aborting run!");
			}
			
			// Now check charge excursion in case of DAC error creep
			if(datvoid==FALSE){
				dt=meastime-lastmeastime;
				lastmeastime = meastime;	// deal with time
				dQ += dt*ib;					// accumulate measured delta charge
				if(sink==FALSE && fabs(dQ)>1.01*deltaQ){		// exceeded limit+1%
					err("actual delta_Q exceeded specified limit (by 1 percent)\n");
				}
				dQtarget += dt*Istim;		// expected charge excursion (delta Q) (v6)
				if(sink==FALSE && fabs(dQtarget)>deltaQ){		// should never happen! (v6)
					err("target delta_Q exceeded specified limit (should never happen)!\n");
				}
			}
			dQbiggest = MAX(dQbiggest,dQ);
			dQsmallest = MIN(dQsmallest,dQ);
			
			// when expected dQ crosses zero, find charge discrepancy (v6)
			if( npts>2000 && SIGN(dQtarget)!=SIGN(last_dQtarget) ){	// well into measurement and crossed zero
				last_dQtarget = dQtarget;					// keep current value for future checks
				Qerror = dQtarget-dQ;						// how far out really?
				i_error = Qerror/meastime;					// Q=it so i=Q/t amps apparent error
				sprintf(rbuf,"==dQ target %sAh crossed zero @%d, dQ=%sAh, (%.3lf%%) -> i_error=%s, itrim=%s", 
					sengstr(dQtarget/3600.0,4), npts, sengstr(dQ/3600.0,4), 100*Qerror/deltaQ, engstr(i_error,4), engstr(itrim,3) );
				progress(rbuf);
			}
			if( npts%20000==19999 ){	// well into measurement 
				Qerror = dQtarget-dQ;					// how far out really?
				i_error = Qerror/meastime;				// Q=it so i=Q/t amps apparent error
				sprintf(rbuf,"===dQ target =%sAh @%d, dQ=%sAh, (%.3lf%% of permitted) -> i_error=%s", 
					sengstr(dQtarget/3600.0,4), npts, sengstr(dQ/3600.0,4), 100*Qerror/deltaQ, engstr(i_error,4) );
				progress(rbuf);
			}
			// when squarewave switches sign we may update itrim (v6)
			if(midcycle!=last_midcycle){	// switched
				last_midcycle=midcycle;		// save current value
				itrim = 1.3 * i_error;		// overcorrect to pull back
				errpc = 100.0*itrim/Imax;
				sprintf(rbuf," ++Idc crossed zero, pt%d: updating itrim to %s, %.3lf%% of Imax", 
					npts, engstr(itrim,4), errpc );
				progress(rbuf);
			}
			
			// info display
			fase = (int)(fracycle*4);
			sprintf(rbuf,"pt%d: %.3lfs V=%.3lf, I=%+.3lf,%c dt=%.3lfs dQ=%sAh %c %c %d%% (%.1lfH to go)",
				npts++,meastime,vb,ib,fase+'a',
				dt,sengstr(dQ/3600.0,3),datvoid?'X':'O',meastime<Xcyc*period?'<':'+',
				(int)(100.0*meastime/(period*(Xcyc+ncyc))),
					((period*ncyc+Xcyc*period+dt+1.0)-meastime)/3600.0 );
			msg(rbuf);
			if(npts%1000==17){
				fprintf(logfile,"%s\n",rbuf);
				fflush(logfile);
			}
			if(npts%5000==18){
				fprintf(logfile,"Istim_max= %s, Istim_min= %s dQ_max= %s, dQ_min= %s\n", 
					engstr(Ibiggest,3),engstr(Ismallest,3),engstr(dQbiggest/3600.0,3),engstr(dQsmallest/3600.0,3));
				fflush(logfile);
			}
			if(npts%5000==19){
				fprintf(logfile,"%s\n",rbuf);
				fprintf(logfile,"Istim=%s, itrim=%s (Iset=%s), i_meas=%s \n", 
					sengstr(Istim,5),sengstr(itrim,5),sengstr(Istim+itrim,5),sengstr(ib,5) );
				fflush(logfile);
			}
			if(datvoid){continue;}					// bad data, don't log
			if(meastime<Xcyc*period){ continue; }	// in the discard window, don't log
			fprintf(tvi,"%.3lf %s %s\n", meastime, engstr(vb,6), engstr(ib,6));	// triple to tvi file

		}
		progress("Completed measurement sequence.");
		wrtstr(hp,"OUTP OFF\n");					// disable outputs
		fclose(tvi);
		sprintf(rbuf,"Largest current = %s ", sengstr(Ibiggest,3));
		progress(rbuf);
		sprintf(rbuf,"Smallest current = %s ", sengstr(Ismallest,3));
		progress(rbuf);
	}

	if(getz){	// use dftp to estimate V and I at each frequency, and so z
		for(i=0;i<nf;i++){				// for each frequency
			// call to dft I then V at that frequency >bat
			fprintf(bat,"%s %s.tvi %s 1 0 D L C 3 %c>%s.tmp\n",dftname,baseName,engstr(f[i],3),i?'>':' ',baseName);
			fprintf(bat,"%s %s.tvi %s 1 0 D L  >>%s.tmp\n",dftname,baseName,engstr(f[i],3),baseName);
		}
		fprintf(bat,"%s %s.tvi %s 1 0 D L C 3 >>%s.tmp\n",dftname,baseName,engstr(fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L  >>%s.tmp\n",dftname,baseName,engstr(fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L C 3 >>%s.tmp\n",dftname,baseName,engstr(3.0*fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L  >>%s.tmp\n",dftname,baseName,engstr(3.0*fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L C 3 >>%s.tmp\n",dftname,baseName,engstr(5.0*fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L  >>%s.tmp\n",dftname,baseName,engstr(5.0*fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L C 3 >>%s.tmp\n",dftname,baseName,engstr(7.0*fdc,8),baseName);
		fprintf(bat,"%s %s.tvi %s 1 0 D L  >>%s.tmp\n",dftname,baseName,engstr(7.0*fdc,8),baseName);
		fclose(bat);
		// exec bat 
		sprintf(cmd,"chmod +x %s.bat\n",baseName);
		progress(cmd);
		if(-1==system(cmd)){
			err("Call to make batch file executable failed");}
		sprintf(cmd,"./%s.bat\n",baseName);
		progress(cmd);
		if(-1==system(cmd)){
			err("Call to batch file failed");}
		// read result from tmp, compute z
		progress("Opening .tmp file...");	// tmp file from script
		strcpy(logfname,baseName);
		strcat(logfname,".tmp");
		bat = fopen(logfname,"r");			// reuse batch file pointer
		if(bat==NULL) err("Cannot open tmp file");
		progress(".tmp open.");		
		// read raw data from tmp file and compute fmp
		for(i=0;i<nf;i++){
			fgets(rbuf, 255, bat);
			sscanf(rbuf,"%le %le %le",&fcheck,&imag[i],&ipha[i]);
			if(relerr(fcheck,f[i])>0.01){err("Line in .tmp with wrong frequency!");}
			fgets(rbuf, 255, bat);
			sscanf(rbuf,"%le %le %le",&fcheck,&vmag[i],&vpha[i]);
			if(relerr(fcheck,f[i])>0.01){err("Line pair in .tmp with different frequencies!");}
			// write fmp
			mag=vmag[i]/imag[i];
			pha=vpha[i]-ipha[i];
			fprintf(fmp,"%s %s %.2lf\n",engstr(f[i],6),engstr(mag,4),pha);
		}
		fclose(fmp);
	}

	if(refine){	// use estimates to start optimiser, refine V & I, get refined z (.ffz)
		// ff ipfile.tvi cn Tw Tx <frequencyFile> <guessFile>
		// write current guess
		strcpy(logfname,baseName);
		strcat(logfname,".gs");
		gs = fopen(logfname,"w+");	 
		if(gs==NULL) err("Cannot open .gs file for ff.");
		for(i=0;i<nf;i++){
			fprintf(gs,"%s %s %.2lf\n",engstr(f[i],6),engstr(imag[i],3),ipha[i]);
		} fclose(gs);
		// refine (if Xcyc, use all of ncyc, else dump 0.5 cycles)
		discard = MAX(0,(MIN(0.5,0.5-Xcyc)));
		sprintf(cmd,"%s %s.tvi 3 %.2lf %.2lf %s.frq %s.gs >hold.ffi\n",
			ffname,baseName,ncyc-discard-0.01,discard,baseName,baseName);
		progress(cmd);
		if(-1==system(cmd)){err("Call to FinerFit file failed");}
		// retrieve refined current
		gs = fopen("hold.ffi","r");	 
		if(gs==NULL) err("Cannot open hold.ffi file.");
		i=0;
		while(NULL!=fgets(rbuf, 255, gs)){
			j=sscanf(rbuf,"%le %le %le",&fcheck,&imag[i],&ipha[i]);
			if(j!=3){
				fprintf(stderr,"read spurious hold.ffi line: %s\n",rbuf); 
				continue;
			}
			if(relerr(fcheck,f[i])>0.01){
				progress(rbuf);
				sprintf(rbuf,"Line in hold.ffi with unexpected frequency (%le/%le)",f[i],fcheck);
				progress(rbuf);
				err("ff failed?");
			}
			i+=1;
		} fclose(gs);
		if(i!=nf){sprintf(rbuf,"Bad number of frequency lines in hold.ffi (%d/%d).",i,nf);err(rbuf);}
		
		// write voltage guess
		strcpy(logfname,baseName);
		strcat(logfname,".gsv");
		gs = fopen(logfname,"w+");	 
		if(gs==NULL) err("Cannot open .gsv file for ff.");
		for(i=0;i<nf;i++){
			fprintf(gs,"%s %s %.2lf\n",engstr(f[i],6),engstr(vmag[i],3),vpha[i]);
		} fclose(gs);
		// refine (if Xcyc, use all of ncyc, else dump 0.5 cycles)
		sprintf(cmd,"%s %s.tvi 2 %.2lf %.2lf %s.frq %s.gsv >hold.ffv\n",
			ffname,baseName,ncyc-discard-0.01,discard,baseName,baseName);
		progress(cmd);
		if(-1==system(cmd)){err("Call to FinerFit file failed");}
		// retrieve refined V
		gs = fopen("hold.ffv","r");	 
		if(gs==NULL) err("Cannot open hold.ffv file.");
		i=0;
		while(NULL!=fgets(rbuf, 255, gs)){
			j=sscanf(rbuf,"%le %le %le",&fcheck,&vmag[i],&vpha[i]);
			if(j!=3){fprintf(stderr,"read hold.ffv line: %s\n",rbuf); continue;}
			if(relerr(fcheck,f[i])>0.01){
				progress(rbuf);
				sprintf(rbuf,"Line in hold.ffv with unexpected frequency (%le/%le)",f[i],fcheck);
				progress(rbuf);
				err("ff failed?");
			}
			i++;
		} fclose(gs);
		if(i!=nf){sprintf(rbuf,"Bad number of frequency lines in hold.ffv (%d/%d).",i,nf);err(rbuf);}

		// write refined Z
		for(i=0;i<nf;i++){
			mag=vmag[i]/imag[i];
			pha=vpha[i]-ipha[i];
			fprintf(ffz,"%s %s %.2lf\n",engstr(f[i],6),engstr(mag,4),pha);	
		}
		fclose(ffz);
	}

	time(&tnow);
	fprintf(stderr,"\n");
	sprintf(wbuf,"bzdcp66 done (took %ld secs, %.1f hours).          ",
		tnow-tstart,(tnow-tstart)/3600.00);	// display we are finished
	progress(wbuf);
	msg(wbuf);
	fprintf(stderr,"\n");
	fclose(logfile);
}

