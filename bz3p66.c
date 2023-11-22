// Program to measure battery impedance using HP66332 and Prologix/Fenrir GPIB on Raspberry Pi
// optionally includes frequency/z analysis by system calls to dftp & ff
// JBS Dec 24, 2020

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>
#include    <math.h>
#include    <fcntl.h>
#include    <errno.h>
#include    <complex.h>
#include 	<unistd.h> // write(), read(), close()
#include <termios.h>
//#include <sys/ioctl.h>
//#include <sys/types.h>
//#include <sys/stat.h>


#define GIG 1000000000
#define NFREQS 32
#define PI 3.141592654
#define TWOPI 2*3.141592654
#define MAX(A,B) (((A)>(B))?(A):(B))
#define MIN(A,B) (((A)<(B))?(A):(B))
#define TRUE 1
#define FALSE 0

FILE *logfile;										// to log errors
double elapstime;

#include "prologix.h"

int main(int argc, char* argv[])
{
	FILE *tvi;
	int hp,skip=FALSE;
	char USBpath[64];
	char rbuf[256], wbuf[128];
 	time_t tstart,tnow,tmark;
	struct timespec ts, tn;
    double lastelapstime, lastvb, lastib;
    double Tcyc, mt_time=0.0, p_time=0.0;
    int gpibaddr=5;
    char baseName[64], logfname[128];
    float ncyc,fmin,fmax,Vmin,Vmax,Imax,ftmp,Xcyc;
    double deltaQ,ib,vb,Istim,dQ=0.00,dt,dtmp;
    double freq, f[NFREQS], period;
    double a[NFREQS], ph[NFREQS], iqf;
    double Ibiggest=-100.0, Ismallest=100.0;
    int i,j, nf=0, narg=0, npts=0, datvoid,scpi;
    double mag,pha,fcheck,discard;
    double imag[NFREQS],vmag[NFREQS],ipha[NFREQS],vpha[NFREQS];
    int sink=FALSE,getz=FALSE,refine=FALSE,readFreqs=FALSE;
	struct termios spset;
	double actualImax,Imultiplier;
	int qloops, eqI=FALSE, inpulse=FALSE, npulses=0;
	double Pf, Pw, Ip, tr;
	double dQexpected=0.00, Qerror;
	double itrim, i_error, errpc;


	FILE *fmp, *ffz, *bat, *frq, *gs, *ff, *ptvi;
	char dftname[256],ffname[256], cmd[256];
    complex double cf[NFREQS], vf[NFREQS], z[NFREQS];

	// version 3.00: cloned from bzp66 v 2.16
	// version 3.01: fixed eqI bug
	// version 3.10: fixed dftv calls with L option on current
	// version 6.00: DAC resolution bug fix; accumulate read-back error, correct Istim 
	// version 6.01: gain reduced to 1.1 for itrim 
	// version 6.03: fixing bug that crashes program around trim code
	// version 6.04: initialise dQ... duh.
    float version = 6.04; 
    if (argc<14+1 || argc>17+1) { // ??
        fprintf(stderr,"bz3p66 V%.2f jbs&cjd, Dec 2020 -> Oct 2021\n", version);
        fprintf(stderr,"Battery Z measurement with triphasic pulses via Prologix/Fenrir GPIB-USB & 66332A.\n");
        fprintf(stderr,"Usage: bz3p66 USB Vmin Vmax Imax dQmax ncyc fmin fmax Xcyc Pf Pw Ip tr baseName [Addr [dftp [ff]]]\n");
        fprintf(stderr,"where-  USB is the rPi USB address (/dev/ttyUSB0, /dev/ttyACM0, etc);\n");
		fprintf(stderr,"        Vmin/Vmax are voltage limits (aborts outside this range);\n");
        fprintf(stderr,"        Imax is the maximum permitted current (-value => only sinks I);\n");
        fprintf(stderr,"        dQmax is the total charge in Ah that can be sourced or sunk (-val => equal I tones);\n");
        fprintf(stderr,"        ncyc is the # cycles at fmin (typically 2.01-6.00);\n");
        fprintf(stderr,"        fmin/fmax are the lowest and highest freqs;\n");
        fprintf(stderr,"        Xcyc is the # cycles at fmin of data to discard before logging.\n"); 
        fprintf(stderr,"        Pf is the frequency of pulse occurences in multitone time, =1/Ttp seconds;\n");
        fprintf(stderr,"        Pw is the period of the triphasic pulse, in seconds;\n");
        fprintf(stderr,"        Ip is the peak current of the triphasic pulses;\n");
        fprintf(stderr,"        tr is the rest period after the triphasic pulse before resuming multitone;\n");
        fprintf(stderr,"        baseName is the file string to be used;\n");
        fprintf(stderr,"        Addr is the optional GPIB bus address, def=%d.\n",gpibaddr);
        fprintf(stderr,"        dftp is the [path]name of Scott/Farrow dft program (dftp,dvtv,etc);\n");
        fprintf(stderr,"        ff is the [path]name of the Scott/Finer multitone optimiser program.\n");
        fprintf(stderr,"Makes a multitone tvi/Z measurement by sourcing current, measuring V & I.\n");
        fprintf(stderr,"If the USB parameter is set to \"skip\" the tvi measurement is skipped.\n");
        fprintf(stderr,"Creates baseName.tvi, basename.log, [.bat, .fmp, [.ffz]] files.\n");
        fprintf(stderr,"Z optionally computed by calls to dftp [& ff] at each frequency.\n");
        fprintf(stderr,".bat file is dft script, fmp has dft's z values, ffz is refined fmp.\n");
        fprintf(stderr,"Frequencies are a 1-2-5 sequence between fmin and fmax;\n");
        fprintf(stderr,"if fmax<0 frequencies are read from baseName.frq file, up to %d freqs.\n",NFREQS);
        fprintf(stderr,"Requires no drivers, communicates using ++cmd protocol.\n");
        fprintf(stderr,"Writes complete data, including pulses, to basename.ptvi file.\n");
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
    	deltaQ=-deltaQ;
    	eqI=TRUE;
    }
	if(deltaQ<0.001) err("deltaQ is less than 1mAh");
	if(deltaQ>30) err("deltaQ is more than 30Ah");
	deltaQ = deltaQ*3600.0;									// convert to Amp-seconds

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
	
	// Pf Pw Ip tr
    Pf = atof(argv[++narg]);
	if(Pf<1e-6) err("Pf must be >=1uHz.");
	if(Pf>0.1) err("Pf must be <=0.1Hz.");
	
    Pw = atof(argv[++narg]);
	if(Pw<2) err("Pulse width must be >=2s.");
	if(Pw>1000) err("Pulse width must be <=1000s.");
	
    Ip = atof(argv[++narg]);
	if(Ip<2e-3) err("Pulse I must be >=2mA.");
	if(Ip>5.01) err("Pulse I must be <=5A.");
	
    tr = atof(argv[++narg]);
	if(tr<0.0) err("Recovery time must be >=0s.");
	if(tr>1000.01) err("Recovery time must be <1000s");
		
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
		progress(".fmp file open.\n");
		strcpy(logfname,baseName); strcat(logfname,".bat");
		bat = fopen(logfname,"w+");			// batch file open 
		if(bat==NULL) err("Cannot open bat file");
		progress(".bat file open.\n");
	}

	if(argc>++narg) {						// this param means we do ff
		refine=TRUE;
		strcpy(ffname,argv[narg]);			// ff program name to call
		progress("Opening .ffz file...");	// for refined fmp results
		strcpy(logfname,baseName); strcat(logfname,".ffz");
		ffz = fopen(logfname,"w+");			// ffz file open 
		if(ffz==NULL) err("Cannot open ffz file");
		progress(".ffz open.");
	}
	
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
	
	if(skip==FALSE){				// execute actual measurement
		// now open tvi file
		strcpy(logfname,baseName);
		strcat(logfname,".tvi");
		tvi = fopen(logfname,"w+");						// tvi file open 
		if(tvi==NULL) err("Cannot open tvi file");
		strcpy(logfname,baseName);
		strcat(logfname,".ptvi");
		ptvi = fopen(logfname,"w+");						// ptvi file open 
		if(ptvi==NULL) err("Cannot open .ptvi file");

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

		// compute amplitudes & phases
		msg("Finding mag & phases of tones... ");
		actualImax=0.00;
		Imultiplier=1.00;
		qloops=0;
		while(++qloops<3000 && actualImax<Imax){		// push up I, if 3k pushes, give up
			for(actualImax=0.00,i=0;i<nf;i++){
				iqf=(deltaQ/nf)*f[i]*PI;				// scrunch Q value at this freq
				a[i]=MIN(Imax*Imultiplier/nf,iqf);		// I is (1/nf) of max or scrunched value
				if(eqI==TRUE){
					a[i] = (deltaQ/nf)*f[0]*PI;
				}
				actualImax+=a[i];
				ph[i] = -PI*i*i/nf;	// compute Schroeder phase, assumes flat spectrum
			}
			Imultiplier *= 1.004;
		}
		sprintf(rbuf,"Current multiplier = %s.", engstr(Imultiplier,4));
		progress(rbuf);
		if(iqf==TRUE){progress("Equal-I flag set");}

		fmin=1000; fmax=1e-7;
		for(i=0;i<nf;i++){		// write out freq/mag/pha for each tone
			fprintf(logfile,"f[%d]=%s a=%s, ph=%.2lf\n",i,sengstr(f[i],3),sengstr(a[i],3),180*ph[i]/PI);
			fmin=MIN(fmin,f[i]);
			fmax=MAX(fmax,f[i]);
		}
		period = 1.0/fmin;					// period of multitone "cycle"

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
		sprintf(wbuf,"*RST;\n");wrtstr(hp,wbuf); 		// reset
		tickle(2500);
		if(Imax>0.02 || (sink && Imax>10e-3)){									// big currents
			sprintf(wbuf,"SENSe:CURRent:RANGe MAX;\n"); 	// 5A range
		}else{
			sprintf(wbuf,"SENSe:CURRent:RANGe MIN;\n"); 	// 20mA range
		}wrtstr(hp,wbuf);

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


		// now iterate set-read loop until required time has elapsed
		wrtstr(hp,"OUTP ON\n");tickle(500);				// enable outputs
		time(&tmark);									// time in seconds for dwells
		clock_gettime(CLOCK_REALTIME, &ts);				// present into ts(tart) structure
		clock_gettime(CLOCK_REALTIME, &tn);				// present into tn(ow) structure
		lastelapstime = elapstime = (tn.tv_sec-ts.tv_sec)+(tn.tv_nsec-ts.tv_nsec)/GIG; // init time
		dt=0.00;
		Tcyc = 1/Pf + Pw + tr;
		msg("Commencing main measurement loop... ");
		progress("Commencing main measurement loop... ");
		while(mt_time<=period*ncyc+Xcyc*period+dt+1.0){		// not covered discard+window+margin yet

			// there is elapstime = tnow-tstart, all the time spent making the measurement
			// the period of a cycle of pulse & multitone, Tcyc = 1/Pf + Pw + tr; 
			// we are in a pulse when time%Tcyc < Pw;
			// mt_time is the time spent delivering the multitone (excludes time in the pulse)
			// TIME: in Raspbian, use clock_gettime()
			clock_gettime(CLOCK_REALTIME, &tn);			// present into tn(ow) structure
			elapstime = (tn.tv_sec-ts.tv_sec)+(double)((tn.tv_nsec-ts.tv_nsec))/GIG; // set elapsed time
			
			if(fmod(elapstime,Tcyc) < (Pw+tr)){ 			// in pulse
				if(inpulse==FALSE){npulses++;}				// count triphasic pulses
				inpulse = TRUE;
			}else{
				inpulse = FALSE;
			}

			// STIMULUS
			if(inpulse){
				itrim = 1.2*i_error;			// update in pulse part
				p_time = fmod(elapstime,Tcyc);
				Istim=0.00;
				if(p_time < Pw){
					Istim = Ip;
				}
				if(p_time < 5.0*Pw/6.0){
					Istim = -Ip;
				}
				if(p_time < Pw/3.0){
					Istim = Ip;
				}
			}else{										// NOT in pulse, so in mt_time
				mt_time = elapstime - npulses*(Pw+tr);		// time spent in multitone parts
				for(Istim=0.0,i=0;i<nf;i++){				// sum Istim over each tone
					Istim += a[i] * sin(2.0*PI*f[i]*mt_time+ph[i]);
				}
				if(sink){Istim -= Imax;}
				Ibiggest = MAX(Ibiggest,Istim);
				Ismallest = MIN(Ismallest,Istim);
			}

			// set required V & I, change heading towards Vmin/Vmax
			sprintf(wbuf,"VOLT %.6lf;CURR %.6lf\n",						// set V & I
				(Istim<0.00)?(0.99*Vmin):(1.01*Vmax),fabs(Istim+itrim));// add in itrim to correct Q drift
			wrtstr(hp,wbuf);											// send	

			// READOUT V & I
			datvoid=FALSE;						// reset warning, retry measurement
			wrtstr(hp,"MEAS:VOLT?\n");			// request the terminal voltage
			getmsg(hp,rbuf);
			i=sscanf(rbuf,"%lf",&vb);
			wrtstr(hp,"MEAS:CURR?\n");
			getmsg(hp,rbuf);
			i+=sscanf(rbuf,"%lf",&ib);
			if(i!=2)datvoid=TRUE;			// something went wrong, did not get 2 numbers
			
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

			if(!datvoid){
				dt=elapstime-lastelapstime;
				lastelapstime = elapstime;		// deal with time
				dQ+=dt*ib;						// accumulate delta charge
				dQexpected += (Istim)*dt;		// expected delta charge
				Qerror = dQexpected-dQ;			// charge leaked
				i_error = Qerror/elapstime;		// Q=it so i=Q/t amps apparent error
				if(dQexpected+dQ){errpc = 200.0*Qerror/(dQexpected+dQ);}else{errpc=0.0;}
				if(npts%1000==3){				// periodically...
					sprintf(rbuf,"--dQ target=%s, actual dQ=%s, (%.2lf%%) -> i_error=%s, itrim=%s", 
						engstr(dQexpected,4), engstr(dQ,4), errpc, 
							engstr(i_error,4), engstr(itrim,4) );
					progress(rbuf);
				}
			}
			sprintf(rbuf,"pt%d: %.3lfs V=%.3lf, I=%+.3lf; dt=%.3lfs dQ=%sAh %c %c %c %d%% (%.1lfH to go)",
				npts++,elapstime,vb,ib,dt,sengstr(dQ/3600.0,3),datvoid?'X':'O',mt_time<Xcyc*period?'<':'+',
				inpulse?'P':'M',
				(int)(100.0*mt_time/(period*(Xcyc+ncyc))),
					((period*ncyc+Xcyc*period+1.0)*(Tcyc*Pf)-elapstime)/3600.0 );
			msg(rbuf);
			if(npts%1000==1){fprintf(logfile,"%s\n",rbuf);}
			if(datvoid){continue;}					// bad data, don't log
			if(inpulse==FALSE && (mt_time>(Xcyc*period))){	// do not log measurements to the tvi file if in the pulse!
				fprintf(tvi,"%.3lf %s %s\n", mt_time, engstr(vb,6), engstr(ib,6));	// triple to tvi file
			}
			fprintf(ptvi,"%.3lf %s %s\n", elapstime, engstr(vb,6), engstr(ib,6));	// triple to complete data file

		}
		progress("Completed measurement sequence.");
		wrtstr(hp,"OUTP OFF\n");					// disable outputs
		fclose(tvi);
		fclose(ptvi);
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
	sprintf(wbuf,"bzp66 done (took %ld secs, %.1f hours).\n",
		tnow-tstart,(tnow-tstart)/3600.00);	// display we are finished
	progress(wbuf);
	fclose(logfile);
}

