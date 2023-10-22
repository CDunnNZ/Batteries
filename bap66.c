// Program to measure battery response to arbitrary I using HP66332 and Prologix/Fenrir GPIB on Raspberry Pi
// JBS & CJD, Jan 2021

#include    <stdio.h>
#include    <stdlib.h>
#include    <string.h>
#include    <time.h>
#include    <math.h>
#include    <fcntl.h>
#include    <errno.h>
#include    <complex.h>
#include    <termios.h>
#include 	<unistd.h> // write(), read(), close()

#define MAX(A,B) (((A)>(B))?(A):(B))
#define MIN(A,B) (((A)<(B))?(A):(B))
#define TRUE 1
#define FALSE 0
#define GIG 1000000000

FILE *logfile;										// to log errors
double meastime;

#include "prologix.h"

int main(int argc, char* argv[])
{
	FILE *tvi, *ti;
	int hp;
	char USBpath[64];
	char rbuf[256], wbuf[128];
 	time_t tstart,tnow;
	struct timespec ts, tn;
    double lastmeastime, vm, im, tin, iin, Vmax,Vmin, dQ=0.00, dt;
    int gpibaddr=5;
    char baseName[64], fname[128], cinline[256];
    int i,j,narg=0,npts=0,nlines=0,scpi;
    struct termios spset;


	// version 0.99: copy from bzp66 v2.02
	// version 1.00: fixed bug where dQ was not initialised
    float version = 1.50;    
    if (argc<4+1 || argc>5+1) { // ??
        fprintf(stderr,"bap66 V%.2f jbs&cjd Jan 2021, Apr 2023\n", version);
        fprintf(stderr,"Battery arbitrary waveform measurement via Prologix/Fenrir GPIB-USB & 66332A.\n");
        fprintf(stderr,"Usage: bap66 USB Vmin Vmax baseName [Addr]\n");
        fprintf(stderr,"where-  USB is the rPi USB address (/dev/ttyUSB0, /dev/ttyACM0, etc);\n");
	fprintf(stderr,"        Vmin/Vmax are voltage limits (aborts outside this range);\n");
        fprintf(stderr,"        baseName is the file string to be used;\n");
        fprintf(stderr,"        Addr is the optional GPIB bus address, def=%d.\n",gpibaddr);
        fprintf(stderr,"Sources current described by a .ti file, measuring V & I to .tvi file.\n");
        fprintf(stderr,"Creates baseName.tvi, basename.log, reads basename.ti file.\n");
        fprintf(stderr,"Assumes ti file contains seconds-amps pairs (or blank lines).\n");
        fprintf(stderr,"Requires no drivers, communicates using ++cmd protocol.\n");
        fprintf(stderr,"\n");
        exit(1);
    }
    
    // process input arguments
	strcpy(USBpath,argv[++narg]);									// /dev/tty?...
	if(strstr(USBpath,"tty")==NULL) err("Bad USB address?");
	if(strstr(USBpath,"dev")==NULL) err("Bad USB address?");
	
    Vmin = atof(argv[++narg]);
	if(Vmin<0) err("Vmin is too small");
	if(Vmin>15.0) err("Vmin is too large");

    Vmax = atof(argv[++narg]);
	if(Vmax<=Vmin) err("Vmin exceeds/equals Vmax");
	if(Vmax>20.0) err("Vmax is too large");
	
	strcpy(baseName,argv[++narg]);

	if(argc>++narg) gpibaddr = atoi(argv[narg]);
	if(gpibaddr<1 || gpibaddr>30) err("Bad GPIB_Address given.\n");

	// open a log file for problem/progress reports
	strcpy(fname,baseName);
	strcat(fname,".log");
	logfile = fopen(fname,"w+");						// log file open 
	if(logfile==NULL) err("Cannot open log file to write.");
	// start logging
	time(&tstart);										// note the time of start
	sprintf(wbuf,"%s started, logfile opened, at %s",argv[0],ctime(&tstart));
	wbuf[strlen(wbuf)-1]='\0';	// clip off newline
	progress(wbuf);
	for(wbuf[0]='\0',i=0;i<argc;i++){strcat(wbuf,argv[i]);strcat(wbuf," ");}
	progress(wbuf);

	// open the input file
	strcpy(fname,baseName);
	strcat(fname,".ti");
	ti = fopen(fname,"r");			// arbitrary source (ti) file open 
	if(ti==NULL) err("Cannot open ti file to read waveform.");
	progress("ti file open.");

	// open the output file
	strcpy(fname,baseName);
	strcat(fname,".tvi");
	tvi = fopen(fname,"w+");			// tvi file open 
	if(tvi==NULL) err("Cannot open tvi file to write.");
	progress("tvi file open.");

//New bit

	// find interface
	hp = open(USBpath,O_RDWR|O_NONBLOCK);	// open read & write ASCII, without hanging
	if(hp<0) {
		fprintf(stderr,"Error %i from open: %s\n", errno, strerror(errno));
		err("Cannot open the device.");
	}
		
//End of new bit

	progress("Handle opened.");
	if (tcgetattr(hp, &spset) < 0) {
		err("Cannot get port attributes.");
	}
	cfmakeraw(&spset); // added in 1.04 to fix port control
	if (tcsetattr(hp, TCSANOW, &spset) < 0) {	// raw port now
		err("Cannot set port attributes.");
	}

	// set up the prologix for 66332
	msg("Setting up prologix interface for 66332... ");
	initPrologix(hp);							// set up interface
	sprintf(wbuf,"++addr %d\n", gpibaddr);
	wrtstr(hp,wbuf);								// point to our instr
	progress("Prologix set up.");

	// grab bus, terminate instrument communications, ID instrument
	msg("Checking GPIB bus... ");
	sprintf(wbuf,"++ifc\n");wrtstr(hp,wbuf);		// send INTERFACE CLEAR
	tickle(500);
	sprintf(wbuf,"++clr\n");wrtstr(hp,wbuf);		// send CLEAR to instrument itself
	msg("Waiting while bus clears... ");
	tickle(500);
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

	progress("Entering main loop...");
	sprintf(wbuf,"VOLT %.6lf;CURR %.6lf\n",1.00,0.00);	// set V & I to harmless values
	wrtstr(hp,wbuf);tickle(50);							// send	
	wrtstr(hp,"OUTP ON\n");tickle(50);					// enable output
	// TIME: in Raspbian, use clock_gettime()
	clock_gettime(CLOCK_REALTIME, &ts);					// present into ts(tart) structure
	clock_gettime(CLOCK_REALTIME, &tn);					// present into tn(ow) structure
	lastmeastime = meastime = 0.00; 					// init meastime
	dt=0.00;
	// now iterate read-set loop until .ti file ends
	while ( fgets(cinline,254,ti) != NULL ) {		// there is another line
        nlines++;       /* count lines in */
        if(strlen(cinline)<3){continue;}			// too small for time-current value
		if(2!=sscanf(cinline,"%lf %lf",&tin,&iin)){
			fprintf(stderr,"Failed to get 2 floats in line %d (%s)\n",nlines,cinline);
			exit(1);
        }
        // if meastime is ahead of time read in, tin, read in more lines to increase value
        if(meastime>tin){continue;}
        // set & measure until meastime again > tin
        while(meastime<tin){
			// READOUT V & I
			do{
				clock_gettime(CLOCK_REALTIME, &tn);					// present into tn(ow) structure
				meastime = (tn.tv_sec-ts.tv_sec)+(double)((tn.tv_nsec-ts.tv_nsec))/GIG; // meastime
				wrtstr(hp,"MEAS:VOLT?\n");			// request the terminal voltage
				getmsg(hp,rbuf);
				i=sscanf(rbuf,"%lf",&vm);
				wrtstr(hp,"MEAS:CURR?\n");
				getmsg(hp,rbuf);
				i+=sscanf(rbuf,"%lf",&im);
			}while(i!=2);											// something went wrong?
			if(vm>Vmax || vm<Vmin){								// hit a voltage limit!
//				sprintf(wbuf,"VOLT %.6lf;CURR %.6lf\n",1.00,0.00);	// set V & I to harmless values
//				wrtstr(hp,wbuf);tickle(50);							// send	
//				wrtstr(hp,"OUTP OFF\n");tickle(50);					// enable output
				progress("Hit a voltage limit... ");
//				err("Hit a voltage limit... aborting run!");
			}
			dt=meastime-lastmeastime; lastmeastime = meastime;
			dQ+=dt*im;										// accumulate delta charge
			sprintf(rbuf,"pt%d: %.3lfs V=%.3lf, I=%+.3lf; dt=%.3lfs dQ=%sAh ",
					npts++,meastime,vm,im,dt,sengstr(dQ/3600.0,3));
			msg(rbuf);
			if(npts%1000==1){fprintf(logfile,"%s\n",rbuf);}
			fprintf(tvi,"%.3lf %s %s\n", meastime, engstr(vm,6), engstr(im,6));	// triple to tvi file
		}
		sprintf(wbuf,"VOLT %.6lf;CURR %.6lf\n",(iin<0.00)?Vmin-0.001:Vmax+0.001,fabs(iin)); // set V & I
		wrtstr(hp,wbuf);							// send	
	}

	progress("Completed measurement sequence.");
	wrtstr(hp,"OUTP OFF\n");					// disable outputs

	time(&tnow);
	sprintf(wbuf,"bap66 done (took %ld secs, %.1f hours)",
		tnow-tstart,(tnow-tstart)/3600.00);	// display we are finished
	printf("\n%s\n", wbuf); // new line to print info to the terminal
	progress(wbuf);
	fclose(logfile);
	fclose(ti);
	fclose(tvi);
}

