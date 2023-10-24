#include	<time.h>
#include	<stdio.h>
#include	<stdlib.h>
#include	<math.h>
#include	<ctype.h>


int main(int argc, char *argv[]){
  FILE *fileptr;
  char *sline;
  size_t li;
  double mtime=0.00, voltage, current;
  double lastmtime=0, dt;
  double u, q=0.00, ein=0.00, eout=0.00;
  long int cntr;

  if ( argc != 2) { 
    fprintf(stderr,"tvi2u                  V3.0 CJD & JBS August 2023\n");
    fprintf(stderr,"Usage: tvi2u file.tvi >file.tu\n");
    fprintf(stderr,"Takes in a 3-col ascii file giving time, voltage, current,\n");
    fprintf(stderr,"writes same time steps and device cycle efficiency to stdout.\n");
    exit(1);
  }

  //	time(&t);
  //	printf("%s", ctime(&t));

  fileptr = fopen( argv[1], "rb");
  if(fileptr==NULL){
    fprintf(stderr, "Cannot open .tvi file %s!\n", argv[1]); //check for the .tvi file
    exit(1);
  }


  cntr=0;
  while(getline(&sline, &li, fileptr)>0 && sscanf(sline,"%le %le %le", &mtime, &voltage, &current)==3){
    //while ( fscanf(fileptr, "%le %le %le", &mtime, &voltage, &current) != EOF) { 
    dt=mtime-lastmtime;
    lastmtime=mtime;
    q+= dt*current;
    if(cntr>0){
      if(current>0.00){ein+=dt*current*voltage;}
      else{eout-=dt*current*voltage;}
      if(ein!=0){u=eout/ein;}else{u=0.00;}
      printf("%.3lf %.3lf \n", mtime, u);
    }
    cntr++; 
  }
  fprintf(stderr,"total # lines = %ld; final u = %.3lf\n", cntr, u);

  return(0);
}

