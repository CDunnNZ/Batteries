# awk script to calculate alpha from sinewave efficiency tests 
BEGIN{
    print "This script multiplies V, I and time elapsed, dt, for each row of a .tvi file and keeps two running totals, Ein (energy in) where I>0 and Eout (energy out) when I<0. Total Eout is divided by total Ein at the end to give an estimate of energy efficiency, u. From this we can determine alpha. Enter a name for the output file: "
	getline name < "-" }
{
    tlast = tnow;
    tnow = $1;
    dt = tnow-tlast;
    if(NR>1){
	if($3>=0)
	    Ein = Ein + dt*abs($2)*$3
	else
	    Eout = Eout + dt*abs($2)*abs($3)
	if(Ein!=0)
	    u = Eout/Ein;
	else
	    u=0;
	printf("%f %f\n", $1, u) > name".tu"
    }
}
END{
    u = Eout/Ein;
    printf("\n%s %.5f\n", "For the file you just read in, final u value =", u);
}
function abs(x) {
    return((x < 0.0) ? -x : x)
}
