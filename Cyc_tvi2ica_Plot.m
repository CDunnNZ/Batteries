%Script to load tvi files and plot CV/ICA data
clear all
%close all
data = importdata('C:/Users/Owner/Dropbox/Battery_Research/CycEIS_to_CV/GreyNCA_QofC/Gry_5A00.tvi');
time = data(:,1);
volts = data(:,2);
current = data(:,3);

vstep = 0.005;
pts = 1;
V = [];
Q = [];
t = [];
lastvolts = volts(1);
lasttime = time(1);
dQ = 0;
dV = 0;

%%
for c = 1:length(current) %c for count
    dQ = dQ+(current(c)*(time(c)-lasttime)); %Amount of charge moved - coulombs
    dV = volts(c)-lastvolts; %Change in voltage since last timepoint
    if dV<0
        dV = lastvolts-volts(c); %Reverse calculation for negative voltages
    end
    if dV>=vstep %If moved specified voltage step
        V(pts) = volts(c);
        Q(pts) = dQ;
        t(pts) = time(c)-lasttime;
        pts = pts+1;
        lastvolts = volts(c);
        lasttime = time(c);
        dQ = 0;
    end
end
%%
i_cv = Q./t; %Calculate dQ/dt = coulombs/sec = amperes
%%
i_ica = Q./V; %Calculate dQ/dV = coulombs/volt
%%
k = V./t; %Calculate dV/dt = volts/sec
%%
figure() %CV plot
plot(V,i_ica.*k,'.');
xlabel('Cell voltage');
ylabel('Current, \deltaQ/\deltat = k\deltaQ/\deltaV (A)');
grid on;
%%
figure() %ICA plot
i_ica = i_ica/3600; %This is equivalent to I(t)/k (or i_cv./k) converted to Ah/s
plot(V,i_ica,'.');
xlabel('Cell voltage');
ylabel('Incremental capacity, \deltaQ/\deltaV = I(t)/k (Ah/V)');
grid on;
%% Cycling tvi plot
figure();
yyaxis left;
plot(time,current,'--');
grid on
xlabel('Time (s)')
ylabel('Current (A)')

yyaxis right;
plot(time,volts);
ylabel('Voltage');
legend('Current','Voltage');
%%
%figure()
%plot(V,i_cv,'.');