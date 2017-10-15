clear all
clc

Cx = .35; %[-]
S = 1.82; %[m^2]
m = 970; %[kg]
Vcil = 1040; %[cm^3]
Tf = .281; %[-]
wmin = 850; %[rpm]
efficiencyt = .9; %[-]
l = 2.3; %[m]
a = 1.15; %[m]
hG = .5; %[m]
wheel_indication = '155/65R13';
R0 = 13*25.4/2 + .65*155; %[mm]
N = 5; %[-]
ts = 1; %[s]
Je = .08; %[kg.m^2]
Jw = 1.6; %[kg.m^2]
g = 9.81; % [m/s^2]
ro = 1.3; %[kg/m^2]
alpha = atan(0:0.1:0.4); %[-]
f0 = .013; %[-]
K = 6.51*10^-6; %[s^2/m^2] 

vKm = 0:10:300; %[km/h]

vm = (vKm./3.6); %[m/s]

% Part 1
A = zeros(5, 1);
B = zeros(5, 1);
R = zeros(5, 31);
P = zeros(5, 31);
Pnorm = zeros(5, 31);
Pcar = zeros(5, 1);
Vcar = zeros(5, 1);
Av = zeros(5, 31);
Bv3 = zeros(5, 31);

for i = 1:5
    A(i) = m*g*(f0*cos(alpha(i)*pi) + sin(alpha(i))); %[N]
    B(i) = m*g*K*cos(alpha(i)) + 0.5*ro*S*Cx; %[kg/m]

    Vcar(i) = sqrt(A(1)/B(1)); %[m/s]
    Pcar(i) = 2*A(i)*sqrt(A(i)/B(i)); %[W]
        
    for j = 1:31
        R(i, j) = A(i) + B(i)*(vm(j).^2); %[N]
        Av(i, j) = A(i)*vm(j); %[W]
        Bv3(i, j) = B(i)*vm(j).^3; %[W]
        P(i, j) = (Av(i, j) + Bv3(i, j)); %[W]
        Pnorm(i, j) = P(i, j)/Pcar(i); %[-]
    end

    Vnorm = vm*10^-3/Vcar(i); %[-]
    
end

figure; hold on;

for i = 1:2
    subplot(3, 3, i);
    plot(vKm, R(i, 1:31), 'g');
    xlabel('Velocity (km/h)');
    ylabel('Resistance (N)');
    title(['Resistance vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

for i = 1:2
    subplot(3, 3, i+2);
    plot(log(vKm), log(P(i, 1:31)), log(vKm), log(Av(i, 1:31)), log(vKm), log(Bv3(i, 1:31)));
    xlabel('log(V)');
    ylabel('log(P)');
    title(['Power vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

for i = 1:5
    subplot(3, 3, i+4);
    plot(log(Vnorm), log(Pnorm(i, 1:31)));
    xlabel('log(Vnorm)');
    ylabel('log(Pnorm)');
    title(['Pnorm vs Vnorm @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

% Part 2
werpm = [1000, 1500, 2000, 2500, 3000, 3250, 3500, 4000, 4500, 5000, 5500, 6000, 6500]; %[rpm]
we = (2*pi/60).*werpm; %[rad/s]
mep = [5.985, 8.5785, 8.841, 9.3345, 9.513, 9.6705, 9.6075, 9.6285, 9.4815, 8.9250, 8.2530, 7.791, 7.119]; %[bar]
%Using a for stroke engine
k = 0.5;
Pe = zeros(13, 1);
Me = zeros(13, 1);
for i = 1:13
   Pe(i) =  k*mep(i)*Vcil*we(i)/(2*pi); % not sure
   Me(i) = Pe(i)/(we(i)); % not sure
end
figure; hold on;
subplot(2, 2, 1);
plot(werpm, Pe);
xlabel('We (rpm)');
ylabel('Pe (W)');
title('Power vs RPM');
subplot(2, 2, 2);
plot(werpm, Me);
xlabel('We (rpm)');
ylabel('Me (Nm)');
title('Torque vs RPM');

[Pmax, imax] = max(Pe); % not sure
wemax = werpm(imax); % [rpm]
Pi = zeros(3, 1);
Peapp = zeros(13, 1);

for i = 1:3
    Pi(i) = Pmax/(wemax^i); % not sure
end

Pi(3) = -Pi(3);

for i= 1:13
    for j = 1:3
       Peapp(i) = Peapp(i) + Pi(j)*werpm(i)^j; % not sure
    end
end

subplot(2, 2, 3);
plot(we, Peapp);
xlabel('We (rpm)');
ylabel('Pe (W)');
title('Approximated Power vs RPM');

Pa = zeros(13, 1);
for i = 1:13
   Pa(i) = efficiencyt*Pe(i); % not sure
end
subplot(2, 2, 4);
plot(we, Pa);
xlabel('We (rpm)');
ylabel('Pa (W)');
title('Power Available at the Wheels vs RPM');

% Part 3
Astar = power(max(Pa)/2/B(1), 1/3); % not sure
Bstar = sqrt(4*(A(1)^3)/27/max(Pa)/B(1)); % not sure
Vmax = Astar*(power(Bstar+1, 1/3)-power(Bstar-1, 1/3)); % not sure
Re = .98*R0; % [mm]
alphamax = atan(.33); %[-]
Tgtop = Vmax/(Tf*Re*wemax*2*pi/60); %[-]
Tgbottom = Me(1)*efficiencyt/(Tf*Re*m*g*(f0*cos(alphamax)+sin(alphamax))); %[-]
Tgi = zeros(5, 1);
Tgi(1) = Tgbottom;
Tgi(5) = Tgtop;
for i = 2:4
   Tgi(i) = Tgi(i-1)*power((Tgtop/Tgbottom), 1/(N-1)); %[-] 
end

figure; hold on;
Vw = zeros(5, 13);
for i = 1:5
   for j = 1:13
       Vw(i, j) = we(j)*Tf*Tgi(i)*Re*10^-3; % [m/s]
   end
end

for i = 1:5
   subplot(2, 3, i); 
   plot(Vw(i, 1:13), Pa(1:13, 1));
end