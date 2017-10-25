clear all
clc

% Define Given Constants
Cx = .35; %[-]
S = 1.82; %[m^2]
m = 970; %[kg]
Vcil = 1040; %[cm^3]
Tf = .281; %[-]
wmin = 850; %[rpm]
efficiencyt = .9; %[-]
l = 2.3; %[m]
a = 1.15; %[m]
b=l-a;
hG = .5; %[m]
wheel_indication = '155/65R13';
R0 = 13*25.4/2 + .65*155; %[mm]
Re = .98*R0*10^-3; % [m]
N = 5; %[-]
ts = 1; %[s]
Je = .08; %[kg.m^2]
Jw = 1.6; %[kg.m^2]
Jt = Jw; %[kg.m^2]
g = 9.81; % [m/s^2]
ro = 1.3; %[kg/m^2]
alpha = atan(0:0.1:0.4); %[-]
f0 = .013; %[-]
K = 6.51*10^-6; %[s^2/m^2] 

% POWER NEEDED FOR MOTION

% Take Artbitrary Velocity Interval
vKm = 0:10:300; %[km/h]
vm = (vKm./3.6); %[m/s]

% Declare All Vectors/Matrices As Zeros
A = zeros(5, 1);
B = zeros(5, 1);
R = zeros(5, 31);
Pn = zeros(5, 31);
Pnorm = zeros(5, 31);
Pcar = zeros(5, 1);
Vcar = zeros(5, 1);
Av = zeros(5, 31);
Bv3 = zeros(5, 31);

% Calculate A, B, Vcar, Pcar, R, Av, Bv^3, Pn, Pnorm and Vnorm at 5
% different inclinations
for i = 1:5
    A(i) = m*g*(f0*cos(alpha(i)) + sin(alpha(i))); %[N]
    B(i) = m*g*K*cos(alpha(i)) + 0.5*ro*S*Cx; %[kg/m]

    Vcar(i) = sqrt(A(1)/B(1)); %[m/s]
    Pcar(i) = 2*A(i)*sqrt(A(i)/B(i)); %[W]
        
    for j = 1:31
        R(i, j) = A(i) + B(i)*(vm(j)^2); %[N]
        Av(i, j) = A(i)*vm(j); %[W]
        Bv3(i, j) = B(i)*vm(j)^3; %[W]
        Pn(i, j) = Av(i, j) + Bv3(i, j); %[W]
        Pnorm(i, j) = Pn(i, j)/Pcar(i); %[-]
    end

    Vnorm = vm/Vcar(i); %[-]
    
end

% Plot The Required Graphs In Part I
% figure; hold on;

for i = 1:2
    subplot(3, 3, i);
    plot(vKm, R(i, 1:31), 'g');
    xlabel('Velocity [km/h]');
    ylabel('Resistance [N]');
    title(['Resistance vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
    grid on;
end

for i = 1:2
    subplot(3, 3, i+2);
    loglog(vKm, Pn(i, 1:31)); hold on;
    loglog(vKm, Av(i, 1:31));
    loglog(vKm, Bv3(i, 1:31));
    xlabel('log(V)');
    ylabel('log(Pn)');
    title(['Power vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
    grid on;
end

for i = 1:5
    subplot(3, 3, i+4);
    loglog(Vnorm, Pnorm(i, 1:31));
    xlabel('log(Vnorm)');
    ylabel('log(Pnorm)');
    title(['Pnorm vs Vnorm @ tan(alpha) = ', num2str(tan(alpha(i)))]);
    grid on;
end

% COMPUTATION OF THE MAXIMUM POWER AVAILABLE AT THE WHEELS

% Define RPM and MEP As Given In Engine Map
werpm = [1000, 1500, 2000, 2500, 3000, 3250, 3500, 4000, 4500, 5000, 5500, 6000, 6500]; %[rpm]
we = (2*pi/60).*werpm; %[rad/s]
mep = [5.985, 8.5785, 8.841, 9.3345, 9.513, 9.6705, 9.6075, 9.6285, 9.4815, 8.9250, 8.2530, 7.791, 7.119]; %[bar]

% Using a four stroke engine
k = .5;

% Declare Pe And Me Vectors As Zeros
Pe = zeros(13, 1);
Me = zeros(13, 1);

% Calculate Pe And Me At Different Engine Rotational Speed
for i = 1:13
   Pe(i) =  .1*k*mep(i)*Vcil*we(i)/(2*pi); % [W]
   Me(i) = Pe(i)/we(i); % [Nm]
end

% Plot The Required Graphs In Part II
figure; hold on;
subplot(2, 2, 1);
plot(werpm, Pe);
xlabel('We [rpm]');
ylabel('Pe [W]');
title('Power vs RPM');
grid on;
subplot(2, 2, 2);
plot(werpm, Me);
xlabel('We [rpm]');
ylabel('Me [Nm]');
title('Torque vs RPM');
grid on;

% Obtain Maximum Power And Corresponding We
[Pmax, imax] = max(Pe); % [W]
wemax = we(imax); % [rad/s]

% Declare Summation Terms And Approximated Power As Zeros Vectors
Pi = zeros(3, 1);
Peapp = zeros(13, 1);

% Calculate The Terms Of The Summation
for i = 1:3
    Pi(i) = Pmax/(wemax^i);
end
Pi(3) = -Pi(3);

% Calculate The Approximated Value Of The Power
for i= 1:13
    for j = 1:3
       Peapp(i) = Peapp(i) + Pi(j)*we(i)^j; % [W]
    end
end

% Plot The Graph Of The Approximated Value
subplot(2, 2, 3);
plot(werpm, Peapp);
xlabel('We [rpm]');
ylabel('Pe [W]');
title('Approximated Power vs RPM');
grid on;

% Calculate The Power Available At The Wheels
Pa = zeros(13, 1);
Paapp = zeros(13, 1);
for i = 1:13
   Pa(i) = efficiencyt*Pe(i); % [W]
   Paapp(i) = efficiencyt*Peapp(i); % [W]
end

% Plot The Graph Of Power Available At The Wheels
subplot(2, 2, 4);
plot(werpm, Pa);
xlabel('We [rpm]');
ylabel('Pa [W]');
title('Power Available at the Wheels vs RPM');
grid on;

% GRADEABILITY AND INITIAL CHOICE OF THE TRANSMISSION RATIOS

% Calculate Maximum Speed Vehicle Can Reach
Pamax = max(Pa);
Astar = power(Pamax/(2*B(1)), 1/3);
Bstar = sqrt(1+(4*A(1)^3/(27*(Pamax^2)*B(1))));
Vmax = Astar*(power(Bstar+1, 1/3)-power(Bstar-1, 1/3)); % [m/s]

% Calculate Gear Ratios
Tgtop = Vmax/(Tf*Re*wemax); %[-]
alphamax = atan(.33); %[-]
Tgbottom = Me(1)*efficiencyt/(Tf*Re*m*g*(f0*cos(alphamax)+sin(alphamax))); %[-]
Tgi = zeros(5, 1);
Tgi(1) = Tgbottom;
Tgi(5) = Tgtop;
for i = 2:4
   Tgi(i) = Tgi(i-1)*power((Tgtop/Tgbottom), 1/(N-1)); %[-] 
end

% Calculate Vehicle Speed According To We And Gear Ratios
Vw = zeros(5, 13);
for i = 1:5
   for j = 1:13
       Vw(i, j) = we(j)*Tf*Tgi(i)*Re; % [m/s]
   end
end

Avw = zeros(5, 13, 2);
Bvw3 = zeros(5, 13, 2);

for i = 1:5
    for j = 1:13
        Avw(i, j, 1) = A(1)*Vw(i, j);
        Bvw3(i, j, 1) = B(1)*Vw(i, j)^3;
    end
end

Aw = m*g*(f0*cos(alphamax) + sin(alphamax)); %[N]
Bw = m*g*K*cos(alphamax) + .5*ro*S*Cx; %[kg/m]

for i = 1:5
    for j = 1:13
        Avw(i, j, 2) = Aw*Vw(i, j);
        Bvw3(i, j, 2) = Bw*Vw(i, j)^3;
    end
end

Pnw = zeros(5, 13, 2);
for i = 1:2
   for j = 1:5
       for k = 1:13
            Pnw(j, k, i) = Avw(j, k, i) + Bvw3(j, k, i); 
       end
   end
end

figure;  hold on;
for i = 1:5
    plot(Vw(i, 1:13), Pa, Vw(i, 1:13), Pnw(i, 1:13, 1), Vw(i, 1:13), Pnw(i, 1:13, 2));
end
xlabel('V [m/s]');
ylabel('Pa | Pn [W]');
title('Power Available & Power Needed (null and max slope) @ Different Gear Ratios');

grid on;

% % MAXIMUM POWER THAT CAN BE TRANSFERRED BY THE TIRES TO THE GROUND

% Define Constants
c1 = [1.1, .8];
c2 = [6, 8]*10^-3;

% Calculate Dx
Rl = 0.92*R0*10^-3;
Dx = Rl.*(f0+K.*vm.^2);

% Calculate Fz1 and Fz2
K1 = ro*S*Cx*hG/2/m/g;
K2 = -K1;
Fz1 = m*g*(b-Dx-K1.*vm.^2)/l;
Fz2 = m*g*(a+Dx-K2.*vm.^2)/l;

% Calculate Maximum Power Transferred in Dry/Wet Conditions
uip(1,1:31) = c1(1) - c2(1).*vm;
uip(2,1:31) = c1(2) - c2(2).*vm;
PmaxWG(1,1:31) = (Fz1).*uip(1,1:31).*vm;
PmaxWG(2,1:31) = (Fz1).*uip(2,1:31).*vm;

% Plot Required Graphs
figure;
plot(vm, PmaxWG(1, 1:31), vm, PmaxWG(2, 1:31), vm, Pn(1, 1:31));
title('Maximum Power Transferred Between Wheel and Ground vs Velocity');
xlabel('V [m/s]');
ylabel('P [W]');

% Obtain The Maximum Velocity
interD = InterX([vm;PmaxWG(1, 1:31)],[vm;Pn]);
interW = InterX([vm;PmaxWG(2, 1:31)],[vm;Pn]);
VmaxWG(1,1) = interD(1,2);
VmaxWG(1,2) = interW(1,2);

% ACCELERATION PERFORMANCE

% Compute Equivalent Mass
me = m + Jt/(Re*Tf)^2 + Je/(Re*Tf)^2./Tgi.^2;

% Compute Maximum Acceleration
amax = zeros(5, 13);
for i = 1:5
   for j = 1:13
      amax(i, j) = (Pa(j)-Pnw(i,j))/(me(i)*Vw(i,j));
   end
end

% In this case we realise that at 5th gear ratio the maximum velocity is
% attained at the index = 12; therefore, theoratically we get a negative
% acceleration which we must substitute by 0 since the acceleration goes to
% 0 (in ideal case) when a body reached its maximum speed
amax(5,13) = 0;

% Plot Maximum Acceleration
oneOamax = 1./amax;
figure; hold on;

for i = 1:5
   plot(Vw(i, 1:13), amax(i, 1:13));
end
title('Maximum Acceleration vs Veloctiy');
xlabel('V [m/s]');
ylabel('A [m/s^2]');
grid on;

% Find Points of Intersections
x = [Vw(1, 13) Vw(2,13) Vw(3,13), 100/3.6];
y = [0 4];
F0 = [Vw(1,1);oneOamax(1,1)];
F1 = InterX([Vw(1,1:13);oneOamax(1,1:13)],[Vw(2,1:13);oneOamax(2,1:13)]);
F2 = InterX([Vw(3,1:13);oneOamax(3,1:13)],[[x(2) x(2)];[y(1) y(2)]]);
F3 = InterX([Vw(4,1:13);oneOamax(4,1:13)],[[x(3) x(3)];[y(1) y(2)]]);
F4 = InterX([Vw(4,1:13);oneOamax(4,1:13)],[[x(4) x(4)];[y(1) y(2)]]);
F = [F1 F2 F3 F4];

% Find Increments for Different Intervals
I = [F1(1)-F0(1), F2(1)-F1(1), F3(1)-F2(1), F4(1)-F3(1)]/10;

% Calculate New Ideal Velocity Vector
newV = zeros(1, 40);
base = Vw(1,1);
for i = 1:4
   for j = (10*i-9):10*i
      newV(j) =  base + I(i)*(j-10*(i-1));
   end
   base = newV(j);
end

% Calculate New Actual Velocity Vector

newVactual = zeros(1, 43);
for i = 43:-1:35
   newVactual(i) = newV(i-3);
end
newVactual(34) = newV(30);
newVactual(33) = newV(30);
for i = 32:-1:24
   newVactual(i) = newV(i-2); 
end
newVactual(23) = newV(20);
newVactual(22) = newV(20);
for i=21:-1:13
   newVactual(i) = newV(i-1); 
end
newVactual(12) = newV(10);
newVactual(11) = newV(10);
for i = 10:-1:1
   newVactual(i) = newV(i);
end

% Calculate New 1/Acceleration Vector
newA = zeros(1, 40);
for i = 1:4
    for j = (10*i-9):10*i
        POINT = InterX([Vw(i,1:13);oneOamax(i,1:13)],[[newV(j) newV(j)];[y(1) y(2)]]);
        newA(j) = POINT(2);
    end
end

% Calculate Ideal Time Using Numerical Integration
Time = zeros(1, 40);
Time(2) = trapz([Vw(1,1) newV(1)],[oneOamax(1,1), newA(1)]);
for i = 3:40
    if (i==22 || i==32)
        index = (i - rem(i,10))/10;
        Time(i) = Time(i-1) + trapz([newV(i-2) newV(i-1)],[F(2, index) newA(i-1)]);
    else
        Time(i) = Time(i-1) + trapz([newV(i-2) newV(i-1)],[newA(i-2) newA(i-1)]);
    end
end

% Calculate Actual Time Using Numerical Integration
TimeActual = zeros(1,43);
TimeActual(2) = trapz([Vw(1,1) newV(1)],[oneOamax(1,1), newA(1)]);
for i = 3:11
    TimeActual(i) = TimeActual(i-1) + trapz([newV(i-2) newV(i-1)],[newA(i-2) newA(i-1)]);
end
TimeActual(12) = TimeActual(11) + ts;
for i = 13:22
    TimeActual(i) = TimeActual(i-1) + trapz([newV(i-3) newV(i-2)],[newA(i-3) newA(i-2)]);
end
TimeActual(23) = TimeActual(22) + ts;
TimeActual(24) = TimeActual(23) + trapz([newV(20) newV(21)],[F(2, 2) newA(21)]);
for i = 25:33
    TimeActual(i) = TimeActual(i-1) + trapz([newV(i-4) newV(i-3)],[newA(i-4) newA(i-3)]);
end
TimeActual(34) = TimeActual(33) + ts;
TimeActual(35) = TimeActual(34) + trapz([newV(30) newV(31)],[F(2, 3) newA(31)]);
for i = 36:43
   TimeActual(i) = TimeActual(i-1) + trapz([newV(i-5) newV(i-4)],[newA(i-5) newA(i-4)]);
end

% Plot Graph of V vs T
figure; hold on;
plot(Time, newV);
plot(TimeActual, newVactual);
legend('Theretical Velocity', 'Actual Velocity');
title('Velocity vs Time');
xlabel('Time [s]');
ylabel('Velocity [m/s]');