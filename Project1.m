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

figure; hold on;

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
       for kz = 1:13
            Pnw(j, kz, i) = Avw(j, kz, i) + Bvw3(j, kz, i); 
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

% MAXIMUM POWER THAT CAN BE TRANSFERRED BY THE TIRES TO THE GROUND

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

% FUEL CONSUMPTION

% Define Constant(s)
rof = 739/1000; % Fuel Density [kg/m^3]
Qm = .62/3600; % Volumetric Flow at idle [l/s];
H = 4.4*10^7; % [J/Kg]

% Define Test Velocity and Time Vectors
Vece15 = [0 0 4.17 4.17 0 0 8.89 8.89 0 0 13.89 13.89 8.89 8.89 0];
Tece15 = [0 11 15 27 32 49 61 85 96 117 143 155 163 176 188];

Veudc = [0 0 19.44 19.44 13.89 13.89 19.44 19.44 27.78 27.78 33.33 33.33 0 0];
Teudc = [0 20 61 111 119 188 201 251 286 316 336 346 380 400];

% Find Equation of All Straight Lines

CoeffEce15 = zeros(14, 2);

for i = 2:14
    CoeffEce15(i, 1:2) = polyfit(Tece15(i:i+1), Vece15(i:i+1), 1); 
end

CoeffEudc = zeros(13, 2);
for i = 2:13
    CoeffEudc(i, 1:2) = polyfit(Teudc(i:i+1), Veudc(i:i+1), 1); 
end

% Find Instantaneous Velocity and Time at Each Second

ViEce15 = zeros(1, 110);
TiEce15 = zeros(1, 110);
ind = 1;
for i = 1:14
    if(Vece15(i+1) >= Vece15(i) && Vece15(i+1) ~=0)
        for j = Tece15(i):Tece15(i+1)
            ViEce15(ind) = CoeffEce15(i,1)*j + CoeffEce15(i,2);
            TiEce15(ind) = j;
            ind = ind+1;
        end
    end 
end

ViEudc = zeros(1, 327);
TiEudc = zeros(1, 327);
ind = 1;
for i = 1:13
   if (Veudc(i+1) >= Veudc(i) && Veudc(i+1) ~= 0)
      for j = Teudc(i):Teudc(i+1)
         ViEudc(ind) = CoeffEudc(i,1)*j + CoeffEudc(i,2);
         TiEudc(ind) = j;
         ind = ind + 1;
      end
   end
end

Vmi = zeros(1, 109);
for i = 1:109
   Vmi(i) = mean([ViEce15(i) ViEce15(i+1)]);
end

VmiE = zeros(1, 326);
for i = 1:326
   VmiE(i) = mean([ViEudc(i) ViEudc(i+1)]);
end

flags = zeros(1, 110);
ind = 1;

for i = 1:110
    flag = 0;
    
   for j = 1:110
       if(ViEce15(j) == ViEce15(i) && flags(i) == 1)
           flag = 1;
       end
   end
   
   if (flag == 0)
       VaEce15(ind) = ViEce15(i);
       for kz = i:110
          if (ViEce15(kz) == ViEce15(i))
              flags(kz) = 1;
          end
       end
       ind = ind + 1;
   end
end

flagsE = zeros(1, 327);
ind = 1;

for i = 1:327
    flag = 0;
    
   for j = 1:327
       if(ViEudc(j) == ViEudc(i) && flagsE(i) == 1)
           flag = 1;
       end
   end
   
   if (flag == 0)
       VaEudc(ind) = ViEudc(i);
       ind = ind + 1;
       for kz = 1:327
          if (ViEudc(kz) == ViEudc(i))
              flagsE(kz) = 1;
          end
       end
   end
end

for i = 1:51
    if (i<6)
       VEce15(i) = VaEce15(i);
    elseif (i>8 && i<21)
        VEce15(i-3) = VaEce15(i);
    elseif (i>21 && i<48)
        VEce15(i-4) = VaEce15(i);
    elseif (i>50)
        VEce15(44) = VaEce15(i);
    end
end

for i = 1:117
   if(i<43)
       VEudc(i) = VaEudc(i);
   elseif (i==45)
       VEudc(i-2) = VaEudc(i);
   elseif (i>47 && i<60)
       VEudc(i-4) = VaEudc(i);
   elseif (i>60 && i<96)
       VEudc(i-5) = VaEudc(i);
   elseif(i>96 && i<117)
      VEudc(i-6) = VaEudc(i); 
   end
end

WeiEce15 = zeros(1, 110);
for i = 1:110
    if (ViEce15(i) < 0.1)
        WeiEce15(i) = 1000;
        
    elseif (ViEce15(i) > 0 && ViEce15(i) < F1(1))
        WeiEce15(i) = ViEce15(i)/Tgi(1)/Tf/Re*60/2/pi;
        
    elseif (ViEce15(i) > F1(1) && ViEce15(i) < F2(1))
        WeiEce15(i) = ViEce15(i)/Tgi(2)/Tf/Re*60/2/pi;
    end
end

WeEce15 = zeros(1, 44);
for i = 1:44
    if (VEce15(i) < 0.1)
        WeEce15(i) = 1000;
        
    elseif (VEce15(i) > 0 && VEce15(i) < F1(1))
        WeEce15(i) = VEce15(i)/Tgi(1)/Tf/Re*60/2/pi;
        
    elseif (VEce15(i) > F1(1) && VEce15(i) < F2(1))
        WeEce15(i) = VEce15(i)/Tgi(2)/Tf/Re*60/2/pi;
    end
end

WeiEudc = zeros(1, 327);
for i = 1:327
    if (ViEudc(i) < 0.1)
        WeiEudc(i) = 1000;
        
    elseif (ViEudc(i) > 0.1 && ViEudc(i) < F1(1))
        WeiEudc(i) = ViEudc(i)/Tgi(1)/Tf/Re*60/2/pi;
        
    elseif (ViEudc(i) > F1(1) && ViEudc(i) < F2(1))
        WeiEudc(i) = ViEudc(i)/Tgi(2)/Tf/Re*60/2/pi;
    
    elseif (ViEudc(i) > F2(1) && ViEudc(i) < F3(1))
        WeiEudc(i) = ViEudc(i)/Tgi(3)/Tf/Re*60/2/pi;
        
    elseif (ViEudc(i) > F3(1) && ViEudc(i) < Vw(4, 13))
        WeiEudc(i) = ViEudc(i)/Tgi(4)/Tf/Re*60/2/pi;
    else
        WeiEudc(i) = ViEudc(i)/Tgi(5)/Tf/Re*60/2/pi;
    end
end

WeEudc = zeros(1, 110);
for i = 1:110
    if (VEudc(i) < 0.1)
        WeEudc(i) = 1000;
        
    elseif (VEudc(i) > 0.1 && VEudc(i) < F1(1))
        WeEudc(i) = VEudc(i)/Tgi(1)/Tf/Re*60/2/pi;
        
    elseif (VEudc(i) > F1(1) && VEudc(i) < F2(1))
        WeEudc(i) = VEudc(i)/Tgi(2)/Tf/Re*60/2/pi;
    
    elseif (VEudc(i) > F2(1) && VEudc(i) < F3(1))
        WeEudc(i) = VEudc(i)/Tgi(3)/Tf/Re*60/2/pi;
        
    elseif (VEudc(i) > F3(1) && VEudc(i) < Vw(4, 13))
        WeEudc(i) = VEudc(i)/Tgi(4)/Tf/Re*60/2/pi;
    else
        WeEudc(i) = VEudc(i)/Tgi(5)/Tf/Re*60/2/pi;
    end
end

PeEce15 = zeros (1, 44);
for i = 1:44
   for j = 1:12
      if (WeEce15(i) > werpm(j) && WeEce15(i) < werpm(j+1))
          coeff = polyfit([werpm(j) werpm(j+1)], [Pe(j) Pe(j+1)], 1);
          PeEce15(i) = coeff(1)*WeEce15(i) + coeff(2);
          
      else
          coeff = polyfit([0 werpm(1)], [0 Pe(1)], 1);
          PeEce15(i) = coeff(1)*WeEce15(i) + coeff(2);
      end
   end
end

PeEudc = zeros (1, 110);
for i = 1:110
   for j = 1:12
      if (WeEudc(i) > werpm(j) && WeEudc(i) < werpm(j+1))
          coeff = polyfit([werpm(j) werpm(j+1)], [Pe(j) Pe(j+1)], 1);
          PeEudc(i) = coeff(1)*WeEudc(i) + coeff(2);
          
      else
          coeff = polyfit([0 werpm(1)], [0 Pe(1)], 1);
          PeEudc(i) = coeff(1)*WeEudc(i) + coeff(2);
      end
   end
end

PmeEce15 = zeros(1,44);
for i = 1:44
   PmeEce15(i) = 10*PeEce15(i)/Vcil/k/(WeEce15(i)/60);
end

coeff(1,1:2) = polyfit([0 werpm(1)], [1/H, 220.11], 1);
coeff(2,1:2) = polyfit([werpm(1) werpm(2)], [220.11 205.94], 1);
coeff(3, 1:2) = polyfit([werpm(2) werpm(3)], [205.94 205.61], 1);
coeff(4,1:2) = polyfit([werpm(3) werpm(4)], [205.61 201.58], 1);
coeff(5,1:2) = polyfit([werpm(4) werpm(5)], [201.58 204.2], 1);
coeff(6, 1:2) = polyfit([werpm(5) werpm(6)], [204.2 206.08], 1);
coeff(7, 1:2) = polyfit([werpm(6) werpm(7)], [206.08 207.39], 1);
coeff(8, 1:2) = polyfit([werpm(7) werpm(8)], [207.39 211.16], 1);
coeff(9, 1:2) = polyfit([werpm(8) werpm(9)], [211.16 212.22], 1);
coeff(10, 1:2) = polyfit([werpm(9) werpm(10)], [212.22 216.16], 1);
coeff(11, 1:2) = polyfit([werpm(10) werpm(11)], [216.16 222.5], 1);
coeff(12, 1:2) = polyfit([werpm(11) werpm(12)], [222.5 235], 1);
coeff(13, 1:2) = polyfit([werpm(12) werpm(13)], [221.89 286.06], 1);

q = zeros(1, 44);
q(1) = 220.11/(1000*3600*745.7);
kz = 1;
for i = 2:44
    if (WeEce15(i)<1000)
            kz = 1;
            
    else
        for j = 1:12
            if(WeEce15(i)>werpm(j) && WeEce15(i)<werpm(j+1))
                kz = j;
            end
        end
    end
   q(i) = (coeff(kz, 1)*WeEce15(i) + coeff(kz, 2))/(1000*3600*745.7);
end

Q1i = zeros(1, 99);
kz=1;
ind = 1;
for i = 1:109
    if(TiEce15(i)~=TiEce15(i+1) && ViEce15(i+1)>=ViEce15(i))
        if (ViEce15(i) > 0.1)
            for j = 1:44
                if (ViEce15(i) == VEce15(j))
                    kz = j;
                end
            end
            
        else
            for j = 1:44
                if (ViEce15(i+1) == VEce15(j))
                    kz = j;
                end
            end
        end
        
        if (ViEce15(i) < F1(1))
            Q1i(ind) = ((A(1)*Vmi(i)+B(1)*Vmi(i)^3) + me(1)*Vmi(i)*((ViEce15(i+1)-ViEce15(i))))*q(kz)/rof/Tf;
            ind = ind + 1;
        else
            Q1i(ind) = ((A(1)*Vmi(i)+B(1)*Vmi(i)^3) + me(2)*Vmi(i)*((ViEce15(i+1)-ViEce15(i))))*q(kz)/rof/Tf;
            ind = ind + 1;
        end
    end
end

 Qzeros1 = Qm*49;
 QiOne = 4*(sum(Q1i) + Qzeros1);
 
 
PmeEudc = zeros(1,110);
for i = 1:110
   PmeEudc(i) = 10*PeEudc(i)/Vcil/k/(WeEudc(i)/60);
end

coeff(14, 1:2) = polyfit([werpm(12) werpm(13)], [225.65 286.06], 1);

q2 = zeros(1, 110);
q2(1) = 220.11/(1000*3600*745.7);

kz=1;
for i = 2:110
    if (WeEudc(i)<1000)
        kz=1;
    else
        for j = 1:12
           if(WeEudc(i)>werpm(j) && WeEudc(i)<werpm(j+1))
              kz = j; 
           end
        end
        if(kz==13 && PmeEudc(i)<7.2)
           kz=14; 
        end
    end
   q2(i) = (coeff(kz, 1)*WeEudc(i) + coeff(kz, 2))/(1000*3600*745.7);
end

Q2i = zeros(1, 316);
kz=1;
ind = 1;
for i = 1:326
    if(TiEudc(i)~=TiEudc(i+1) && ViEudc(i+1)>=ViEudc(i))
        if (ViEudc(i) > 0.1)
            for j = 1:110
                if (ViEudc(i) == VEudc(j))
                    kz = j;
                end
            end
            
        else
            for j = 1:110
                if (ViEudc(i+1) == VEudc(j))
                    kz = j;
                end
            end
        end
        
        if (ViEudc(i)<F1(1))
            Q2i(ind) = ((A(1)*VmiE(i)+B(1)*VmiE(i)^3) + me(1)*VmiE(i)*(ViEudc(i+1)-ViEudc(i)))*q2(kz)/rof/Tf;
            ind = ind + 1;
        elseif (ViEudc(i)>F1(1) && ViEudc(i)<F2(1))
            Q2i(ind) = ((A(1)*VmiE(i)+B(1)*VmiE(i)^3) + me(2)*VmiE(i)*(ViEudc(i+1)-ViEudc(i)))*q2(kz)/rof/Tf;
            ind = ind + 1;
        elseif (ViEudc(i)>F2(1) && ViEudc(i)<F3(1))
            Q2i(ind) = ((A(1)*VmiE(i)+B(1)*VmiE(i)^3) + me(3)*VmiE(i)*(ViEudc(i+1)-ViEudc(i)))*q2(kz)/rof/Tf;
            ind = ind + 1;
        elseif (ViEudc(i)>F3(1) && ViEudc(i)<Vw(4,13))
            Q2i(ind) = ((A(1)*VmiE(i)+B(1)*VmiE(i)^3) + me(4)*VmiE(i)*(ViEudc(i+1)-ViEudc(i)))*q2(kz)/rof/Tf;
            ind = ind + 1;
        else
            Q2i(ind) = ((A(1)*VmiE(i)+B(1)*VmiE(i)^3) + me(5)*VmiE(i)*(ViEudc(i+1)-ViEudc(i)))*q2(kz)/rof/Tf;
            ind = ind + 1;
        end
    end
end

Qzeros2 = Qm*40;
QiTwo = sum(Q2i) + Qzeros2;

Qtotal = 100/11*(QiOne + QiTwo);