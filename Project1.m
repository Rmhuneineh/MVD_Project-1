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
b = l-a; %[mm]
hG = .5; %[m]
wheel_indication = '155/65R13';
R0 = 13*25.4/2 + .65*155; %[mm]
Re = .98*R0*10^-3; % [m]
N = 5; %[-]
ts = 1; %[s]
Je = .08; %[kg.m^2]
Jw = 1.6; %[kg.m^2]
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

% for i = 1:2
%     subplot(3, 3, i);
%     plot(vKm, R(i, 1:31), 'g');
%     xlabel('Velocity [km/h]');
%     ylabel('Resistance [N]');
%     title(['Resistance vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
%     grid on;
% end
% 
% for i = 1:2
%     subplot(3, 3, i+2);
%     loglog(vKm, Pn(i, 1:31)); hold on;
%     loglog(vKm, Av(i, 1:31));
%     loglog(vKm, Bv3(i, 1:31));
%     xlabel('log(V)');
%     ylabel('log(Pn)');
%     title(['Power vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
%     grid on;
% end
% 
% for i = 1:5
%     subplot(3, 3, i+4);
%     loglog(Vnorm, Pnorm(i, 1:31));
%     xlabel('log(Vnorm)');
%     ylabel('log(Pnorm)');
%     title(['Pnorm vs Vnorm @ tan(alpha) = ', num2str(tan(alpha(i)))]);
%     grid on;
% end

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
% figure; hold on;
% subplot(2, 2, 1);
% plot(werpm, Pe);
% xlabel('We [rpm]');
% ylabel('Pe [W]');
% title('Power vs RPM');
% grid on;
% subplot(2, 2, 2);
% plot(werpm, Me);
% xlabel('We [rpm]');
% ylabel('Me [Nm]');
% title('Torque vs RPM');
% grid on;

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
% subplot(2, 2, 3);
% plot(werpm, Peapp);
% xlabel('We [rpm]');
% ylabel('Pe [W]');
% title('Approximated Power vs RPM');
% grid on;

% Calculate The Power Available At The Wheels
Pa = zeros(13, 1);
Paapp = zeros(13, 1);
for i = 1:13
   Pa(i) = efficiencyt*Pe(i); % [W]
   Paapp(i) = efficiencyt*Peapp(i); % [W]
end

% Plot The Graph Of Power Available At The Wheels
% subplot(2, 2, 4);
% plot(werpm, Pa);
% xlabel('We [rpm]');
% ylabel('Pa [W]');
% title('Power Available at the Wheels vs RPM');
% grid on;

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

% Calculate Power Needed for Motion According to New Values of Velocity

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

% Plot Required Graphs

% figure;  hold on;
% for i = 1:5
%     plot(Vw(i, 1:13), Pa, Vw(i, 1:13), Pnw(i, 1:13, 1), Vw(i, 1:13), Pnw(i, 1:13, 2));
% end
% xlabel('V [m/s]');
% ylabel('Pa | Pn [W]');
% title('Power Available & Power Needed (null and max slope) @ Different Gear Ratios');
% 
% grid on;

%  MAXIMUM POWER THAT CAN BE TRANSFERRED BY THE TIRES TO THE GROUND

% Defining New Constants
Rl = .92*(13*25.4*10^-3/2 +155*10^-3*.65); %[m]
c1 = [1.1, .8];
c2 = [6*10^-3, 8*10^-3]; %[s/m]
K1 = ro*S*Cx*hG/2/m/g; %[s^2/m]
K2 = -K1; %[s^2/m]
Dx = zeros(5, 13);
for i = 1:5
   for j = 1:13
      Dx(i, j) = Rl*(f0 + K*Vw(i, j)^2); %[m]
   end
end

% Calculate The Forces Acting in z Direction
Fz1 = zeros(5, 13);
Fz2 = zeros(5 ,13);

for i = 1:5
   for j = 1:13
      Fz1(i, j) = m*g*((b - Dx(i, j))*cos(alpha(1)) - hG*sin(alpha(1)) - K1*Vw(i, j)^2)/l; %[N]
      Fz2(i, j) = m*g*((a + Dx(i, j))*cos(alpha(1)) - hG*sin(alpha(1)) - K2*Vw(i, j)^2)/l; %[N]
   end
end

% Compute Longitudinal Force Coefficient
uip = zeros(5, 13, 2);
for i = 1:2
   for j = 1:5
       for k = 1:13
            uip(j, k, i) = c1(i) - c2(i)*Vw(j, k); %[-]
       end
   end
end

% Calculate Max Power
Fz1u = zeros(5, 13, 2);
Fz2u = zeros(5, 13, 2);
for k = 1:2
    for i = 1:5
        for j = 1:13
            Fz1u(i, j, k) = Fz1(i, j)*uip(i, j, k); %[N]
            Fz2u(i, j, k) = Fz2(i, j)*uip(i, j, k); %[N]
        end
    end
end
FzTu = Fz1u + Fz2u;
PmaxWG = zeros(5, 13, 2);
for k = 1:2
    for i = 1:5
        for j = 1:13
            PmaxWG(i, j, k) = Vw(i, j)*FzTu(i, j, k); %[W]
        end
    end
end

f = zeros(5, 13);
PnZ = zeros(5, 13);
for i = 1:5
   for j = 1:13
      f(i, j) = f0 + K*Vw(i, j)^2;
      PnZ(i, j) = ((Fz1(i, j) + Fz2(i, j))*f(i, j) + (.5*ro*S*Cx*Vw(i, j)^2))*Vw(i, j);
   end
end

figure; hold on;
for i = 1:5
   plot(Vw(i, 1:13), PmaxWG(i, 1:13, 1), 'g');
   plot(Vw(i, 1:13), PmaxWG(i, 1:13, 2), 'r');
   plot(Vw(i, 1:13), PnZ(i, 1:13), 'm');
end
legend('Dry', 'Wet', 'Needed');
xlabel('V [m/s]');
ylabel('P [W]');
