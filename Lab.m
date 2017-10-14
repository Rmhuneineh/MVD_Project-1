clear all
clc

Cx = .35;
S = 1.82;
m = 970;
Vcil = 1040*10^-6;
Tf = .281;
wmin = 850;
efficiencyt = .9;
l = 2.3;
a = 1.15;
hG = .5;
wheel_indication = '155/65R13';
N = 5;
ts = 1;
Je = .08;
Jw = 1.6;
g = 9.81;
ro = 1.3;
alpha = atan(0:0.1:0.4);
f0 = .013;
K = 6.51*10^-6;

vKm = 0:10:300;

vm = vKm./3.6;

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
    A(i) = m*g*(f0*cos(alpha(i)*pi) + sin(alpha(i)));
    B(i) = m*g*K*cos(alpha(i)) + 0.5*ro*S*Cx;

    Vcar(i) = sqrt(A(1)/B(1));
    Pcar(i) = 2*A(i)*sqrt(A(i)/B(i));
        
    for j = 1:31
        R(i, j) = A(i) + B(i)*(vm(j).^2);
        Av(i, j) = A(i)*vm(j);
        Bv3(i, j) = B(i)*vm(j).^3;
        P(i, j) = Av(i, j) + Bv3(i, j);
        Pnorm(i, j) = P(i, j)/Pcar(i);
    end

    Vnorm = vm/Vcar(i);
    
end

figure; hold on;

for i = 1:2
    subplot(2, 2, i);
    plot(vm, R(i, 1:31), 'g');
    xlabel('Velocity (m/s)');
    ylabel('Resistance (N)');
    title(['Resistance vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

for i = 1:2
    subplot(2, 2, i+2);
    plot(log(vm), log(P(i, 1:31)), log(vm), log(Av(i, 1:31)), log(vm), log(Bv3(i, 1:31)));
    xlabel('log(V)');
    ylabel('log(P)');
    title(['Power vs Velocity @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

figure; hold on;

for i = 1:5
    subplot(2, 3, i);
    plot(log(Vnorm), log(Pnorm(i, 1:31)));
    xlabel('log(Vnorm)');
    ylabel('log(Pnorm)');
    title(['Pnorm vs Vnorm @ tan(alpha) = ', num2str(tan(alpha(i)))]);
end

% Part 2
we = [1000, 1500, 2000, 2500, 3000, 3250, 3500, 4000, 4500, 5000, 5500, 6000, 6500];
mep = [5.985, 8.5785, 8.841, 9.3345, 9.513, 9.6705, 9.6075, 9.6285, 9.4815, 8.9250, 8.2530, 7.791, 7.119];