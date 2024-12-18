clc; close all; clear all;

% seguindo passo a passo dado em exercício de aula

% Parâmetros do sistema
mc = 1.5; % massa do carrinho
mp = 0.5; % massa do pêndulo
g = 9.82; % gravidade
L = 1;    % comprimento do pêndulo
d1 = 1e-2;  % amortecimento do deslocamento do carrinho
d2 = 1e-2; % amortecimento da junta

% Matrizes do modelo linearizado
A = [0,   0,   1,    0;
     0,   0,   0,    1;
     0,   g*mp/mc,   -d1/mc, -d2/(L*mc);
     0,   g*(mc+mp)/(L*mc),  -d1/(L*mc), -d2*(mc+mp)/(L^2*mc*mp)];

B = [0; 0; 1/mc;  1/(L*mc)];

C = [1, 0, 0, 0]; 
D = 0;

% Definindo o peso em Q(1,1)
rho = 100; 
Q = diag([rho, 0, 0, 0]); 
R = 1;

% Calculando o controlador LQR
[K, P] = lqr(A, B, Q, R)
Ac = A - B * K;

% Resposta ao degrau
figure(1);
step(ss(Ac, B, C, D)); grid on;
title('Resposta ao Degrau para \rho = 100');

% Correção para regime permanente
rss = 1;
A_ = [A, B; C, 0];
xu = A_ \ [0; 0; 0; 0; 1];
xss = xu(1:4);
uss = xu(5);
Nx = xss / rss;
Nu = uss / rss;
N_ = Nu + K * Nx

% Resposta normalizada ao degrau
figure(2);
step(ss(Ac, N_ * B, C, D)); grid on;
title('Resposta Normalizada ao Degrau para \rho = 100');

% Esforço de controle
t = 0:0.01:1.6;
r = 2 * ones(size(t)); 
[y, t, x] = lsim(ss(Ac, N_ * B, C, D), r, t);
u = N_ * rss - K * x'; 

figure(3);
plot(t, u); grid on;
title('Esforço de Controle u(t) para \rho = 100');
xlabel('Tempo (s)');
ylabel('u(t)');
