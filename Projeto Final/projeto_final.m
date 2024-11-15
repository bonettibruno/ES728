%% Modelagem do carro pendulo

mc = 1.5; % mass of the cart
mp = 0.5; % mass of the pendulum
g = 9.82; % gravity
L = 1;    % length of the pendulum
d1 = 1e-2;  % damping of the cart displacement
d2 = 1e-2; % damping of the joint

A = [0,   0,   1,    0;
    0,   0,   0,    1;
    0,   g*mp/mc,   -d1/mc, -d2/(L*mc);
    0,   g*(mc+mp)/(L*mc),  -d1/(L*mc), -d2*(mc+mp)/(L^2*mc*mp)];

B = [ 0; 0; 1/mc;  1/(L*mc)];

C = [0 1 0 0];

D = 0;

sys = ss(A, B, C, D)

%% Análise de estabilidade

poles = pole(sys)
eigenvalues = eig(A)
zeros_sys = zero(sys)

% como tem polo maior que 0, não é estavel (tem que estar no semiplano
% esquerdo)



