mp = 0.05;
mc = 0.1;
g = 9.81;
l = 0.45;

x = [0; 0; -20; 0]; %Example State Vector

A = [0 1 0 0; 0 0 ((-mp*g)/mc) 0; 0 0 0 1; 0 0 ((mc+mp)/(mc*l)) 0];

B = [0; 1/mc; 0; 1/(mc*l)];

Q = diag([10, 1, 100, 1]);

R = 1;

K = lqr(A, B, Q, R)

u = -dot(K, x) %Example Control Calculation

% %Arduino Code
% float K[1][4] = {
%     {-10.0, -2.5, -120.0, -3.0}
% };
% 
% float state[4] = {0, 0, 0, 0};
% 
% float controlInput = 0;
% 
% float computeControlInput() {
%     float u = 0;
%     for (int i = 0; i < 4; i++) {
%         u -= K[0][i] * state[i];  // u = -K * state
%     }
%     return u;
% }
% 
% state[0] = readPosition();          // Cart position
% state[1] = readVelocity();          // Cart velocity
% state[2] = readAngle();             // Pendulum angle
% state[3] = readAngularVelocity();   // Pendulum angular velocity
% 
% controlInput = computeControlInput();
