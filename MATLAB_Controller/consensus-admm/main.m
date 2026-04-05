close all, clear all;

%EXPERIMENTAL CASES
% case 1
L1 = 150; d1 = 30; L2 = 80; d2 = 0;
% case 2
%L1 = 80; d1 = 50; L2 = 150; d2 = 50;
% case 3
%L1 = 80; d1 = 0; L2 = 170; d2 = 50;

%COST FUNCTION PARAMETERS
%symmetric costs
c1 = 1; c2 = 1; 
%asymmetric costs
c1 = 3; c2 = 1;

%SOLVER PARAMETERS
rho = 0.02;
maxiter = 50;

%SYSTEM CALIBRATION PARAMETERS
k11 = 2; k12 = 1; k21 = 0.5; k22 = 2;


%VARIABLES FOR CENTRALIZED SOLUTION
K = [k11, k12 ; k21 , k22];
c = [c1;c2]; 
L = [L1;L2]; 
d = [d1;d2];

%VARIABLES FOR STORING THE HISTORY OF THE DISTRIBUTED SOLUTION
u11 = zeros(1,maxiter);
u12 = zeros(1,maxiter);
u21 = zeros(1,maxiter);
u22 = zeros(1,maxiter);
av1 = zeros(1,maxiter);
av2 = zeros(1,maxiter);

%DISTRIBUTED NODE INITIALIZATION

%node1
node1.index = 1;
node1.u = [0;0];
node1.u_av = [0;0];
node1.lambda = [0;0];
node1.k = [k11;k12]; 
node1.n = norm(node1.k)^2;
node1.m = node1.n-k11^2;
node1.c = [c1;0];
node1.d = d1;
node1.L = L1;

%node 2
node2.index = 2;
node2.u = [0;0];
node2.u_av = [0;0];
node2.lambda = [0;0];
node2.k = [k21;k22]; 
node2.n = norm(node2.k)^2;
node2.m = node2.n-k22^2;
node2.c = [0;c2];
node2.d = d2;
node2.L = L2;


%DISTRIBUTED SOLVER

%Initial condition (iteration = 1)
u11(1) = node1.u(1);
u12(1) = node1.u(2);
u21(1) = node2.u(1);
u22(1) = node2.u(2);
av1(1) = (u11(1)+u21(1))/2;
av2(1) = (u12(1)+u22(1))/2;

%iterations
for i=2:maxiter
   %COMPUTATION OF THE PRIMAL SOLUTIONS
   % node 1
   [u1, cost1] = consensus_iterate(node1, rho);
   node1.u = u1;
   %DEBUG: check with matlab quadprog
   Q_ = [rho, 0; 0 rho];
   c_ = node1.c+node1.lambda - rho*node1.u_av;
   A_ = [-k11 -k12; -1 0; 1 0];
   b_ = [d1-L1, 0, 100];
   u1_ = quadprog(Q_,c_,A_,b_,[],[],[],[],[],optimset('display','off'));
  
   
   %node2
   [u2, cost2] = consensus_iterate(node2, rho);
   node2.u = u2;
   %DEBUG: check with matlab quadprog
   Q_ = [rho, 0; 0 rho];
   c_ = node2.c+node2.lambda - rho*node2.u_av;
   A_ = [-k21 -k22;0 -1; 0 1];
   b_ = [d2-L2, 0, 100];
   u2_ = quadprog(Q_,c_,A_,b_,[],[],[],[], [], optimset('display','off'));
  
 
   % NODES EXCHAGE THEIR SOLUTIONS
   % (COMMUNICATIONS HERE)
   
   %COMPUTATION OF THE AVERAGE
   %node 1
   node1.u_av = (node1.u+node2.u)/2;
   %node 2
   node2.u_av = (node1.u+node2.u)/2;
   
   
   %COMPUTATION OF THE LAGRANGIAN UPDATES
   %node 1
   node1.lambda = node1.lambda + rho*(node1.u-node1.u_av);  
   %node 2
   node2.lambda = node2.lambda + rho*(node2.u-node2.u_av);
    
   %SAVING DATA FOR PLOTS
   u11(i) = node1.u(1);
   u12(i) = node1.u(2);
   u21(i) = node2.u(1);
   u22(i) = node2.u(2);
   av1(i) = (u11(i)+u21(i))/2;
   av2(i) = (u12(i)+u22(i))/2;
end;

% CENTRALIZED SOLVER: USIGN MATLAB LINPROG
A_ = -K; 
b_ = [d1-L1; d2-L2];
c_ = c;
lb = [0;0]; ub = [100;100];
u_ = linprog(c_,A_,b_,[],[],lb,ub, optimset('display','off'));
l_ = K*u_+d;

%DISPLAY DATA
disp('Matlab solutions')
u_
l_

disp('Consensus Solutions')
u = node1.u_av
l = K*u+d

%Plots
figure(10);
plot(1:50, av1, 1:50, av2, 1:50, u11, 1:50, u12, 1:50, u21, 1:50, u22);
legend('av1','av2', 'u11', 'u12', 'u21', 'u22');
title('time convergence');
xlabel('iter');

figure(15);
t = 0:100;
constr1 = (L1-d1)/k12-(k11/k12)*t;
constr2 = (L2-d2)/k22-(k21/k22)*t;
[x,y]=meshgrid(t,t);
hold on;
z = c1*x+c2*y;
contour(x,y,z);
plot(t,constr1,t,constr2,'LineWidth',2);
plot(t,zeros(size(t)),'k','LineWidth',2);
plot(zeros(size(t)),t,'k','LineWidth',2);
plot(t,100*ones(size(t)),'k','LineWidth',2);
plot(100*ones(size(t)),t,'k','LineWidth',2);
plot(av1,av2,'b--o','LineWidth',2);
%plot(av1,av2,'bo');
plot(u11,u12,'m-.x','LineWidth',1);
% plot(u11,u12,'mx');
plot(u21,u22,'k-.+','LineWidth',1);
% plot(u21,u22,'kx');
plot(u_(1),u_(2),'g*')
legend('cost','c1', 'c2', 'c3', 'c4', 'c5', 'c6', 'av', 'u1', 'u2', 'optim');
title('solution space convergence');
xlabel('u_1');
ylabel('u_2');
axis([-10,110,-10,110]);
hold off;





