%Toy problem code for dual decompostition: only works for two nodes
clear all;
close all;

%Experimental cases
L1 = 150; d1 = 30; L2 = 80; d2 = 0;

%COST FUNCTION PARAMETERS
c1 = 1; c2 = 1; 
%c1 = 1; c2 = 6;
%c1 = 6; c2 = 1;

% SOLVER PARAMETERS
MAX_ITER = 50;
alpha = 0.005; % constant gradient step common to all nodes

%Small quadratic costs for regularization
q1 = 0.01; q2 = 0.01;

% System calibration parameters
K11 = 2; K12 = 1; K21 = 1; K22 = 2;

%VARIABLES FOR CENTRALIZED SOLUTION
L = [L1;L2]; 
d = [d1;d2];
K = [K11 , K12 ; K21 , K22];
c = [c1 c2]; 
Q = [q1 0; 0 q2];

% SOLVE WITH MATLAB QUADPROG
A = -K; b = [d1-L1; d2-L2];
lb = [0;0]; ub = [100; 100];
disp('Matlab solutions')
u_matlab = quadprog(2*Q,c,A,b,[],[],lb,ub)
l_matlab = K*u_matlab+d

% SOLVE WITH DUAL DECOMPOSITION

node1 = initialize_node(1, c1, K11, K12, d1, L1, q1, alpha);
node2 = initialize_node(2, c2, K22, K21, d2, L2, q2, alpha);

%iterations
for i=1:MAX_ITER,
   % node 1 compute u
   u1 = compute_u(node1);
   node1.u_own = u1;
   node1.u_ant = u1;
   u1_store(i) = u1;               %save for plots

   % node 2 compute u
   u2 = compute_u(node2);
   node2.u_own = u2;
   node2.u_ant = u2;
   u2_store(i) = u2;               %save for plots

   %communicate the solutions to the other nodes 
   node1.u_other = u2;
   node2.u_other = u1;

   %compute the lambdas
   l1 = compute_l(node1);
   node1.l_own = l1;
   l1_store(i) = l1;
   
   l2 = compute_l(node2);
   node2.l_own = l2;
   l2_store(i) = l2;

   %compute price for other nodes
   p12 = compute_price(node1);
   p21 = compute_price(node2);
   %coomunicate the prices
   node1.price_other = p21;
   node2.price_other = p12;
end;



disp('Dual Decomposition Solutions')
u_dualdecomp = [node1.u_own; node2.u_own]
l_dualdecomp = K*u_dualdecomp+d
%Plots
figure(12);
plot(1:50, u1_store, 1:50, u2_store);
legend('u_1','u_2');
title('primal vars');
xlabel('iter');
figure(13);
plot(1:50,l1_store,1:50,l2_store);
legend('\lambda_1','\lambda_2')
title('dual vars');
xlabel('iter');
figure(15);
t = 0:100;
constr1 = (L1-d1-K11*t)/K12;
constr2 = (L2-d2-K21*t)/K22;
[x,y]=meshgrid(t,t);
hold on;
z = c1*x+c2*y+q1*x.^2+q2*y.^2;
contour(x,y,z);
plot(t,constr1,t,constr2,'LineWidth',2);
plot(t,zeros(size(t)),'k','LineWidth',2);
plot(zeros(size(t)),t,'k','LineWidth',2);
plot(t,100*ones(size(t)),'k','LineWidth',2);
plot(100*ones(size(t)),t,'k','LineWidth',2);
plot(u1_store,u2_store,'--','LineWidth',2);
plot(u1_store,u2_store,'bo');
title('trajectory');
xlabel('u_1');
ylabel('u_2');
plot(u_matlab(1),u_matlab(2),'r*')
axis([-10,110,-10,110]);
hold off;

function node = initialize_node(i, c, K_own, K_other, d, L, q, alpha)
   node.i = i;
   node.u_ant = 0; %previsous value for filtering
   node.u_own = 0;  %primal variables 
   node.u_other = 0; % commmunicated data
   node.l_own = 0;   %dual variables
   node.price_other = 0; %communicated data
   node.c = c;
   node.k_own = K_own;
   node.k_other = K_other;
   node.L = L;
   node.d = d;
   node.q = q;
   node.alpha = alpha;
end

function u = compute_u( node )
   u = (-node.c+node.l_own*node.k_own+node.price_other)/(2*node.q);
   u = 0.5*u + 0.5*node.u_ant;
   if u < 0, u = 0; end
   if u > 100; u = 100; end
end

function l = compute_l( node )
   l = node.l_own + node.alpha*(-node.u_own*node.k_own-node.u_other*node.k_other-node.d+node.L);
   if l < 0, l = 0; end
end

function p = compute_price( node )
   p = node.l_own*node.k_other;
end

