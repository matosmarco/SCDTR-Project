%FUNCTION TO COMPUTE THE PRIMAL SOLUTION
function [u, cost] = consensus_iterate(node, rho)
    u_best = [-1,-1]';
    cost_best = 1000000; %large number
    sol_unconstrained = 1;
    sol_boundary_linear = 1;
    sol_boundary_0 = 1;
    sol_boundary_100 = 1;
    sol_linear_0 = 1;
    sol_linear_100 = 1;
    z = rho*node.u_av - node.lambda - node.c;
    %unconstrained minimum
    u_u = (1/rho)*z;
    sol_unconstrained = check_feasibility(node,u_u);
    if sol_unconstrained
        %REVISE: IF UNCONSTRAINED SOLUTION EXISTS, THEN IT IS OPTIMAL
        %NO NEED TO COMPUTE THE OTHER
        cost_unconstrained = evaluate_cost(node, u_u, rho);
        if cost_unconstrained < cost_best
           u_best = u_u;
           cost_best = cost_unconstrained;
        end;
    end;
    %compute minimum constrained to linear boundary   
    u_bl = (1/rho)*z - node.k/node.n*(node.d-node.L+(1/rho)*z'*node.k);
    %check feasibility of minimum constrained to linear boundary
    sol_boundary_linear = check_feasibility(node, u_bl);
    % compute cost and if best store new optimum
    if sol_boundary_linear 
        cost_boundary_linear = evaluate_cost(node, u_bl,rho);
        if cost_boundary_linear < cost_best
           u_best = u_bl;
           cost_best = cost_boundary_linear;
        end;
    end;
    %compute minimum constrained to 0 boundary
    u_b0 = (1/rho)*z;
    u_b0(node.index) = 0;
    %check feasibility of minimum constrained to 0 boundary
    sol_boundary_0 = check_feasibility(node, u_b0);
    % compute cost and if best store new optimum
    if sol_boundary_0 
        cost_boundary_0 = evaluate_cost(node, u_b0,rho);
        if cost_boundary_0 < cost_best
           u_best = u_b0;
           cost_best = cost_boundary_0;
        end;
    end;
    %compute minimum constrained to 100 boundary
    u_b1 = (1/rho)*z;
    u_b1(node.index) = 100;
    %check feasibility of minimum constrained to 100 boundary
    sol_boundary_100 = check_feasibility(node, u_b1);
    % compute cost and if best store new optimum
    if sol_boundary_100 
        cost_boundary_100 = evaluate_cost(node, u_b1,rho);
        if cost_boundary_100 < cost_best
           u_best = u_b1;
           cost_best = cost_boundary_100;
        end;
    end;
    % compute minimum constrained to linear and 0 boundary
    u_l0 = (1/rho)*z - ...
          (1/node.m)*node.k*(node.d-node.L) + ...
          (1/rho/node.m)*node.k*(node.k(node.index)*z(node.index)-z'*node.k);
    u_l0(node.index) = 0;
    %check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, u_l0);
    % compute cost and if best store new optimum
    if sol_linear_0 
        cost_linear_0 = evaluate_cost(node, u_l0,rho);
        if cost_linear_0 < cost_best
           u_best = u_l0;
           cost_best = cost_linear_0;
        end;
    end;
    % compute minimum constrained to linear and 100 boundary
    u_l1 = (1/rho)*z - ...
          (1/node.m)*node.k*(node.d-node.L+100*node.k(node.index)) + ...
          (1/rho/node.m)*node.k*(node.k(node.index)*z(node.index)-z'*node.k);
    u_l1(node.index) = 100;
    %check feasibility of minimum constrained to linear and 0 boundary
    sol_linear_0 = check_feasibility(node, u_l1);
    % compute cost and if best store new optimum
    if sol_linear_0 
        cost_linear_0 = evaluate_cost(node, u_l1,rho);
        if cost_linear_0 < cost_best
           u_best = u_l1;
           cost_best = cost_linear_0;
        end;
    end;
    u = u_best;
    cost = cost_best;
end