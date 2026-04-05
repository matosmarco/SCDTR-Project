%FUNCTION TO COMPUTE THE AUGMENTED LAGRANGIAN COST AT A POSSIBLE SOLUTION
%USED BY CONSENSUS_ITERATE
function cost = evaluate_cost(node,u,rho)
    cost =  node.c'*u + node.lambda'*(u-node.u_av) + ...
            rho/2*norm(u-node.u_av)^2;
end