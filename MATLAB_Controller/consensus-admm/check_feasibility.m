%FUNCTION TO CHECK SOLUTION FEASIBILITY
function check = check_feasibility(node, u)
   tol = 0.001; %%tolerance for rounding errors
   if (u(node.index) < 0-tol), check = 0; return; end;
   if (u(node.index) > 100+tol), check = 0; return; end;
   if (u'*node.k < node.L-node.d-tol), check = 0; return; end;
   check = 1;
end