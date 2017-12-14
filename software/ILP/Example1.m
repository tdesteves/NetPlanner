clear all
lp=mxlpsolve('make_lp', 0, 2);                                      % initialize the LP model
mxlpsolve('set_sense', lp, 1);                                       % maximaze
mxlpsolve('set_obj_fn', lp, [120, 160]);                        % define the objetive function
mxlpsolve('add_constraint', lp, [1, 1.5], 1, 150);          % add constrain, x_1 + 1.5 <= 150
mxlpsolve('add_constraint', lp, [4, 3], 1, 360); 
mxlpsolve('set_int', lp, 1, 1);                                         % add restriction
mxlpsolve('set_int', lp, 2, 1);
mxlpsolve('write_lp', lp, 'Example.lp');                          
mxlpsolve('solve', lp);
obj = mxlpsolve('get_objective', lp);
var = mxlpsolve('get_variables', lp);
