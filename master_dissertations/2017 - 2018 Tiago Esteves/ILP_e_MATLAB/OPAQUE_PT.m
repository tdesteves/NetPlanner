clear all
%inputs
% G=[0,1,1,0;1,0,0,1;1,0,0,1;0,1,1,0];
% D(:,:,1)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,2)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,3)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,4)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,5)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% 
% n=4;

G=[0,1,1,0,0,0;1,0,1,1,0,0;1,1,0,0,1,0;0,1,0,0,1,1;0,0,1,1,0,1;0,0,0,1,1,0];

D(:,:,1)=[0,5,1,3,1,3;5,0,0,1,5,0;1,0,0,1,4,1;3,1,1,0,1,1;1,5,4,1,0,3;3,0,1,1,3,0];
D(:,:,2)=[0,2,4,2,0,5;2,0,0,3,1,1;4,0,0,1,1,0;2,3,1,0,1,3;0,1,1,1,0,1;5,1,0,3,1,0];
D(:,:,3)=[0,1,1,1,0,0;1,0,0,0,1,0;1,0,0,1,1,0;1,0,1,0,1,0;0,1,1,1,0,1;0,0,0,0,1,0];
D(:,:,4)=[0,0,0,0,0,0;0,0,1,0,0,1;0,1,0,0,1,0;0,0,0,0,0,0;0,0,1,0,0,0;0,1,0,0,0,0];
D(:,:,5)=[0,0,0,0,0,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,1;0,1,0,0,1,0];
n=6;

%ILP
number_flows = (n * (n-1))/2;

var_f = number_flows * (n * (n-1));
var_W = number_flows;
total_var = var_f+var_W;

ilp=mxlpsolve('make_lp', 0, total_var);

%OBJECTIVE FUNCTION
f_row = ones(1,total_var);
mxlpsolve('set_obj_fn', ilp, f_row);

%CONSTRAINTS
%FLOW CONSERVATION CONSTRAINTS
for o=1:n   
    for d=o+1:n
        for i=1:n 
            %ORIGIN NODES
            if(i==o)
                %sum over all i
                index_sum = [];
                f_row = zeros(1,total_var);
                for j=1:n
                  if(j~=o && G(i,j)==1)
                    f_index = index_calculation2(i,j,o,d,n);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 1);
            
             %INTERMEDIATE NODES
             elseif (i ~=o && i ~=d)
                index_sum = [];
                f_row = zeros(1,total_var);
                %income flow
                for j=1:n
                  if(j~=o && G(i,j)==1) 
                    f_index = index_calculation2(i,j,o,d,n);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=1;
                
                %outcome flow   
                index_sum = [];
                for j=1:n
                  if(j~=d && G(i,j)==1) 
                    f_index = index_calculation2(j,i,o,d,n);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
                 
            %DESTINATION NODES    
            elseif (i==d)
                index_sum = [];
                f_row = zeros(1,total_var);
                for j=1:n
                  if(j~=d && G(i,j)==1) 
                    f_index = index_calculation2(j,i,o,d,n);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 1);     
            end
        end
    end
end

%GROOMING CONSTRAINTS
for i=1:n
    for j=i+1:n     
        f_row = zeros(1,total_var);
        for o=1:n
            for d=o+1:n
             index_sum = [];   
             %sum over fodij   
             bandwith = 1.25 * D(o,d,1) + 2.5 * D(o,d,2) + 10 * D(o,d,3) + 40 * D(o,d,4) + 100 * D(o,d,5);
             f_index1 = index_calculation2(i,j,o,d,n);
             f_index2 = index_calculation2(j,i,o,d,n);
             index_sum = [index_sum, f_index1, f_index2];
             f_row(index_sum)=bandwith;
            end
        end

       %variable W(i,j)
        W_index = index_calculation(i,j,n);
        W_index = var_f + W_index;
        f_row(W_index)=-100*G(i,j);
        mxlpsolve('add_constraint', ilp, f_row, 1, 0);      
    end
end

%LINK CAPACITY CONSTRAINT
for i=1:n
    for j=i+1:n     
        f_row = zeros(1,total_var);
        W_index = index_calculation(i,j,n);
        W_index = var_f + W_index;
        f_row(W_index)=1;
        mxlpsolve('add_constraint', ilp, f_row, 1, 80);          
    end
end


%BINARY VARIABLES DEFINITION
for i=1:var_f
    mxlpsolve('set_binary', ilp, i, 1);
end

%INTEGER VARIABLES DEFINITION
for i=1:var_W
    mxlpsolve('set_int', ilp, var_f+i, 1);
end

mxlpsolve('write_lp', ilp, 'opaque.lp');
mxlpsolve('solve', ilp);
obj = mxlpsolve('get_objective', ilp);
var = mxlpsolve('get_variables', ilp);


% SHOW RESULTS
fprintf('---------------------------------------------------');
fprintf('\nRESULTS\n');
fprintf('---------------------------------------------------\n');
fprintf('LINKS\n\n');
W_matrix = zeros(n,n);
for i=1:n
    for j=i+1:n
        if G(i,j)==1
            fprintf('Number of optical channels in link (%d,%d): %d\n', i, j, var(var_f+index_calculation(i,j,n)));
            W_matrix(i,j) = var(var_f+index_calculation(i,j,n));
            W_matrix(j,i) = W_matrix(i,j);
        end
    end
end
fprintf('\n');

fprintf('---------------------------------------------------\n');

fprintf('NODES\n\n');
for i=1:n
    aux_trib_ports(i,1) = i;
    for j=1:5
        aux_trib_ports(i,j+1) = sum(D(i,:,j));
    end
end
Tributary_Ports = array2table(aux_trib_ports,'VariableNames',{'Node' 'ODU0' 'ODU1' 'ODU2' 'ODU3' 'ODU4'})

for i=1:n
    aux_thr_ports(i,1) = i;
    aux_thr_ports(i,2) = sum(W_matrix(i,:));
end
Line_Ports = array2table(aux_thr_ports,'VariableNames',{'Node' 'OTU4'})


fprintf('---------------------------------------------------\n');

fprintf('\n');
fprintf('PATHS\n');
for i=1:n
    for j=i+1:n
        if G(i,j)==1
            fprintf('Link (%d,%d)---------------------------------\n', i, j);
            for o=1:n
                for d=o+1:n
                    if var(index_calculation2(i,j,o,d,n)) ~= 0
                        fprintf('Path between node (%d,%d)\n',o,d);
                    elseif var(index_calculation2(j,i,o,d,n)) ~= 0
                        fprintf('Path between node (%d,%d)\n',o,d);
                    end
                end
            end
            
        end
    end
end
