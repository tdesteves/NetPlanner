clear all
%inputs
% G=[0,1,1,0;1,0,0,1;1,0,0,1;0,1,1,0];
% D(:,:,1)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,2)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,3)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,4)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% D(:,:,5)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
% n=4;

G=[0,1,1,0,0,0;1,0,1,1,0,0;1,1,0,0,1,0;0,1,0,0,1,1;0,0,1,1,0,1;0,0,0,1,1,0];

D(:,:,1)=[0,25,5,15,5,15;25,0,0,5,25,0;5,0,0,5,20,5;15,5,5,0,5,5;5,25,20,5,0,15;15,0,5,5,15,0];
D(:,:,2)=[0,10,20,10,0,25;10,0,0,15,5,5;20,0,0,5,5,0;10,15,5,0,5,15;0,5,5,5,0,5;25,5,0,15,5,0];
D(:,:,3)=[0,5,5,5,0,0;5,0,0,0,5,0;5,0,0,5,5,0;5,0,5,0,5,0;0,5,5,5,0,5;0,0,0,0,5,0];
D(:,:,4)=[0,0,0,0,0,0;0,0,5,0,0,5;0,5,0,0,5,0;0,0,0,0,0,0;0,0,5,0,0,0;0,5,0,0,0,0];
D(:,:,5)=[0,0,0,0,0,0;0,0,0,0,0,5;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,5;0,5,0,0,5,0];
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
%GROOMING CONSTRAINTS
for o=1:n
    for d=o+1:n
        index_sum = [];      
        f_row = zeros(1,total_var);
        bandwith = 1.25 * D(o,d,1) + 2.5 * D(o,d,2) + 10 * D(o,d,3) + 40 * D(o,d,4) + 100 * D(o,d,5);
        %variable W(o,d)
        W_index = index_calculation(o,d,n);
        W_index = var_f + W_index;
        f_row(W_index)=100;
        mxlpsolve('add_constraint', ilp, f_row, 2, bandwith);
    end
end
        
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
                %variable W(o,d)
                W_index = index_calculation(o,d,n);
                W_index = var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
            
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
                %variable W(o,d)
                W_index = index_calculation(o,d,n);
                W_index = var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);     
            end
        end
    end
end

%CAPACITY CONSTRAINTS
for i=1:n
    for j=i+1:n    
        index_sum = [];
        f_row = zeros(1,total_var);
        for o=1:n
            for d=o+1:n 
             f_index1 = index_calculation2(i,j,o,d,n);
             f_index2 = index_calculation2(j,i,o,d,n);
             index_sum = [index_sum, f_index1, f_index2];
             end
        end
        f_row(index_sum)=1;
        mxlpsolve('add_constraint', ilp, f_row, 1, 80*G(i,j));
    end
end

%INTEGER VARIABLES DEFINITION
for i=1:total_var
    mxlpsolve('set_int', ilp, i, 1);
end

mxlpsolve('write_lp', ilp, 'transparent.lp');
mxlpsolve('solve', ilp);
obj = mxlpsolve('get_objective', ilp);
var = mxlpsolve('get_variables', ilp);


path_matrix = zeros(n,n);
for i=1:n
    for j=1:n
        if i ~= j
            for o=1:n
                for d=o+1:n
                    if var(index_calculation2(i,j,o,d,n))>0
                        path_matrix(i,j) = path_matrix(i,j) + var(index_calculation2(i,j,o,d,n));     
                        path_matrix(j,i) = path_matrix(i,j);
                    end
                end        
            end 
        end
    end
end

OCh_matrix = zeros(n,n);
for i=1:n
    for j=i+1:n
        OCh_matrix(i,j) = var(var_f+index_calculation(i,j,n));
        OCh_matrix(j,i) = OCh_matrix(i,j);
    end
end


% % SHOW RESULTS
fprintf('\n\nRESULTS\n\n');

fprintf('LINKS\n\n');
for i=1:n
    for j=i+1:n
        if G(i,j)==1
            fprintf('Number of optical channels in link (%d,%d): %d\n', i, j, path_matrix(i,j));
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
    aux_trib2_ports(i,1) = i;
    aux_trib2_ports(i,2) = sum(OCh_matrix(i,:));
end
Line_Ports = array2table(aux_trib2_ports,'VariableNames',{'Node' 'ODU4'})

for i=1:n
    aux_trgh_ports(i,1) = i;
    aux_trgh_ports(i,2) = sum(path_matrix(i,:));
end
Through_Ports = array2table(aux_trgh_ports,'VariableNames',{'Node' 'ODU4'})


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