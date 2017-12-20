clear all
clc

% Notation
% o - demand origin node
% d - demand destination node
% i - link origin node
% j - link destination node
% G - topology adjancency matrix
% D - demand matrix in tributary bit-rates (1.25, 2.5, 10, 40, 100 Gbps)
% f^{od}_{ij} - binary variable indicating if link ij is used to suppor od
% W_{ij} - number of bidirectional optical channel between node ij

% Inputs

G=[0,1,0,1;1,0,1,0;0,1,0,1;1,0,1,0];                % adjacency matrix, G_{ij} defines the network physical topology

% client traffic demands in the form of a 3-dimensional matrix D_{odc}
D(:,:,1)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];         % ODU0 matrix, 1.25 Gbps
D(:,:,2)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU1 matrix, 2.5 Gbps
D(:,:,3)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU2 matrix, 10 Gbps
D(:,:,4)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU3 matrix, 40 Gbps
D(:,:,5)=[0,1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];         % ODU4 matrix, 100 Gbps

% G=[0,1,1,0,0,0;1,0,1,1,0,0;1,1,0,0,1,0;0,1,0,0,1,1;0,0,1,1,0,1;0,0,0,1,1,0];
% 
% D(:,:,1)=[0,5,1,3,1,3;5,0,0,1,5,0;1,0,0,1,4,1;3,1,1,0,1,1;1,5,4,1,0,3;3,0,1,1,3,0];
% D(:,:,2)=[0,2,4,2,0,5;2,0,0,3,1,1;4,0,0,1,1,0;2,3,1,0,1,3;0,1,1,1,0,1;5,1,0,3,1,0];
% D(:,:,3)=[0,1,1,1,0,0;1,0,0,0,1,0;1,0,0,1,1,0;1,0,1,0,1,0;0,1,1,1,0,1;0,0,0,0,1,0];
% D(:,:,4)=[0,0,0,0,0,0;0,0,1,0,0,1;0,1,0,0,1,0;0,0,0,0,0,0;0,0,1,0,0,0;0,1,0,0,0,0];
% D(:,:,5)=[0,0,0,0,0,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,1;0,1,0,0,1,0];

N=length(G);                                % number of nodes

number_flows = (N * (N-1))/2;               % number of possible bidirectional demands, i.e. number of possible od pairs

var_f = number_flows * (N * (N-1));         % number f^{od}_{ij} variables (unideritonal)
var_W = number_flows;                       % number W_{ij} variables (bidirectional)

% total number of variables
total_var = var_f + var_W;

% this function is going to create a new ILP with total_var variables
ilp=mxlpsolve('make_lp', 0, total_var);

%OBJECTIVE FUNCTION
f_row = ones(1,total_var);
mxlpsolve('set_obj_fn', ilp, f_row);

%CONSTRAINTS
%FLOW CONSERVATION CONSTRAINTS
for o=1:N   
    for d=o+1:N
        for i=1:N 
            %ORIGIN NODES
            if(i==o)
                %sum over all i
                index_sum = [];
                f_row = zeros(1,total_var);
                for j=1:N
                  if(j~=o && G(i,j)==1)
                    f_index = index_calculation2(i,j,o,d,N);
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
                for j=1:N
                  if(j~=o && G(i,j)==1) 
                    f_index = index_calculation2(i,j,o,d,N);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=1;
                
                %outcome flow   
                index_sum = [];
                for j=1:N
                  if(j~=d && G(i,j)==1) 
                    f_index = index_calculation2(j,i,o,d,N);
                    index_sum = [index_sum, f_index];
                  end
                end
                f_row(index_sum)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
                 
            %DESTINATION NODES    
            elseif (i==d)
                index_sum = [];
                f_row = zeros(1,total_var);
                for j=1:N
                  if(j~=d && G(i,j)==1) 
                    f_index = index_calculation2(j,i,o,d,N);
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
for i=1:N
    for j=i+1:N     
        f_row = zeros(1,total_var);
        for o=1:N
            for d=o+1:N
             index_sum = [];   
             %sum over fodij   
             bandwidth = 1.25 * D(o,d,1) + 2.5 * D(o,d,2) + 10 * D(o,d,3) + 40 * D(o,d,4) + 100 * D(o,d,5);
             f_index1 = index_calculation2(i,j,o,d,N);
             f_index2 = index_calculation2(j,i,o,d,N);
             index_sum = [index_sum, f_index1, f_index2];
             f_row(index_sum)=bandwidth;
            end
        end

       %variable W(i,j)
        W_index = index_calculation(i,j,N);
        W_index = var_f + W_index;
        f_row(W_index)=-100*G(i,j);
        mxlpsolve('add_constraint', ilp, f_row, 1, 0);      
    end
end

%LINK CAPACITY CONSTRAINT
for i=1:N
    for j=i+1:N     
        f_row = zeros(1,total_var);
        W_index = index_calculation(i,j,N);
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
W_matrix = zeros(N,N);
for i=1:N
    for j=i+1:N
        if G(i,j)==1
            fprintf('Number of optical channels in link (%d,%d): %d\n', i, j, var(var_f+index_calculation(i,j,N)));
            W_matrix(i,j) = var(var_f+index_calculation(i,j,N));
            W_matrix(j,i) = W_matrix(i,j);
        end
    end
end
fprintf('\n');

fprintf('---------------------------------------------------\n');
fprintf('NODES\n\n');
for i=1:N
    aux_trib_ports(i,1) = i;
    for j=1:5
        aux_trib_ports(i,j+1) = sum(D(i,:,j));
    end
end
Tributary_Ports = array2table(aux_trib_ports,'VariableNames',{'Node' 'ODU0' 'ODU1' 'ODU2' 'ODU3' 'ODU4'})

for i=1:N
    aux_thr_ports(i,1) = i;
    aux_thr_ports(i,2) = sum(W_matrix(i,:));
end
Line_Ports = array2table(aux_thr_ports,'VariableNames',{'Node' 'OTU4'})

fprintf('---------------------------------------------------\n');
fprintf('PATHS\n');
fprintf('---------------------------------------------------\n');
for o=1:N
   for d=o+1:N
       bandwidth = 1.25 * D(o,d,1) + 2.5 * D(o,d,2) + 10 * D(o,d,3) + 40 * D(o,d,4) + 100 * D(o,d,5);
       if bandwidth~=0
            fprintf('Bidirectional demands between (%d,%d) (%6.2f Gbps)--\n', o, d, bandwidth);
            fprintf('---------------------------------------------------\n');
            fprintf('Working Path\n');
            for i=1:N
                for j=i+1:N
                    if G(i,j)==1
                        if var(index_calculation2(i,j,o,d,N)) ~= 0
                            fprintf('Link (%d,%d)\n',i,j);
                        elseif var(index_calculation2(j,i,o,d,N)) ~= 0
                            fprintf('Link (%d,%d)\n',j,i);
                        end
                    end
                end
            end
            fprintf('---------------------------------------------------\n');
       end
   end
end
