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

G=[0,1,0,1;1,0,1,0;0,1,0,1;1,0,1,0];          % adjacency matrix, G_{ij} defines the network physical topology

% client traffic demands in the form of a 3-dimensional matrix D_{odc}
D(:,:,1)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
D(:,:,2)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
D(:,:,3)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
D(:,:,4)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];
D(:,:,5)=[0,1,1,1;1,0,1,1;1,1,0,1;1,1,1,0];

%G=[0,1,1,0,0,0;1,0,1,1,0,0;1,1,0,0,1,0;0,1,0,0,1,1;0,0,1,1,0,1;0,0,0,1,1,0];

% D(:,:,1)=[0,5,1,3,1,3;5,0,0,1,5,0;1,0,0,1,4,1;3,1,1,0,1,1;1,5,4,1,0,3;3,0,1,1,3,0];
% D(:,:,2)=[0,2,4,2,0,5;2,0,0,3,1,1;4,0,0,1,1,0;2,3,1,0,1,3;0,1,1,1,0,1;5,1,0,3,1,0];
% D(:,:,3)=[0,1,1,1,0,0;1,0,0,0,1,0;1,0,0,1,1,0;1,0,1,0,1,0;0,1,1,1,0,1;0,0,0,0,1,0];
% D(:,:,4)=[0,0,0,0,0,0;0,0,1,0,0,1;0,1,0,0,1,0;0,0,0,0,0,0;0,0,1,0,0,0;0,1,0,0,0,0];
% D(:,:,5)=[0,0,0,0,0,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,1;0,1,0,0,1,0];

DD = 1.25*D(:,:,1) + 2.5*D(:,:,2) + 10.0*D(:,:,3) + 40.0*D(:,:,4) + 100.0*D(:,:,5);

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
%GROOMING CONSTRAINTS
for o=1:N
    for d=o+1:N
        index_sum = [];      
        f_row = zeros(1,total_var);
        bandwith = DD(o,d);
        %variable W(o,d)
        W_index = index_calculation(o,d,N);
        W_index = var_f + W_index;
        f_row(W_index)=100;
        mxlpsolve('add_constraint', ilp, f_row, 2, bandwith);
    end
end
        
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
                %variable W(o,d)
                W_index = index_calculation(o,d,N);
                W_index = var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
            
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
                %variable W(o,d)
                W_index = index_calculation(o,d,N);
                W_index = var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);     
            end
        end
    end
end

%CAPACITY CONSTRAINTS
for i=1:N
    for j=i+1:N    
        index_sum = [];
        f_row = zeros(1,total_var);
        for o=1:N
            for d=o+1:N 
             f_index1 = index_calculation2(i,j,o,d,N);
             f_index2 = index_calculation2(j,i,o,d,N);
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


path_matrix = zeros(N,N);
for i=1:N
    for j=1:N
        if i ~= j
            for o=1:N
                for d=o+1:N
                    if var(index_calculation2(i,j,o,d,N))>0
                        path_matrix(i,j) = path_matrix(i,j) + var(index_calculation2(i,j,o,d,N));     
                        path_matrix(j,i) = path_matrix(i,j);
                    end
                end        
            end 
        end
    end
end

OCh_matrix = zeros(N,N);
for i=1:N
    for j=i+1:N
        OCh_matrix(i,j) = var(var_f+index_calculation(i,j,N));
        OCh_matrix(j,i) = OCh_matrix(i,j);
    end
end


% SHOW RESULTS
fprintf('---------------------------------------------------');
fprintf('\nRESULTS\n');
fprintf('---------------------------------------------------\n\n');

fprintf('---------------------------------------------------\n');
fprintf('LINKS\n');
fprintf('---------------------------------------------------\n\n');

fprintf('---------------------------------------------------\n');
for o=1:N
    for d=o+1:N
        fprintf('Number of lightpaths between node (%d,%d): %d\n', o, d, var(var_f+index_calculation(o,d,N)));
    end
end
fprintf('---------------------------------------------------\n');
fprintf('\n');
fprintf('---------------------------------------------------\n');
for i=1:N
    for j=i+1:N
        if G(i,j)==1
           aux = 0;
           for o=1:N
               for d=o+1:N
                   w = var(index_calculation2(i,j,o,d,N));
                   if w==0
                       w = var(index_calculation2(j,i,d,o,N));
                   end
                   aux = aux + w;
               end
           end
           fprintf('Number of optical channels in link (%d,%d): %d\n', i, j, aux);
        end
     end
end
fprintf('---------------------------------------------------\n\n');

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
fprintf('---------------------------------------------------\n\n');
fprintf('---------------------------------------------------\n');
fprintf('NODES\n');
fprintf('---------------------------------------------------\n');
for i=1:N
    aux_trib_ports(i,1) = i;
    for j=1:5
        aux_trib_ports(i,j+1) = sum(D(i,:,j));
    end
end
Tributary_Ports = array2table(aux_trib_ports,'VariableNames',{'Node' 'ODU0' 'ODU1' 'ODU2' 'ODU3' 'ODU4'})

for i=1:N
    aux_trib2_ports(i,1) = i;
    aux_trib2_ports(i,2) = sum(OCh_matrix(i,:));
end
Add_Ports = array2table(aux_trib2_ports,'VariableNames',{'Node' 'ODU4'})

for i=1:N
    aux_trgh_ports(i,1) = i;
    aux_trgh_ports(i,2) = sum(path_matrix(i,:));
end
Line_Ports = array2table(aux_trgh_ports,'VariableNames',{'Node' 'ODU4'})




