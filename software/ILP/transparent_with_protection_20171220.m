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
D(:,:,1)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU0 matrix, 1.25 Gbps
D(:,:,2)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU1 matrix, 2.5 Gbps
D(:,:,3)=[0,1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];         % ODU2 matrix, 10 Gbps
D(:,:,4)=[0,0,0,0;0,0,0,0;0,0,0,0;0,0,0,0];         % ODU3 matrix, 40 Gbps
D(:,:,5)=[0,1,0,0;1,0,0,0;0,0,0,0;0,0,0,0];         % ODU4 matrix, 100 Gbps

DD = 1.25*D(:,:,1) + 2.5*D(:,:,2) + 10.0*D(:,:,3) + 40.0*D(:,:,4) + 100.0*D(:,:,5);

N=length(G);                                % number of nodes

number_flows = (N * (N-1))/2;               % number of possible bidirectional demands, i.e. number of possible od pairs

var_f = number_flows * (N * (N-1));         % f^{od}_{ij}, number of lightpaths that cross the ij link to serve the od demand
var_W = number_flows;                       % W_{od} number of bidirectional lightpaths used to serve the od demand

% total number of variables
total_var = 2*var_f+2*var_W;                % the number of variables is double to considerer the protection

ilp=mxlpsolve('make_lp', 0, total_var);

%OBJECTIVE FUNCTION
f_row = ones(1,total_var);
mxlpsolve('set_obj_fn', ilp, f_row);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%              CONSTRAINTS              %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%GROOMING CONSTRAINTS

% Working lightpaths, w^{od}, 100 w^{od} \geq DD^{od}
for o=1:N
    for d=o+1:N
        index_sum = [];      
        f_row = zeros(1,total_var);
        W_index = index_calculation(o,d,N);
        W_index = 2*var_f + W_index;
        f_row(W_index)=100;
        mxlpsolve('add_constraint', ilp, f_row, 2, DD(o,d));
    end
end

% Protection lightpaths, w^{od,p}, 100 w^{od,p} \geq DD^{od}
for o=1:N
    for d=o+1:N
        index_sum = [];      
        f_row = zeros(1,total_var);
        Wp_index = index_calculation(o,d,N);
        Wp_index = 2*var_f + var_W + Wp_index;
        f_row(Wp_index)=100;
        mxlpsolve('add_constraint', ilp, f_row, 2, DD(o,d));
    end
end
        
%FLOW CONSERVATION CONSTRAINTS

% Working optical channels, f^{od}_{ij}
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
                W_index = 2*var_f + W_index;
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
                W_index = 2*var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);     
            end
        end
    end
end

%Protection

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
                    fp_index = var_f+index_calculation2(i,j,o,d,N);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                %variable W(o,d)
                Wp_index = index_calculation(o,d,N);
                Wp_index = 2*var_f + var_W + Wp_index;
                f_row(Wp_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
            
             %INTERMEDIATE NODES
             elseif (i ~=o && i ~=d)
                index_sum = [];
                f_row = zeros(1,total_var);
                %income flow
                for j=1:N
                  if(j~=o && G(i,j)==1) 
                    fp_index = var_f + index_calculation2(i,j,o,d,N);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                
                %outcome flow   
                index_sum = [];
                for j=1:N
                  if(j~=d && G(i,j)==1)
                    fp_index = var_f+ index_calculation2(j,i,o,d,N);
                    index_sum = [index_sum, fp_index];
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
                    fp_index = var_f + index_calculation2(j,i,o,d,N);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                %variable W(o,d)
                Wp_index = index_calculation(o,d,N);
                Wp_index = 2*var_f + var_W + Wp_index;
                f_row(Wp_index)=-1;
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
              fp_index1 = var_f+index_calculation2(i,j,o,d,N);
              fp_index2 = var_f+index_calculation2(j,i,o,d,N);
              index_sum = [index_sum, f_index1, f_index2, fp_index1, fp_index2];
            end
        end
        f_row(index_sum)=1;
        mxlpsolve('add_constraint', ilp, f_row, 1, 80*G(i,j));
    end
end



%f^od_ij != fp^od_ij, disjunts working and protection paths

for o=1:N  
    for d=o+1:N      
         for i=1:N     
             for j=i+1:N
                f_row = zeros(1,total_var);
                index_sum=[];
                f_index = index_calculation2(i,j,o,d,N);
                fp_index = var_f+index_calculation2(i,j,o,d,N);
                index_sum = [index_sum, f_index, fp_index];
                f_row(index_sum)=1;
                mxlpsolve('add_constraint', ilp, f_row, 1, ceil(DD(o,d)/100)); 
             end
         end 
    end 
end


%INTEGER VARIABLES DEFINITION
for i=1:total_var
    mxlpsolve('set_int', ilp, i, 1);
end

mxlpsolve('write_lp', ilp, 'transparent_with_protection.lp');
mxlpsolve('solve', ilp);
obj = mxlpsolve('get_objective', ilp);
var = mxlpsolve('get_variables', ilp);


% SHOW RESULTS
fprintf('---------------------------------------------------');
fprintf('\nRESULTS\n');
fprintf('---------------------------------------------------\n\n');

fprintf('---------------------------------------------------\n');
fprintf('LINKS\n');
fprintf('---------------------------------------------------\n\n');

fprintf('---------------------------------------------------\n');
for i=1:N
    for j=i+1:N
        fprintf('Number of lightpaths between nodes (%d,%d): %d\n', i, j, var(2*var_f+index_calculation(i,j,N))+var(2*var_f+var_W+index_calculation(i,j,N)));
    end
end

fprintf('---------------------------------------------------\n');
for i=1:N
    for j=i+1:N
        fprintf('Number of working lightpaths between nodes (%d,%d): %d\n', i, j, var(2*var_f+index_calculation(i,j,N)));
        fprintf('Number of protection lightpaths between nodes (%d,%d): %d\n', i, j, var(2*var_f+var_W+index_calculation(i,j,N)));
    end
end

fprintf('---------------------------------------------------\n');
fprintf('\n');
fprintf('---------------------------------------------------\n');
OCh_matrix = zeros(N,N);
for i=1:N
    for j=i+1:N
        if G(i,j)==1
            aux_w = 0;
            aux_p = 0;
            for o=1:N
                for d=o+1:N
                    aux_w = aux_w + var(index_calculation2(i,j,o,d,N)) + var(index_calculation2(j,i,d,o,N));
                    aux_p = aux_p + var(var_f+index_calculation2(i,j,o,d,N)) + var(var_f+index_calculation2(j,i,d,o,N));
                end
            end
            OCh_matrix(i,j) = aux_w+aux_p;
            OCh_matrix(j,i) = OCh_matrix(i,j);
            fprintf('Number of optical channels in link (%d,%d): %d\n', i, j, OCh_matrix(i,j));
        end
    end
end
fprintf('---------------------------------------------------\n');
for i=1:N
    for j=i+1:N
        if G(i,j)==1
            aux_w = 0;
            aux_p = 0;
            for o=1:N
                for d=o+1:N
                    aux_w = aux_w + var(index_calculation2(i,j,o,d,N)) + var(index_calculation2(j,i,d,o,N));
                    aux_p = aux_p + var(var_f+index_calculation2(i,j,o,d,N)) + var(var_f+index_calculation2(j,i,d,o,N));
                end
            end
            fprintf('Number of working optical channels in link (%d,%d): %d\n', i, j, aux_w);
            fprintf('Number of protection optical channels in link (%d,%d): %d\n', i, j, aux_p);
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
    aux_trib2_ports(i,2) = sum(ceil(DD(i,:)/100));
end
Add_Ports = array2table(aux_trib2_ports,'VariableNames',{'Node' 'ODU4'})

for i=1:N
    aux_line_ports(i,1) = i;
    aux_line_ports(i,2) = sum(OCh_matrix(i,:));
end
Line_Ports = array2table(aux_line_ports,'VariableNames',{'Node' 'ODU4'})

