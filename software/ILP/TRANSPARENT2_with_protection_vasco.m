clear all
%inputs

% Network Topology
G=[0,1,0,1; 1,0,1,0; 0,1,0,1; 1,0,1,0];

% Demands
D(:,:,1)=[0,1,0,0; 1,0,0,0; 0,0,0,0; 0,0,0,0];

DD = 10 * D(:,:,1);
[n n]=size(G);

%ILP
number_flows = (n * (n-1))/2;

var_f = number_flows * (n * (n-1));
var_W = number_flows;
total_var = 2*var_f+2*var_W;

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
        bandwith = DD(o,d);
        %variable W(o,d)
        W_index = index_calculation(o,d,n);
        W_index = 2*var_f + W_index;
        f_row(W_index)=100;
        mxlpsolve('add_constraint', ilp, f_row, 2, bandwith);
    end
end



for o=1:n
    for d=o+1:n
        index_sum = [];      
        f_row = zeros(1,total_var);
        bandwith = DD(o,d);
        %variable W(o,d)
        Wp_index = index_calculation(o,d,n);
        Wp_index = 2*var_f + var_W + Wp_index;
        f_row(Wp_index)=100;
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
                W_index = 2*var_f + W_index;
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
                W_index = 2*var_f + W_index;
                f_row(W_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);     
            end
        end
    end
end

%%Protection

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
                    fp_index = var_f+index_calculation2(i,j,o,d,n);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                %variable W(o,d)
                Wp_index = index_calculation(o,d,n);
                Wp_index = 2*var_f + var_W + Wp_index;
                f_row(Wp_index)=-1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 0);
            
             %INTERMEDIATE NODES
             elseif (i ~=o && i ~=d)
                index_sum = [];
                f_row = zeros(1,total_var);
                %income flow
                for j=1:n
                  if(j~=o && G(i,j)==1) 
                    fp_index = var_f + index_calculation2(i,j,o,d,n);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                
                %outcome flow   
                index_sum = [];
                for j=1:n
                  if(j~=d && G(i,j)==1)
                    fp_index = var_f+ index_calculation2(j,i,o,d,n);
                    index_sum = [index_sum, fp_index];
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
                    fp_index = var_f + index_calculation2(j,i,o,d,n);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                %variable W(o,d)
                Wp_index = index_calculation(o,d,n);
                Wp_index = 2*var_f + var_W + Wp_index;
                f_row(Wp_index)=-1;
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
             fp_index1 = var_f+index_calculation2(i,j,o,d,n);
             fp_index2 = var_f+index_calculation2(j,i,o,d,n);
             index_sum = [index_sum, f_index1, f_index2, fp_index1, fp_index2];
             end
        end
        f_row(index_sum)=1;
        mxlpsolve('add_constraint', ilp, f_row, 1, 80*G(i,j));
    end
end



%f^od_ij != fp^od_ij

for o=1:n  
    for d=o+1:n      
         for i=1:n     
             for j=i+1:n

             f_row = zeros(1,total_var);
             index_sum=[];
             
             f_index = index_calculation2(i,j,o,d,n);
             fp_index = var_f+index_calculation2(i,j,o,d,n);
             index_sum = [index_sum,f_index, fp_index];
               
                
            
             f_row(index_sum)=1;
             mxlpsolve('add_constraint', ilp, f_row, 1, 1);
            end
            
        end
           
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


% SHOW RESULTS
fprintf('\n\nRESULTS\n\n');
for i=1:n
    for j=i+1:n
        fprintf('Number of lightpaths between nodes (%d,%d): %d\n', i, j, var(2*var_f+index_calculation(i,j,n)) + var(2*var_f+var_W+index_calculation(i,j,n)));
    end
end

fprintf('\n');
for i=1:n
    for j=i+1:n
        if G(i,j)==1
            aux = 0;
            for o=1:n
                for d=o+1:n
                    aux = aux + var(index_calculation2(i,j,o,d,n)) + var(var_f+index_calculation2(i,j,o,d,n)) + var(index_calculation2(j,i,o,d,n)) + var(var_f+index_calculation2(j,i,o,d,n));
                end
            end
            fprintf('Number of wavelengths in bidirectional link (%d,%d): %d\n', i, j, aux);
        end
    end
end
