clear all
clc

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
total_var = 2*var_f+var_W;

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


%Protection Segments
%fp^od_ij

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
                mxlpsolve('add_constraint', ilp, f_row, 3, 1);
            
             %INTERMEDIATE NODES
             elseif (i ~=o && i ~=d)
                index_sum = [];
                f_row = zeros(1,total_var);
                %income flow
                for j=1:n
                  if(j~=o && G(i,j)==1) 
                    fp_index = var_f+index_calculation2(i,j,o,d,n);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                
                %outcome flow   
                index_sum = [];
                for j=1:n
                  if(j~=d && G(i,j)==1) 
                    fp_index = var_f+index_calculation2(j,i,o,d,n);
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
                    fp_index = var_f+index_calculation2(j,i,o,d,n);
                    index_sum = [index_sum, fp_index];
                  end
                end
                f_row(index_sum)=1;
                mxlpsolve('add_constraint', ilp, f_row, 3, 1);     
            end
        end
    end
end


%%f^od_ij != fp^od_ij

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
                
               

%GROOMING CONSTRAINTS
for i=1:n
    for j=i+1:n     
        f_row = zeros(1,total_var);
        for o=1:n
            for d=o+1:n
             index_sum = [];   
             %sum over fodij   
             bandwith = DD(o,d);
             f_index1 = index_calculation2(i,j,o,d,n);
             f_index2 = index_calculation2(j,i,o,d,n);
             fp_index1 = var_f+index_calculation2(i,j,o,d,n);
             fp_index2 = var_f+index_calculation2(j,i,o,d,n);
             index_sum = [index_sum, f_index1, f_index2,fp_index1,fp_index2]
             f_row(index_sum)=bandwith;
            end
        end

       %variable W(i,j)
        W_index = index_calculation(i,j,n);
        W_index = 2*var_f + W_index;
        f_row(W_index)=-100*G(i,j);
        mxlpsolve('add_constraint', ilp, f_row, 1, 0);      
    end
end

%LINK CAPACITY CONSTRAINT
for i=1:n
    for j=i+1:n     
        f_row = zeros(1,total_var);
        W_index = index_calculation(i,j,n);
        W_index = 2*var_f + W_index;
        f_row(W_index)=1;
        mxlpsolve('add_constraint', ilp, f_row, 1, 80);          
    end
end


%BINARY VARIABLES DEFINITION
for i=1:2*var_f
    mxlpsolve('set_binary', ilp, i, 1);
end

%INTEGER VARIABLES DEFINITION
for i=1:var_W
    mxlpsolve('set_int', ilp, 2*var_f+i, 1);
end

mxlpsolve('write_lp', ilp, 'opaque.lp');
mxlpsolve('solve', ilp);
obj = mxlpsolve('get_objective', ilp);
var = mxlpsolve('get_variables', ilp);



% SHOW RESULTS
fprintf('\n\nRESULTS\n\n');
for o=1:n
    for d=o+1:n
        
        if(var(2*var_f+index_calculation(o,d,n))~=0)
        fprintf('Number of optical channels in the link (%d,%d): %d\n', o, d, var(2*var_f+index_calculation(o,d,n)));
        end
    end
end

fprintf('\n');
for o=1:n
    for d=o+1:n
        
         if(var(2*var_f+index_calculation(o,d,n))>=1)   
         fprintf('\n');
         fprintf('---------------Demand (%d,%d)---------------\n', o, d);
         fprintf('\n');
         fprintf('\n');
         i=o;
         j=1;
         temp=i;
         count=0;
         
         fprintf('------Working Path------\n');
         while(i~=d)
             
             
            if (G(i,j)==1 && var(var_f+index_calculation2(i,j,o,d,n))==1)
            fprintf('Link  (%d,%d)\n',i,j);
            count=count+1;
            temp=j;
            j=0;
            end 
            
         i=temp;
         j=j+1;
         end
         
         fprintf('Number of links: %d \n',count);
         fprintf('\n');
         count=0;
         i=o;
         j=1;
         temp=i;
         fprintf('-----Protection Path-----\n');
         
         while(i~=d)
             
             
            if (G(i,j)==1 && var(index_calculation2(i,j,o,d,n))==1)
            fprintf('Link  (%d,%d)\n',i,j);
             count=count+1;   
             temp=j;
             j=0;
            end 
         
         i=temp;
         j=j+1;
         end
         fprintf('Number of links: %d \n',count);
         fprintf('\n');
         
         end   
      end
       
 end          
