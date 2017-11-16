clear all
%inputs

G=[0,1,1,0,0,0;1,0,1,1,0,0;1,1,0,0,1,0;0,1,0,0,1,1;0,0,1,1,0,1;0,0,0,1,1,0];

D(:,:,1)=[0,5,1,3,1,3;5,0,0,1,5,0;1,0,0,1,4,1;3,1,1,0,1,1;1,5,4,1,0,3;3,0,1,1,3,0];
D(:,:,2)=[0,2,4,2,0,5;2,0,0,3,1,1;4,0,0,1,1,0;2,3,1,0,1,3;0,1,1,1,0,1;5,1,0,3,1,0];
D(:,:,3)=[0,1,1,1,0,0;1,0,0,0,1,0;1,0,0,1,1,0;1,0,1,0,1,0;0,1,1,1,0,1;0,0,0,0,1,0];
D(:,:,4)=[0,0,0,0,0,0;0,0,1,0,0,1;0,1,0,0,1,0;0,0,0,0,0,0;0,0,1,0,0,0;0,1,0,0,0,0];
D(:,:,5)=[0,0,0,0,0,0;0,0,0,0,0,1;0,0,0,0,0,0;0,0,0,0,0,0;0,0,0,0,0,1;0,1,0,0,1,0];

n=6;

%Variables
range V 1:n;
range O 1:5;
range B 1:1;

int traffic[O,V,V];
int Granularities[O];
int C[V,V];
int BD[B];

dvar l[V,V,B];
dvar L[V,V,O,V,V];
dvar y[V,V,V,V,B];

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

%Routing of the services in the virtual topology

for s=1:V
    for d=s+1:V
        for o=1:O
            index_sum = [];
            for K=1:V
                if(k ~= s)
                   L[s,d,o,s,k] = traffic[o,s,d];
                   index_sum = L[s,d,o,s,k];
                end                
            end
        end
    end
end

for s=1:V
    for d=s+1:V
        for p=1:V
            for o=1:O
                if(p~=s && p~=d)
                    index_sum = [];
                    for K=1:V
                        if(k ~= p && k~=s)
                           L[s,d,o,s,k] = traffic[o,s,d];
                           index_sum = L[s,d,o,s,k];
                        end                
                    end
                end
            end
        end
    end
end

for s=1:V
    for d=s+1:V
        for o=1:O
            index_sum = [];
            for K=1:V
                if(k ~= d)
                   L[s,d,o,k,d] = traffic[o,s,d];
                   index_sum = L[s,d,o,k,d];
                end                
            end
        end
    end
end

for p=1:V
    for k=1:V
        
    end
end

for s=1:V
    for d=s+1:V
        for p=1:V
            for k=1:V
                for o=1:O
                    ge(L[s,d,o,p,k],0);
                end
            end
        end
    end
end

%Routing of the lightpaths

