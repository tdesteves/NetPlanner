function[index] = index_calculation(i,j,n)
    if(j>i)    
        index = ((i-1)*(2*n - (i-1) - 1))/2 + ((j-1) - (i-1) - 1) + 1; 
    else
        index = ((j-1)*(2*n - (j-1) - 1))/2 + ((i-1) - (j-1) - 1) + 1; 
    end
end