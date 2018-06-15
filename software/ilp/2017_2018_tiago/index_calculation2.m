function[index2] = index_calculation2(i,j,o,d,n)

    block_index = index_calculation(o,d,n);
    starting_point = (block_index - 1) * (n*(n-1));
    
    if(j<i)    
        index = ((i-1)*(n-1) + (j-1)) + 1; 
    else
        index = ((i-1)*(n-1) + (j-1) - 1) + 1; 
    end    
    
    index2 = starting_point + index;   
end