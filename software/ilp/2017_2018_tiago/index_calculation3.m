function[index3] = index_calculation3(i,j,o,d,c,n)

    block_point = (c-1) * (n * (n-1))/2 * (n * (n-1));
    
    block_index = index_calculation(o,d,n);
    starting_point = (block_index - 1) * (n*(n-1));
    
    if(j<i)    
        index = ((i-1)*(n-1) + (j-1)) + 1; 
    else
        index = ((i-1)*(n-1) + (j-1) - 1) + 1; 
    end    
    
    index3 = block_point + starting_point + index;   
end