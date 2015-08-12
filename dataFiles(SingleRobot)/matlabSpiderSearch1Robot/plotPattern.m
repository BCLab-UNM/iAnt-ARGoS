%Helper function that plots the pattern using quiver.
%It takes in 2 vectors.
%e.g. plot(x,y)
%05/29/15
function plotPattern(a,b)
    x = a;
    y = b;
    u = []; 
    
    %Looks through the x,y,u,v vectors
    %for quiver to plot.
    
    for i=1:size(x,2)-1
        j=i+1;
        x1 = x(i);
        x2 = x(j);
        %u = change in x
        u(i) = x2-x1;
         
        y1 = y(i);
        y2 = y(j);
        %v = change in y
        v(i) = y2-y1;
    end
        %appends 0 to the end to prevent from having
        %different sized vectors
        u = [u,0];
        v = [v,0];       
        quiver(x,y,u,v);
end   
        
       