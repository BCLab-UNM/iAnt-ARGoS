%This function is a helper function for linearSpiral.
%It takes in a 2x1 vector of coordinates and a inital
%string of pattern i.e. parsingPattern([0,0], 'NE')
%This function parses the inital pattern and generates
%the x&y coordinates for the pattern.
%Returns a vector comprised of the x,y coordinates.
%e.g. parsingPattern([0 0], 'NE')
%05/29/15
function [x,y] = parsingPattern(newPos,pattern)
    for i = 1:size(pattern,2)  
       
       oldPos = newPos;
       %calls helper fuction update position to 
       %calculate the next move
       newPos = updatePosition(newPos,pattern(i));
       
       %stores x,y coord in vectors
       x(i) = newPos(1);
       y(i) = newPos(2);   
   end
end
