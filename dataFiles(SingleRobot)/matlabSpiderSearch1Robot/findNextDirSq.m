%This function is helper for the insertSqPattern
%It finds the next direction in the pattern
%The direction is determined by the a string curDir and int change = 0 or 1
%It returns the string newDir
%05/29/15
function newDir = findNextDirSq(curDir,change)

%if change =1, then rotate clockwise
if change == 1
    switch curDir
        case 'N'
            newDir = 'E';
        case 'E'
            newDir = 'S';
        case 'S'
            newDir = 'W';
        otherwise
            newDir = 'N';
    end 
%else repeat curDir, n level of times
else
   newDir = curDir;
end
end