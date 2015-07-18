  function pattern = insertSqPattern(curDir,seq,levels)

%This code generates the pattern for a square spiral
%given the current dir of 'S', seq = 'NE', levels specified in linearSpiral
%This function will be called by linearSpiral and 
%returns a string of coordinate directions.
%e.g. insertSqPattern('S','NE',3)
%Use a helper called findNextDirSq(curDir,change).
%05/29/15

%PATTERN 
% 1 = NE
% 2 = SX2 WX3
% 3 = NX3 EX3
% 4 = SX4 WX4
% 5 = NX5 EX5
% 6 = SX6 WX6

pattern = seq;
currentLevel = 2;%default at 2, initPattern already exist
change = 0; %change direction when change = 1;
dirCount =2; %number of states in a level;

while currentLevel <= levels
    %for loop represents 1 level
    for i=1:currentLevel
        %calls helper function findNextDirSq
        %to calculate the next String dir given
        %if change = 1 then rotate, otherwise repeat
        %repeat the current direction
        nextDir = findNextDirSq(curDir,change);
        %appends the nextDir into pattern
        pattern = [pattern,nextDir];     
    end
    %sets the current Dir = previous dir
    curDir = nextDir;
    change = 1; %change every level
    %decrement #different directions count,e.g. NE = 2 dirCount
    dirCount = dirCount -1; 
    if dirCount == 0 
       dirCount = 2;
       %increment level when complete with n steps for that level
       %i.e. level2 SSWW, dirCount = 2
       currentLevel = currentLevel+1;
    end
    %resets change status.
    if change == 0
    change =1;
    end    
end

end
