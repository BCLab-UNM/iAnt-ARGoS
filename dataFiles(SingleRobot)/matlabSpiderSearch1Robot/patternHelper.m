%This function is a helper to insertStairPattern
%It generates the pattern for n levels. It is called
%repeatedly inside a for loop loop.
%It returns the a string of directions.
%05/29/15
function string = stringHelper(levelState,repeat,totalLevelSteps)
%levelState is mod(x,2). Indicates whether, the current level is odd or
%even
%repeat is used in a subHelper method called repeatPartial,that dictates
%wether the partial e.g. 'NE is repeated n times.
%totalLevelSteps is 4x+3 numbers of total steps for each level

%There are 4 states for each level,determines which string is appended to
%the string.
state = 1;  
string = [];
while size(string,2)< totalLevelSteps
%   resets state to one
    if state == 5
        state = 1;
    end
    switch state
        case 1
           if levelState == 0 %odd
              string = [string, 'W'];
           else %even
              string = [string, 'E'];
           end
        case 2
            if levelState == 0
                partial = 'NE';
                
                %call helper repeatPartial 'NE', repeat times
                string = repeatPartial(string,partial,repeat);
            else
                partial = 'SW';
                string = repeatPartial(string,partial,repeat);
            end
        case 3
            if levelState == 0
                string = [string,'NS'];
            else
                string = [string,'SN'];
            end
        case 4
            if levelState == 0
                partial = 'ES';
                string = repeatPartial(string,partial,repeat);
            else
                partial = 'WN';
                string = repeatPartial(string,partial,repeat);
            end
    end
    %increment state
    state = state+1;
end
    
end

%This function is a local helper for stringHelper
%it is called inside the switch conditions
%it returns a string of pattern with the duplications
%appended.
%It takes in the current string, partial that going to
%to be appended and repeat an int number of times
function string = repeatPartial(string,partial,repeat)
    for i=1:repeat
        string = [string,partial];
    end
end