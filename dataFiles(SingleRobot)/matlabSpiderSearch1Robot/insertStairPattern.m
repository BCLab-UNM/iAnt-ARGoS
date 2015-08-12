%This code generates the pattern for a diamond shaped spiral
%Given the initial string pattern and n int of levels 
%This function will be called by linearSpiral and 
%returns a string pattern.
%e.g. insertStairPattern('NSESWSNWN',3)
%Uses a helper function called patternHelper().
%05/29/15
function pattern = insertStairPattern(seq,n)

%PATTERN 
% 1 = N SE SW SN WN

% 2 = W NEx2 NS ESx2
% 3 = E SWx3 SN WNX3
% 4 = W NEX4 NS ESX4
% 5 = E SWX5 SN WNX5
% 6 = W NEX6 NS ESX6

%total steps = 2n^2+5n+2
%%%%%%%%%%%%%%%%%%%%
% 3 = E SEx3 SN WNX3
% 5 = E SEX5 SN WNX5
% 2 = W NEx2 NS ESx2 
% 4 = W NEX4 NS ESX4 
% 6 = W NEX6 NS ESX6
%separated by odd and even pattern

pattern = seq; %init pattern = 'NSESWSNWN'
% totalSteps = 2*(n.^2)+5*n+2;

% correponds to the # of times each partial sequence repeats
% e.g. NEx2
partial = [];
for x=2:n
    totalLevelSteps = 4*x+3; %total # of steps in that level
    %partial stores the pattern generated starting from level 2 till n
    %levels. this calls the patternHelper to generate the pattern for n
    %levels
    partial = [partial,patternHelper(mod(x,2),x,totalLevelSteps)];
end
    
% end
%appends partial levels to the initial pattern
pattern = [pattern,partial];
end

