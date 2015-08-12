%This Fuction takes in 2 ints for # of levels within spiral & 
%the spiral expected to run. 
%e.g. linearSpiral(6,1) or linearSpiral(6,0)
%The result returned is the pattern being ploted.
%This function uses three helper functions entitled:
%parsingPattern, insertSqPattern or insertStairPattern, and plot pattern.
%There is a total of 5 helper functions called in this function.
%This code produces 2 spirals, the square and diamond for a single robot.
%05/29/15
function pattern = linearSpiral(levels,run)
    initPos = [0 0];

    %run = 1, then draw square sprial
    %run = 0, then draw diamond shape spiral
    if run == 1
        initDir = 'S';
        initPattern = 'N,E,:';
        pattern=insertSqPattern2(initDir,initPattern,levels);
    else
        initPattern = 'NSESWSNWN';
        pattern = insertStairPattern(initPattern,levels);
    end
    
    %parses the pattern into coordinates
    [x,y] = parsingPattern(initPos,pattern);
    
    a = [x,y]; %stores x,y coord in one vector
    stepsTot = size(a)/2;%splits the a vector in half btwn x&y
    
    %setsand stores the x & y coordinates
    x = [0,a(1,1:stepsTot(2))]; 
    y = [0,a(1,stepsTot(2)+1:end)];
    
    %plot the pattern
    plotPattern(x,y);
    hold on
    grid on 
    hold off
    %writes pattern to a text file given a specific path.
    %for the purpose of using reading in string from text file for
    %argos, txt file is read in iAnt_Controller.cpp file under
    %linh_development branch.
    if run == 0
        path = '/Users/linhtran/Desktop/iAntProject/iAnt-ARGoS/';
        fid=fopen([path,'DiamondPatternOutPut.txt'],'w');
        fprintf(fid, pattern);
        fclose(fid);
    end
    
%next objective.
%1.consider multiple robots situation
%2.implement in ARGoS
%     a.don't pick anything up when searching.
%     b.when pick up go directly to nest,and directly back to where you stop
%searching
end
    
    




















%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%plotFood();
    
%     m = 1; b= 0; x = -10:10;
%     plot(x,m*x+b);
%     scatter(x, m*x+b);
    %initPattern = ['N','S','E','W','S','N','W','N'];
    
   %Call helper function findState that records the fist instance
   %of a "state" found in the the inital state and store the index as q.
%    [p,q] = findState(initPattern,initPos);

   
 %    disp(states(r))
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
   % p = 0 || 1, q = index of the 1st occurrence.
   % this is how you call findState ([1,2,3,4],4)
%     function [p,q] = findState (initPattern, state)
%         [p,q] = ismember(state, initPattern); 
%     end
%     
    %store the pattern starting from the first instance of the state 
    %and ends at the end of the initPattern.
    
%     pattern = initPattern(q:length(initPattern));
  
%     coord_path = parsingPattern(initPos,pattern);
%     coord_path = [initPos, coord_path];
%     disp(['Coord Path', coord_path]);
               


