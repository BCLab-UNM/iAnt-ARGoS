%This function imports a file and returns an array of
%The mean and standard deviation of the data.
%This is a helper function for the plotCPFAdata script.
%07/08/10
function [meanData,stdData] = dataRead(fileName)
    wholeData = importdata(fileName);
    printData = wholeData.data();
    printData = printData(:,1);
    meanData = mean(printData);
    stdData = std(printData);
end