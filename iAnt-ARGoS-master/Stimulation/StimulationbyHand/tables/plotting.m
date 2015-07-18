function plotting (myfile)
    data1 = importdata(myfile);
    mydata = data1.data;
    disp(data1)
    disp(mydata(:,1:2))
    plot(mydata(:, 1))
    mean(mydata(:,1:2))
    ylabel('Tags Collected', 'fontsize', 14)
    xlabel('Time in Minutes','fontsize', 14)
    title('Random Cluster Distribution', 'fontsize',16)
end


    
