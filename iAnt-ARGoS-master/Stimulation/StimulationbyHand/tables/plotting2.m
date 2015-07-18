function plotting2 (myfile,xlab,ylab,mytitle,cond)
    data1 = importdata(myfile);
    mydata = data1.data;
    disp(data1)
    disp(mydata(:,1:2))
    plot(mydata(:, 1), cond)
    mean(mydata(:,1:2))
    xlabel(xlab ,'fontsize', 16)
    ylabel(ylab, 'fontsize', 16)
    title(mytitle,'fontsize',16)
end