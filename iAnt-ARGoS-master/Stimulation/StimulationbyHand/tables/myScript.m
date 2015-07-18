hold on
plotting2 RandomDistrib.txt 'Experiments' 'Tags Collected' 'Random Distribution' 'b*-'
plotting2 ClusterDistrib.txt 'Experiments' 'Tags Collected' 'Cluster Distribution' 'ro-'
plotting2 PowerLawDistrib.txt 'Experiments' 'Tags Collected' 'Power Law Distribution' 'gs-'
legend ('Random', 'Cluster','PowerLaw')
title ('Results', 'fontsize', 16)
grid on
hold off