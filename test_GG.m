% GENERATING DATA
ndata = 15;

rand('state',1);  % to get always the same set of random points
randn('state',1); % to get always the same set of random points
ndata1=ceil(0.5*ndata);
ndata2=ndata-ndata1;
data=[randn(ndata1,2); rand(ndata2,2)];

dataDist=distFast(data,data);
[dall,indall]=sort(dataDist,'ascend');
[G] = GabrielGraph(data, indall);

%preallocating adj matrix
adj = zeros(ndata, ndata);

figure(10)
plot(data(:,1),data(:,2),'k.');
hold on
for t=1:length(G)
    neighbors=G{t};
    numn=length(neighbors);
    for nt=1:numn
        adj(t,neighbors(nt)) = 1;
        midpoint=0.5*(data(t,:)+data(neighbors(nt),:));
        plot([data(t,1) midpoint(1)],[data(t,2) midpoint(2)],'r-');
        hold on
    end
end
plot(data(:,1),data(:,2),'k.');
axis equal
hold off