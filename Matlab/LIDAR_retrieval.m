%%
BinLength = 20;
Bins = 100:BinLength:2000;
Beta = 1e6;
CloudRange = 600;
CloudRange2 = 1800;
CloudRadius = 50;
CloudMaxDensity = 10; % particles/cc
Background = 1;
% Gaussian distribution with mean mu and std sig is
% (1/(sig*sqrt(2*pi)))exp(-(1/2)*((x-mu)/sig)^2)
Density1 = CloudMaxDensity*exp(-(1/2)*((Bins-CloudRange)/CloudRadius).^2);
Density2 = CloudMaxDensity*exp(-(1/2)*((Bins-CloudRange2)/CloudRadius).^2);
Density = Density1+Density2+Background;
%%
plot(Bins,Density,'*');
shg;
%%
signal = Beta .* BinLength .* Density ./ (Bins.^2);
%%
plot(Bins,signal,'*');
shg;
%%
SN = 0*signal;
for i=1:length(signal)
  SN(i) = SamplePoisson(signal(i));
end
%%
plot(Bins,signal,Bins,SN,'*'); shg
%%
NSN = SN.*(Bins.^2)./(Beta.*BinLength);
%%
plot(Bins,Density,Bins,NSN,'*'); shg
