function TestPoisson(mu)
  % TestPoisson(mu)
  
  N = 1000;
  x = zeros(N,1);
  for i=1:N
    x(i) = SamplePoisson(mu);
  end
  xmax = max(x);
  bins = 0:xmax;
  NH = hist(x,bins);
  NHexp = zeros(size(NH));
  NGexp = zeros(size(NH));
  NHexp(1) = N*exp(-mu);
  NGexp = N*(1/sqrt(mu*2*pi))*exp(-(1/2)*(bins-mu).^2/mu);
  for i=2:length(NHexp)
    NHexp(i) = NHexp(i-1)*mu/(i-1);
  end
  figure; plot(bins,NH,'*',bins,NHexp,bins,NGexp);
  % plot(bins,NHexp-NGexp);
  meanx = mean(x);
  title(sprintf('mu = %.1f, mean = %.1f', mu, meanx));
  
