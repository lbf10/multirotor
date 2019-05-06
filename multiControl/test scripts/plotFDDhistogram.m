rng('shuffle');
histogram(0.1*randn(10000,1),'FaceAlpha',1,'Normalization','probability')
hold on
histogram(0.05*randn(10000,1),'FaceAlpha',1,'Normalization','probability')
hold on
histogram(0.01*randn(10000,1),'FaceAlpha',1,'Normalization','probability')
legend('hit rate = 90 %','hit rate = 95 %','hit rate = 99 %')
title('FDD´s \xi estimation error histogram based on hit rate')
xlabel('\xi error')
ylabel('Count (%)')