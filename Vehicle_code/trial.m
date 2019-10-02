s_cloth = path.s_m(50:96);
k_cloth = path.k_1pm(50:96);
plot(s_cloth,k_cloth)
p = polyfit(s_cloth,k_cloth,1);
y = polyval(s_cloth,p)
plot(s_cloth,y)
%%
s_cloth = path.s_m(159:205);
k_cloth = path.k_1pm(159:205);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(302:348);
k_cloth = path.k_1pm(302:348);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(410:456);
k_cloth = path.k_1pm(410:456);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(553:599);
k_cloth = path.k_1pm(553:599);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(662:708);
k_cloth = path.k_1pm(662:708);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(805:852);
k_cloth = path.k_1pm(805:852);
plot(s_cloth,k_cloth)
polyfit(s_cloth,k_cloth,1)
%%
s_cloth = path.s_m(914:960);
k_cloth = path.k_1pm(914:960);
plot(s_cloth,k_cloth)
plot(polyfit(s_cloth,k_cloth,1))