% PROFILE GENERATOR
% Desc : Generates velocity and accleration profile using coordinate values from a mat file
		 under given constraints for the project 
% Input : Coords .mat file
% Output : Acceleeration and velocity arrays
% By : Kshitij Kumbar


clear all; close all;
load('project_data.mat')
load('desired.mat')
aX = zeros(1009,1); 
K_T = path.k_1pm

%% Speed for 0 - 49

s = (path.s_m(1:49)-path.s_m(49));
% K = (path.k_1pm(159:205));
Ux1(1)= 0;
aDes(1)= 0;
aX(1) = 0;
aX(2:49)=3*ones(48,1);
for i = 1:length(s)
    if i<length(s)
        Ux1(i+1) = sqrt(Ux1(i)^2 + 2*2.6*0.25);
    end
end
Ux = [Ux1'];
clear Ux1
clear aDes1
clear s
clear K

%% Speed for 57 - 96

s = flip(path.s_m(50:66)-path.s_m(50));
K = flip(path.k_1pm(50:66));


idx = length(s);
Ux1= zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 3;
for i = 1:idx
    if i<idx
        Ux1(i+1) = Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2));
        Ux1(i+1) = abs(min(Ux1(i+1)));
        aX1(i+1)= min((Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25,3);
    end
end
Ux = [Ux;flip(Ux1)];
aX(50:66) = -flip(aX1);

clear Ux1;
clear aDes1
clear aX1;
clear s;
clear K;
%% Speed for 97 - 157
Ux1 = 5.87*ones(157-67+1,1);
Ux = [Ux;Ux1]

aX(67:157) = zeros(157-67+1,1);
clear Ux1
clear aDes1
clear s;
clear K;

%% Speed for 159 - 205
s = (path.s_m(158:205)-path.s_m(154));
K = (path.k_1pm(158:205));
idx = length(s)
Ux1 = zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= min(3,(Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25);
    end
end
Ux = [Ux;Ux1]
aX(158:205) = aX1;
clear Ux1;
clear aDes1
clear aX1;
clear s;
clear K;

%% Speed for 206 - 301

s = (path.s_m(205:262)-path.s_m(205));
% K = (path.k_1pm(159:205));
Ux1(1)= 9.531;
aX(205:262) = 3*ones(length(s),1);
for i = 1:length(s)
    if i<length(s)
        Ux1(i+1) = abs(sqrt(Ux1(i)^2 + 2*3*0.25));
        
    end
end
Ux = [Ux;Ux1']
clear Ux1;
clear s;
s = (path.s_m(263:301)-path.s_m(263));

Ux1(length(s))= 9.69;
aX(263:301) = -4*ones(length(s),1);
for i = length(s):-1:1

    if i>1
        Ux1(i-1) = sqrt(Ux1(i)^2 + 2*4*0.25);
    end
end
Ux = [Ux;Ux1']
clear Ux1;
clear aDes1
clear K;
clear s;

%% Speed for 302 - 348

s = flip(path.s_m(302:348)-path.s_m(302));
K = flip(path.k_1pm(302:348));

clear aX1;
idx = length(s);
Ux1 = zeros(idx,1);
Ux1(1) = 5.875;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= min(3,(Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25);   
    end
end
Ux1 = flip(abs(Ux1))
for i = 1:idx
    if i<idx
        aX1(i+1)= (Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25;   
    end
end

aX(302:348)=((aX1));
aX(302) = - 4;
clear aX1;

Ux = [Ux;Ux1]
clear Ux1;
clear aDes1
clear s;
%% Speed for 349 - 409
Ux1 = 5.875*ones(408-349+1,1);
Ux = [Ux;Ux1]
aDes1 = 4*ones(408-349+1,1);
aX(349:408) = zeros(408-349+1,1);
aDes = [aDes;aDes1];
clear Ux1;
clear aDes1

%% Speed for 410 - 456
clear K;
clear s;
s = (path.s_m(410:455)-path.s_m(410));
K = (path.k_1pm(410:455));
idx = length(s)
Ux1 = zeros(idx,1);
Ux1(1) = 5.877;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= min(3,(Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25);
        
    end
end

Ux1 = abs(Ux1)
Ux = [Ux;Ux1]
aX(410:455) = aX1;
clear aX1;
clear Ux1;

%% Speed for 457 - 552
clear s;
clear K;
s = (path.s_m(456:513)-path.s_m(456));
Ux1(1)= 9.65;
for i = 1:length(s)
    
    if i<length(s)
        Ux1(i+1) = sqrt(Ux1(i)^2 + 2*3*0.25);
        
    end
end
aX(456:513) = 3*ones(length(s),1);
Ux = [Ux;Ux1']
clear Ux1;
clear aDes1
clear s;
s = (path.s_m(514:552)-path.s_m(514));
Ux1(length(s))= 9.65;
aX(514:552) = -4*ones(length(s),1);
for i = length(s):-1:1
    aDes1(i)= -4;
    if i>1
        Ux1(i-1) = sqrt(Ux1(i)^2 + 2*4*0.25)
        
    end
end

Ux = [Ux;Ux1']
clear Ux1;
clear aDes1
clear s;
clear K;
%% Speed for 553 - 599
clear aX1;
s = flip(path.s_m(553:599)-path.s_m(553));
K = flip(path.k_1pm(553:599));

idx = length(s);
Ux1 = zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= (Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25;
        
    end
end
aX(553:599) = -flip(aX1);
Ux = [Ux;flip(Ux1)]
clear Ux1;
clear aDes1
clear s;
clear K;
clear aX1;


%% Speed for 600 - 661
clear s;
clear K;
Ux1 = 5.877*ones(661-600+1,1);
Ux = [Ux;Ux1]
aX(600:661) = zeros*ones(661-600+1,1);
clear Ux1;
clear aDes1

%% Speed for 662 - 708
clear aX1;
s = (path.s_m(662:708)-path.s_m(662));
K = (path.k_1pm(662:708));
idx = length(s)
Ux1 = zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= min(3,(Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25);
    end
end
aX(662:708) = aX1;
Ux1 = abs(Ux1)
Ux = [Ux;Ux1]
clear Ux1;
clear aDes1
clear s;
clear K;
clear ax1;

%% Speed for 709 - 804

s = (path.s_m(709:765)-path.s_m(709));
% K = (path.k_1pm(159:205));
Ux1(1)= 9.65;
aX(709:765) = 3*ones(length(s),1);
for i = 1:length(s)
    if i<length(s)
        Ux1(i+1) = sqrt(Ux1(i)^2 + 2*3*0.25)
        
    end
end
Ux = [Ux;Ux1']
clear Ux1;
clear aDes1
clear s;
clear aX1;

s = (path.s_m(766:804)-path.s_m(766));
% K = (path.k_1pm(159:205));
Ux1(length(s))= 9.67;
aX(766:804) = -4*ones(length(s),1);
for i = length(s):-1:1
    
    if i>1
        Ux1(i-1) = sqrt(Ux1(i)^2 + 2*4*0.25);
        
    end
end

Ux = [Ux;Ux1']
clear Ux1;
clear aDes1
% plot(s,Ux1);
%% Speed for 805 - 852
clear s;
clear K;
clear aX1;
s = flip(path.s_m(805:852)-path.s_m(805));
K = flip(path.k_1pm(805:852));
idx = length(s);
Ux1 = zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        aX1(i+1)= (Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25;
    end
end
aX(805:852) = -flip(aX1);
Ux1 = abs(Ux1);
Ux = [Ux;flip(Ux1)]
clear Ux1;
clear aDes1
clear s;
clear K;
clear aX1;
%% Speed for 853 - 913
Ux1 = 5.87*ones(913-853+1,1);
Ux = [Ux;Ux1]
aX(853:913) = zeros(length(Ux1),1);
clear Ux1;
clear aDes1;
%% Speed for 914 - 960
clear s;
clear K;
s = (path.s_m(914:960)-path.s_m(914));
K = (path.k_1pm(914:960));
idx = length(s)
Ux1 = zeros(idx,1);
Ux1(1) = 5.87;
ay_max = 4;
a_max = 4;
for i = 1:idx
    if i<idx
        Ux1(i+1) = abs(Ux1(i) + 0.25*(1/(Ux1(i)))*sqrt(((a_max)^2 - (K(i)*Ux1(i)^2)^2)));
        Ux1(i+1) = min (Ux1(i+1),8.65)
        aX1(i+1)= min(3,(Ux1(i)*(Ux1(i+1) - Ux1(i)))/0.25);
    end
end
aX(914:960) = aX1;
clear aX1;
Ux1 = abs(Ux1)
Ux = [Ux;Ux1]
clear Ux1;
clear aDes1
%% Speed for 961 - 996
s = (path.s_m(961:996)-path.s_m(961));
clear Ux1
% K = (path.k_1pm(159:205));
Ux1(length(s))= 0;
aX(961:996) = -4*ones(length(s),1);
for i = length(s):-1:1
    if i>1
        Ux1(i-1) = sqrt(Ux1(i)^2 + 2*4*0.25)
       
    end
end
 Udes = [Ux;Ux1';0.001*ones(13,1)]
 