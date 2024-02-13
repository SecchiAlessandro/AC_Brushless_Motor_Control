clc
clear all

%% DC machine parameters
load('Traction_e402B_online');
load('ac_brushless_data');
%parameters of 1 motor = equivalent motor
V_batt = 800; %voltage
pp=2; %pole pairs of each motor
cos_fi=1; %power factor of each motor
Rs=0.1; %ohm
Ls=0.8*10^-3; %H

d=60*10^-2; %diameter wheels
tr=1/3; %trasmission ratio
mass_tot=1580; %kg

v_max=240*1000/3600; %max speed m/s
omega_max=v_max/tr/d*2; %rad/s
omega_rated=400; %rad/s
v_rated=omega_rated*tr*d/2*3.6; %km/h
eff=0.95; %efficiency 
J=mass_tot*v_max^2/omega_max^2;
%J_eq=mass_tot*(tr*d)^2/4;
v_base=70/3.6; %base speed assumed in m/s
omega_base=v_base/tr/d*2; %rad/s
Pn=180*10^3; %tot mechanical power [W]
Pe_tot=Pn/eff;


phi_PM=0.5; %Wb

Tn = Pn/omega_base; % Nominal torque provided by the machine
i_max=Tn/pp/eff/phi_PM;
V_motor=Pe_tot/sqrt(3)/i_max/cos_fi;

tau_s=Ls/Rs;
B=0.065;
tau_O=J/B;

a_max=4; %m/s
T_max=mass_tot*a_max*d/2*tr;

T_friction=4*B*omega_rated;
% B = Tfriction/rated_speed_motor; % friction coefficient < Or
% tau_mec=J_eq/B;
speed_rad_sim=speed_kmh./((d/2)*tr*3.6); %rad/s


speed_ref =[0, 35, 70,150,160,170,180,190,200,210,220];

id_ref=[0,0,0,-phi_PM/Ls,-phi_PM/Ls,-phi_PM/Ls,...
    -phi_PM/Ls,-phi_PM/Ls,-phi_PM/Ls,-phi_PM/Ls,-phi_PM/Ls];


%%  PI controller design parameters
s=tf('s');
%Gi
tau_s_desired=tau_s/1000;
wc_s=2*pi/tau_s_desired;

%GO
tau_O_desired=tau_O/1000;
wc_O=2*pi/tau_O_desired;

%tf
Gi = 1/(Rs+Ls*s);

GO = 1/(B+J*s);


%% Zero Pole cancellation (90 phase margin)
% %PI parameters ia
% kp_a=wc_a*La;
% ki_a=wc_a*Ra;
% Regi=kp_a+ki_a/s
% Ti_a=kp_a/ki_a;
% %tf open loop
% Li=Regi*Gi;
% %tf close loop
% Fi=Li/(1+Li);
% % figure
% % bode(Li)
% % figure
% % bode(Fi)
% %PI parameters ie
% kp_e=wc_e*Le;
% ki_e=wc_e*Re;
% Rege=kp_e+ki_e/s
% Ti_e=kp_e/ki_e;
% %tf open loop
% L_e=Rege*Ge;
% %tf close loop
% Fe=L_e/(1+L_e);
% % figure
% % bode(L_e)
% % figure
% % bode(Fe)
% %PI parameters speed
% kp_O=wc_O*J_eq;
% ki_O=wc_O*B;
% RegO=kp_O+ki_O/s
% Ti_O=kp_O/ki_O;
% %tf open loop
% LO=RegO*GO;
% %tf close loop
% FO=LO/(1+LO);
% % figure
% % bode(LO)
% % figure
% % bode(FO)
% %saturation
% SatUp_Va = Van; % [A]
% SatLow_Va = -Van;
% SatUp_T = Tn*10; % [V]
% SatLow_T = -Tn*10;
% SatUp_Ve = Ven*1.1;
% SatLow_Ve = 0;
%% Pidtool
%otherwise use pidtool
phase_m=90;
%pidtool(Gi)
opt=pidtuneOptions('PhaseMargin', phase_m);
par_regi=pidtune(Gi,'PI',wc_s,opt);

ki_s=par_regi.Ki;
kp_s=par_regi.Kp;

Regi=kp_s+ki_s/s
Ti_s=kp_s/ki_s;
%tf open loop
Li=Regi*Gi;
%tf close loop
Fi=Li/(1+Li);
% figure
% bode(Li);
% figure
% bode(Fi);
% figure
% margin(Li);


%pidtool(GO)
par_reg_speed=pidtune(GO,'PI',wc_O,opt);
ki_O=par_reg_speed.Ki;
kp_O=par_reg_speed.Kp;

RegO=kp_O+ki_O/s
Ti_O=kp_O/ki_O;
%tf open loop
LO=RegO*GO;
%tf close loop
FO=LO/(1+LO);
% figure
% bode(LO);
% figure
% bode(FO);
% figure
% margin(LO);



