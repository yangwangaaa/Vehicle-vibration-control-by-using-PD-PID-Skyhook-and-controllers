%%-------------------------------------------%%
%%Task 1.1---------
% Equations in report

%Task 1.2-----------
mp = 0.16;
cp = 0.4;
kp = 6.32;
wn = sqrt(kp/mp);
Ita = cp/(2*sqrt(kp*mp));

%%Task 1.3%%------------
T1= tf([0.4 6.32],[0.16 0.4 6.32]);     %Damped System
T2 = tf([6.32],[0.16 0 6.32]);         %Undampled system
% figure(1)
% bode(T1,'r'),hold on,grid on
% bode(T2,'k'),grid on,hold off

%%Task 1.4-------------
% sys2 = ([2*Ita.*wn wn.^2],[1 2*Ita*wn wn^2]);

%Before excitation requirements for plots
Nf1 = 1; %natural frequency
cpx = Nf1* 2* sqrt(kp*mp);
T1x = tf([cpx 6.32],[0.16 cpx 6.32]);
Nf2 = 0.5;    %underdamped
cpxx = Nf2* 2* sqrt(kp*mp);
T1xx = tf([cpxx 6.32],[0.16 cpxx 6.32]);
Nf3 = 4;       %overdamped
cpxxx = Nf3* 2* sqrt(kp*mp);
T1xxx = tf([cpxxx 6.32],[0.16 cpxxx 6.32]);

% SinosuidalExcitation:
t = 0:0.1:10;    %u,t define the input signal
u = 0.05*sin(1*t); 
% figure(2)
% lsim(T1,u,t,'k'),hold on, grid on

%EXTRA PART : We compared sine excitation for natural/overdamped/underdamped
% lsim(T1x,u,t,'g'),hold on, grid on    %natural
% lsim(T1xx,u,t,'r'),hold on, grid on     %underdamped
% lsim(T1xxx,u,t,'b'),hold off, grid on       %overdamped   

% %StepResponse
% figure(3)
% step(0.05*T1,'k'),hold on,grid on
% step(0.05*T1x,'r'),hold on,grid on
% step(0.05*T1xx,'g'),hold on, grid on
% step(0.05*T1xxx,'b'),grid on,hold off

% % PSD
cp = 0.4;
w = 0:0.1:25;
H1 = freqresp(T1,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
Hx1 = reshape(H1,[1 251]); 
PSD1 = abs((Hx1.^2)).*(Sw);
% figure(4)
% semilogy(w,PSD1,'r');

%%------------------------------------------------%%

%% Task 2.1  (PD)---------------------------------
%equation in report

%%Task 2.2 (PD)----------------------------------
ddi = cp;
dp = 0;

%%Task 2.3  (PD)----------------------------------
%Manually tuning dp and dd values by observing the graph
T3 = tf([6.32],[0.16 0.4 6.32]);  % comparing to damped passive system and finding values
   
%Tuning for Dd
T4 = tf([6.32],[0.16 0.1 6.32]);
T5 = tf([6.32],[0.16 1.005 6.32]);       %Real Case scenario Ita = 0.5, cp = dd = 1
T6 = tf([6.32],[0.16 1.4 6.32]);
T7 = tf([6.32],[0.16 1.7 6.32]);
T8 = tf([6.32],[0.16 1.85 6.32]);      %ddi = 1.85(critical damping happening)[Extra task 1]
T9 = tf([6.32],[0.16 2 6.32]);

%Tuning for Dp
T10 = tf([6.32],[0.16 1.005 6]); % dp = 0.32
T11i = tf([6.32],[0.16 1.005 6.24]);  %dp = 0.08
T12 = tf([6.32],[0.16 1.005 6.28]);   %dp = 0.04
T13 = tf([6.32],[0.16 1.005 6.32]);   %dp = 0            %no ss error
T14 = tf([6.32],[0.16 1.005 6.36]);   %dp = -0.04
T15 = tf([6.32],[0.16 1.005 6.8]);    %dp = -0.48

T16 = tf([6.32],[0.16 1.005 6.32]);    % Final Tuning PD controller values
T17 = tf([6.32],[0.16 1.85 6.32]);     %critical damping PD


% SinosuidalExcitation:
t = 0:0.1:10; %u,t define the input signal
u = 0.05*sin(1*t);
% figure(5)
% lsim(T8,u,t,'k')   %PD(Critical)
% hold on 
% lsim(T3,u,t,'r')  %Damped
% hold on

%%Final Tuning
% lsim(T16,u,t,'g')   %PD(real scenario- tuned values)
% hold off

%Step excitation
% figure(6)
% step(0.05*T3,'k'),hold on,grid on   %comparative system
% step(0.05*T1,'m'),hold on,grid on   % damped passive system

% step(0.05*T4,'r'),hold on,grid on
% step(0.05*T5,'g'),hold on, grid on   % real case dd tuned
% step(0.05*T6,'b'),grid on,hold on
% step(0.05*T7,'r'),hold on,grid on
% step(0.05*T8,'g'),hold on, grid on
% step(0.05*T9,'b'),grid on,hold off

% step(0.05*T10,'r'),hold on,grid on
% step(0.05*T11i,'g'),hold on, grid on
% step(0.05*T12,'b'),grid on,hold on
% step(0.05*T13,'y'),hold on,grid on
% step(0.05*T14,'c'),hold on, grid on
% step(0.05*T15,'m'),grid on,hold off

% % PSD
cp = 0.4;
w = 0:0.1:25;
H2 = freqresp(T16,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
Hx2 = reshape(H2,[1 251]); 
PSD2 = abs((Hx2.^2)).*(Sw);
% figure(7)
% semilogy(w,PSD2,'g');
%-----------------------------------------

%% Task 3.1
hp = 0;
hd = 1.3;
hi = 0;
T31 = tf([kp 0],[mp hd kp+hp hi]);
% 
% %sine excitations
% figure(8)
% lsim(T31,u,t,'r'),hold on, grid on
% lsim(T1,u,t),hold off, grid on
 
% % %Step excitation
% figure(9)
% step(0.05*T31,'r'),grid on,hold on
% step(0.05*T16,'b'),grid on, hold off

% % PSD
cp = 0.4;
w = 0:0.1:25;
H3 = freqresp(T31,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
Hx3 = reshape(H3,[1 251]); 
PSD3 = abs((Hx3.^2)).*(Sw);
% figure(10)
% semilogy(w,PSD3,'b');

%Task 3.2
%In report
%-------------------------------------------

%% Task 4.1 SKYHOOK

%equation in report
Tx1 = 0.4;              
T41 = tf([kp],[mp Tx1 kp]);
Tx2 = 1;                 %tuned for litte underdamped system  
T42 = tf([kp],[mp Tx2 kp]);
Tx3 = 1.6;  
T43 = tf([kp],[mp Tx3 kp]);
Tx4 = 1.7;
T44 = tf([kp],[mp Tx4 kp]);
Tx5 = 1.8; 
T45 = tf([kp],[mp Tx5 kp]);
Tx6 = 1.9;               % tuned for 1.9  critical damped            
T46 = tf([kp],[mp Tx6 kp]);
Tx7 = 2;
T47 = tf([kp],[mp Tx7 kp]);
Tx8 = 2.2;
T48 = tf([kp],[mp Tx8 kp]);

% %sine excitation
% figure(11)
% lsim(T43,u,t,'r'),hold on, grid on
% lsim(T46,u,t,'g'), hold on, grid on
% lsim(T1,u,t,'b'), hold off ,grid on

% Step excitation
% figure(12)
% % step(0.05*T1x,'c'),grid on,hold on    %damped system
% % step(0.05*T16,'k'),grid on,hold on    %PD
% step(0.05*T41,'r'),grid on,hold on    %trial one tuning T 
% step(0.05*T42,'g'),grid on, hold on   %trial two tuning T %underdamping
% step(0.05*T43,'b'),grid on, hold on   %trial two tuning T
% step(0.05*T44,'k'),grid on, hold on   %trial two tuning T
% step(0.05*T45,'y'),grid on, hold on   %trial two tuning T
% step(0.05*T46,'c'),grid on, hold on   %trial two tuning T   %Criticaldamping
% step(0.05*T47,'m'),grid on, hold on   %trial two tuning T
% step(0.05*T48,'r'),grid on, hold off   %trial two tuning T

%PSD

cp = 0.4;
w = 0:0.1:25;
H4 = freqresp(T42,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
Hx4 = reshape(H4,[1 251]); 
PSD4 = abs((Hx4.^2)).*(Sw);
% figure(13)
% semilogy(w,PSD4,'y'),hold on, grid on

%% Task 5.1 

% Comparing Amplitudes
% figure(14)
% bode(T1,'r'),hold on,grid on            %damped
% bode(T2,'g'),hold on,grid on            %undamped
% bode(T16,'b'),hold on,grid on           %PD
% bode(T31,'y'),hold on,grid on           %PID
% bode(T42,'k'),hold off,grid on          %Skyhook

%Task 5.2
% %Sine xcitation - ALL
% figure(15)
% lsim(T1,u,t,'r'),hold on, grid on    %Damped System
% %  lsim(T2,u,t,'g'),hold on, grid on    %Undampled System
% lsim(T16,u,t,'b'),hold on, grid on   %PD
% lsim(T31,u,t,'y'),hold on, grid on   %PID
% lsim(T42,u,t,'k'),hold off, grid on   %Skyhook

% %Step xcitation - ALL
% figure(16)
% step(0.05*T1,'y'),grid on,hold on     %damped system
% step(0.05*T16,'r'),grid on,hold on    %PD
% step(0.05*T31,'b'),grid on,hold on    %PID 
% step(0.05*T42,'k'),grid on,hold off   %Skyhook

% %PSD xcitation -ALL
% figure(17)
% semilogy(w,PSD1,'r'),hold on,grid on    %damped
% semilogy(w,PSD2,'g'),hold on,grid on    %PD
% semilogy(w,PSD3,'b'),hold on,grid on    %PID
% semilogy(w,PSD4,'k'),hold off,grid on   %Skyhook

%%------------------------------------------------
%% Task 6.1 
%Equation in report---------------

mp = 0.16;
cp1 = 0.8;
kp = 6.32;
ms = 0.16;
cs = 0.05;
ks = 0.0632;

s = tf('s');
T62 = (s^2*cp*cs+s*(ks*cp+kp*cs)+kp*ks)/(s^4*(mp*ms)+s^3*(cp*ms+cs*mp+cs*ms)+s^2*(kp*ms+cp*cs+ks*mp+ks*ms)+s*(kp*cs+cp*ks)+kp*ks);

%Equation6.2--------------------
% figure(18)
% bode(T62);

% %Sine Excitation
% figure(19)
% lsim(T62,u,t,'r')
% 
% %Step Excitation
% figure(20)
% step(0.05*T62,'c');

%PSD
cp = 0.4;
w = 0:0.1:25;
H5 = freqresp(T62,w);
Sw = (4.028*(10^-7))./(2.88*(10^-4) + 0.68*(w.^2) + (w.^4));
Hx5 = reshape(H5,[1 251]); 
PSD5 = abs((Hx5.^2)).*(Sw);
% figure(21)
% semilogy(w,PSD5,'k'),hold on, grid on   %two mass damped system
% semilogy(w,PSD1,'r'),hold on,grid on    %damped
% semilogy(w,PSD2,'g'),hold on,grid on    %PD
% semilogy(w,PSD3,'b'),hold on,grid on    %PID
% semilogy(w,PSD4,'y'),hold off,grid on   %Skyhook

%Equation 6.3--------------------

A = [0 1 0 0; 
    -ks/ms  0  ks/ms  0;
    0 0 0 1 ;
    ks/mp 0 (-ks-kp)/mp -cp/mp;];

B = [0  0 0 ;
     1/ms  0 0 ; 
     0 0 0; 
     -1/mp  kp/mp cp/mp;];

C = [1 0 0 0];

D = 0;

%Equation 6.4--------------------


%% Task 7




