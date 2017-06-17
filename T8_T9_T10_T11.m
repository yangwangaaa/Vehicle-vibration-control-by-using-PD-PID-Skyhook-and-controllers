%%                          %%%
% AVDC Lab 3 
% Task 8 onwards
% Bounce & Pitch Model
                            %%%


 %% 
% Vehicle parametrs

m = 22000 ; %kg
j =  700000 ; %kgm^2
c2 = 40000 ; % Ns/m
c1 = c2 ; 
k2 = 600000 ; % N/m
k1 = 600000 ;
k = 600000;
c = 40000; 
Lambda = 0.5 ; %
L = 6 ; %wheel base % m
% theta = 0 ;
% z= 0 ;

% fa1 = 1 ;
% 
% fa2 = 1 ;



%%
% % % % % % bounce  pitch model 
% % % % % 
% % % % % m = [m 0 ;0 j/L*L];      % mass matrix
% % % % % 
% % % % % c = [c1+c2 ((1-lambda)*c2-(lambda*c1)) ; ((1-lambda)*c2-(lambda*c1)) (lambda*lambda*c1 + ((1-lambda).^2)*c2)] ;   % damping matrix
% % % % %     
% % % % % k = [k1+k2 ((1-lambda)*k2-(lambda*k1)) ; ((1-lambda)*k2-(lambda*k1)) (lambda*lambda*k1 + ((1-lambda).^2)*k2)] ;   % stiffness matrix
% % % % % 
% % % % % q = [z theta*L] ;   % force matrix 

%% 
% Task 8  
% state space
 
 A = [ 0 1 0 0 ; 
     (k1 + k2)*(-1/m) (c1 + c2)*(-1/m) ((1-Lambda)*k2 -Lambda*k1)*(-L/m) ((1-Lambda)*c2 - Lambda*c1)*(-L/m);
     0 0 0 1; 
     ((1-Lambda)*k2 - Lambda*k1)*(-L/j) ((1-Lambda)*c2 - Lambda*c1)*(-L/j) (Lambda*Lambda*k1 + (1- Lambda)*(1-Lambda)*k2)*(-(L*L)/j) (Lambda*Lambda*c1 + (1- Lambda)*(1-Lambda)*c2)*(-(L*L)/j )] ;

 B = [ k1/m c1/m k2/m c2/m ;
      0 0 0 0; 
     (-k1*Lambda*L)/j (-c1*Lambda*L)/j (k2*(1-Lambda)*L)/j (c2*(1-Lambda)*L)/j;
      0 0 0 0;] ;
 
 C = [1 0 0 0 ;0 1 0 0;] ; 
 
 D = 0 ;
 
 BP=ss(A,B,C,D);
 
 
 
 %% 
%  A = [ 0 1 0 0 ;
%      (k1 + k2)*(-1/m) (-fa1 -fa2)*(-1/m) ((1-Lambda)*k2 -Lambda*k1)*(-L/m) ((1-Lambda)*(-fa2) - Lambda*(-fa1))*(-L/m)
%      ; 0 0 0 1; 
%      ((1-Lambda)*k2 - Lambda*k1)*(-L/j) ((1-Lambda)*(-fa2) - Lambda*(-fa1))*(-L/j) (Lambda*Lambda*k1 + (1- Lambda)*(1-Lambda)*k2)*(-(L*L)/j) (Lambda*Lambda*(-fa1) + (1- Lambda)*(1-Lambda)*(-fa2))*(-(L*L)/j )] ;
% 
%   B = [ k1/m k2/m 1/m 1/m ; 
%       (-k1*Lambda) (Lambda) (k2*(1-Lambda)) (1-Lambda);
%       0 0 0 0; 
%       0 0 0 0;] ;
%  
%  C = [1 0 0 0 ;0 1 0 0;] ; 
%  
%  D = 0 ;
%  
%  
% SHBP = ss(A,B,C,D);


%% 
Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];
Csk=[0 1 0 0
    0 0 0 1];
Dsk=zeros(2,4);

BPSK = ss(Ask,Bsk,Csk,Dsk);

E = [0 1 0 0 ; (k1 + k2)*(-1/m) (c1 + c2)*(-1/m) ((1-Lambda)*k2 -Lambda*k1)*(-L/m) ((1-Lambda)*c2 - Lambda*c1)*(-L/m); 0 0 0 1; ((1-Lambda)*k2 - Lambda*k1)*(-L/j) ((1-Lambda)*c2 - Lambda*c1)*(-L/j) (Lambda*Lambda*k1 + (1- Lambda)*(1-Lambda)*k2)*(-(L*L)/j) (Lambda*Lambda*c1 + (1- Lambda)*(1-Lambda)*c2)*(-(L*L)/j )] ;

F= eig(E);

%% 

%% This is a Matlab file for designing H_infinity controller (assignment 3
%% of SD2231)
% clear all
s=tf('s');

% systme parameters
m=22000;   %kg
j=700e3;   %kgm^2
c=40e3;    %Ns/m
k=2*300e3; %N/m
L=6;       %m

%% State space model for skyhook contorl
Ask=[0 1 0 0
    -2*k/m 0 0 0
    0 0 0 1
    0 0 -2*k*L^2/j 0];
Bsk=[0 0 0 0
    k/m k/m -1/m -1/m
    0 0 0 0
    -L*k/j L*k/j L/j -L/j];
Csk=[0 1 0 0
    0 0 0 1];
Dsk=zeros(2,4);

sys=ss(Ask,Bsk,Csk,Dsk);
%% H_inf using linmod syntax

%state space: The same as skyhook

      
%Weighting functions

%For penalizing actuator force
Wa1=(0.00175*s+1)/(0.00025*s+1);
Wa2=Wa1;

%For penalizing bounce and pitch motions
eps=1;
wnb= 7.38 ;            %Find the right equation or value for wnb
wnchi= 7.8558  ;          %Find the right equation or value for wnchi
s1b=-eps+1i*sqrt(wnb^2-eps^2);
s2b=-eps-1i*sqrt(wnb^2-eps^2);
s1chi=-eps+1i*sqrt(wnchi^2-eps^2);
s2chi=-eps-1i*sqrt(wnchi^2-eps^2);
kb= 7750 ;
% input('Enter the gain for Wb = '); 
kchi= 30000 ;
% input('Enter the gain for Wchi = ');
Wb=(kb*s1b*s2b)/((s-s1b)*(s-s2b));
Wchi=(kchi*s1chi*s2chi)/((s-s1chi)*(s-s2chi));

%Extracting the extended model
[A_Pe,B_Pe,C_Pe,D_Pe] = linmod('Extended_model');% state space parameters of the extended system: Pe
Pe=ss(A_Pe,B_Pe,C_Pe,D_Pe);

%Calculating the controller
ncont = 2;%Number of control inputs
nmeas = 2;%Number of measured outputs provided to the controller
Pe=minreal(Pe);%This syntax cancels pole-zero pairs in transfer
%functions. The output system has minimal order and the same response
%characteristics as the original model.
[K,Pec,gamma,info]=hinfsyn(Pe,nmeas,ncont,'method','lmi'); % for working with the error
[Ainf, Binf, Cinf, Dinf]=ssdata(K);

%Now use the controller K in your simulation