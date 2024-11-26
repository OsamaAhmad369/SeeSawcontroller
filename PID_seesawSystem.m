clc;clear; close all

%%%Parameters of SeeSaw System%%%
m1=70*(10^-3);%Mass of bldc 1 from center in kg
m2=50*(10^-3);%Mass of bldc 2 from center in kg
l1=0.65; %position of bldc 1 from center in meters
l2=0.65;%position of bldc 2 from center in meters
g=9.81;

%%%Parameters of BLDC System%%%

R=0.304;
L=0.2135*(10^-3);
P=24;
J=0.0099;  %inertial in kgm^-2
ke=36.9*(10^-3);
kt=36.9*(10^-3);
B=0.1246; %Damping Coefficient of BLDC motor in Nms
Vd=24;
d=6; %diameter of the propeller in inch
pitch=4;%pitch of the propeller in inch

%%% FBD Equation coefficient of BLDC System %%%
a_bar=(g*(l1*m1-l2*m2))/(m1*l1^2+m2*l2^2);
b_bar=l1/(m1*l1^2+m2*l2^2);
c_bar=l2/(m1*l1^2+m2*l2^2);

wmin_rpm=50;
wmax_rpm=25000;
wmin=wmin_rpm*0.1047;
wmax=wmax_rpm*0.1047;
theeta_sat=30*pi/180;
q=(1.860*10^-11)*(d^3.5)*sqrt(pitch);
%%%Speed Tracking of BLDC motor%%%
Kp=100;
Ki=5;
Kd=0.001;

% W_o=tf(1,[J B])*wref; %open loop transfer function of BLDC speed tracking
% W_c=tf([Kd Kp Ki],[J+Kd B+Kp Ki])*wref;% closed loop transfer function with PID controller
% step(W_o)
% figure
% step(W_c)

%%
%Open loop Control Simulation of SeeSaw System

Time = 10;          % total simulation time in seconds
T = 0.01;           % sampling time
n = round(Time/T);  % number of samples

angle_o(1)=0;
omega(1)=0;
force1(1)=0;
force2(1)=0;
wref=8000*0.1047;
w1(1)=0;
w2(1)=0;
Te1=wref;
Te2=wref;
for i=2:n+1

  w1(i)=(T/J)*Te1+w1(i-1)-(T/J)*B*w1(i-1);
  w2(i)=(T/J)*Te2+w2(i-1)-(T/J)*B*w2(i-1);
  if w1(i)>wmax
      w1(i)=wmax;
  elseif w1(i)<wmin
      w1(i)=wmin;    
  end
  if w2(i)>wmax
      w2(i)=wmax;
  elseif w2(i)<wmin
      w2(i)=wmin;    
  end
   w1_rpm=w1(i)*9.5493;
   w2_rpm=w2(i)*9.5493;
  force1(i)=q*(w1_rpm^2); %Thrust Force of BLDC1 depends on the RPM of the speed 
  force2(i)=q*(w2_rpm^2); %Thrust Force of BLDC2 depends on the RPM of the speed

  force=force2(i)*c_bar-force1(i)*b_bar+a_bar;

  angle_o(i)=angle_o(i-1)+T*omega(i-1)+((T^2)/2)*force;
  if angle_o(i)>0.5236
      angle_o(i)=0.5236;
  elseif angle_o(i)<-0.5236
      angle_o(i)=-0.5236;
  end
  omega(i)=omega(i-1)+T*force;

end
figure
n = 0:T:Time;
angle_o=angle_o*180/pi;
plot(n,angle_o)
title('Open Loop System')
xlabel('Time (sec)')
ylabel('angle (degree)')
legend('Simulated')
%%
%PID control on SeeSaw System

Time = 10;          % total simulation time in seconds
T = 0.01;           % sampling time
n = round(Time/T);  % number of samples

angle(1)=0.5;
omega(1)=0;
force(1)=0;
force1(1)=0;
force2(1)=0;
err(1)=0;
error1(1)=0;
error2(1)=0;
w1(1)=0;
w2(1)=0;
integral(1)=0;
integral1(1)=0;
integral2(1)=0;
derivative(1)=0;
Wref(1)=0;
woffset=wmin;
%PID parameter for SeeSaw Balancing
Kp=0.812; Kd=0; Ki=0.8; Ks=0.92;
%kp=0.65  Ki=0.8  Ks=1
Kp_s=100;   %100
Ks_s=0.7; %0.75
Kd_s=50000; %50000
Ki_s=0.01;  %0.001
w1_p=0;
w2_p=0;
wref_p=0;
desired_angle=0;
sum_err(1)=0;
tic
for i=2:n+1
  err(i)=desired_angle-angle(i-1); 
  
  integral(i)=integral(i-1)+Ki_s*T*(err(i)+err(i-1))/2-Ks_s*(wref_p-Wref(i-1));
  derivative(i)=(2*Kd_s*(err(i)-err(i-1)))/T-derivative(i-1);
  Wref(i)=Kp_s*abs(err(i))+derivative(i)+integral(i);
  wref_p=Wref(i);
  if Wref(i)>wmax
      Wref(i)=wmax;
    elseif Wref(i)<wmin
      Wref(i)=wmin;
    end

  if err(i)<0
     error1(i)= Wref(i)+w1(i-1);
     error2(i)= Wref(i)-w2(i-1);
  elseif err(i)>0
    error1(i)= Wref(i)-w1(i-1);
    error2(i)= Wref(i)+w2(i-1);
  else
      error1(i)=0;
      error2(i)=0;
      w1(i)=w1(i-1);
      w2(i)=w2(i-1);
      integral1(i)=0;
      integral2(i)=0;
    end

  if(err(i)~=0)
   integral1(i)=integral1(i-1)+(Ki*T*(error1(i)+error1(i-1)))/2-Ks*(w1_p-w1(i-1));
  integral2(i)=integral2(i-1)+(Ki*T*(error2(i)+error2(i-1)))/2-Ks*(w2_p-w2(i-1));
  Te1=Kp*error1(i)+integral1(i);
  Te2=Kp*error2(i)+integral2(i);
  w1(i)=(T/J)*Te1+w1(i-1)-(T/J)*B*w1(i-1);
  w2(i)=(T/J)*Te2+w2(i-1)-(T/J)*B*w2(i-1);
  w1_p=w1(i);
  w2_p=w2(i);
  end
  if w1(i)>wmax
      w1(i)=wmax;
  elseif w1(i)<wmin
      w1(i)=wmin;    
  end
  if w2(i)>wmax
      w2(i)=wmax;
  elseif w2(i)<wmin
      w2(i)=wmin;    
  end
  w1_rpm=w1(i)*9.5493;
  w2_rpm=w2(i)*9.5493;
  force1(i)=q*(w1_rpm^2); %Thrust Force of BLDC1 depends on the RPM of the speed 
  force2(i)=q*(w2_rpm^2); %Thrust Force of BLDC2 depends on the RPM of the speed
  
  force(i)=force2(i)*c_bar-force1(i)*b_bar;
  angle(i)=angle(i-1)+T*omega(i-1)+((T^2)/2)*force(i)+((T^2)/2)*a_bar;
  if angle(i)>0.5236
      angle(i)=0.5236;
  elseif angle(i)<-0.5236
      angle(i)=-0.5236;
  end
  omega(i)=omega(i-1)+T*force(i)+T*a_bar;
  if(angle(i)-angle(i-1))==0
  omega(i)=0;
  end
  sum_err(i)=sum_err(i-1)+i*abs(err(i))^2;
end
toc
% plot results
angle=angle*180/pi;
figure
 n = 0:T:Time;
plot(n,angle)
title('Balancing using PID control')
xlabel('Time (sec)')
ylabel('angle (degree)')
legend('Simulated')
figure 
plot(n,Wref)
figure
plot(n,w1)
figure
plot(n,w2)
figure
plot(n,omega)