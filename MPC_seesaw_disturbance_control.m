clc;clear;close all

Time = 10;          % total simulation time in seconds
T = 0.01; 
n = round(Time/T);
%%%Parameters of SeeSaw System%%%
m1=50*(10^-3);%Mass of bldc 1 from center in kg
m2=50*(10^-3);%Mass of bldc 2 from center in kg
l1=0.65; %position of bldc 1 from center in meters
l2=0.65;%position of bldc 2 from center in meters
g=9.81;
gr=9.81;
u_k=0.3; %kinetic friction uk (metal on wood)
u_s=0.5; %Static friction us (metal on wood)

%%%Parameters of BLDC System%%%

J=0.0099;  %inertial in kgm^-2
b=0.1246; %Damping Coefficient of BLDC motor i n Nms
d=6; %diameter of the propeller in inch
pitch=4;%pitch of the propeller in inch

%%%Parameters of Ball%%%
mb=50*(10^-3);%mass of the disturbance
Jb=9.99*(10^-6);%moment of inertia 

%%% FBD Equation coefficient of BLDC System %%%
a_bar=(g*(l1*m1-l2*m2))/(m1*l1^2+m2*l2^2);
b_bar=l1/(m1*l1^2+m2*l2^2);
c_bar=l2/(m1*l1^2+m2*l2^2);

wmin_rpm=50;
wmax_rpm=25000;
wmin=wmin_rpm*0.1047;
wmax=wmax_rpm*0.1047;

q=(1.860*10^-11)*(d^3.5)*sqrt(pitch);
fmin=q*(wmin_rpm^2);
fmax=q*(wmax_rpm^2);

umin=fmin*c_bar-fmax*b_bar; %lower bound of torque
umax=fmax*c_bar-fmin*b_bar; %upper bound of torque

Kp=0.8; Kd=0; Ki=0.8; Ks=0.92;
error1(1)=0;
error2(1)=0;
w1(1)=0;
w2(1)=0;
integral1(1)=0;
integral2(1)=0;
w1_p=0;
w2_p=0;
l_b(1)=-l2+(l1+l2)*rand(1,1);
vb(1)=0;
 
% System parameters and simulation parameters
A=[1 T;0 1];
B=[-b_bar*(T^2)/2 c_bar*(T^2)/2;-b_bar*T c_bar*T];
N=6;n_s=2;m_s=2; NT=n;
w1(1)=0;
w2(1)=0;
x0=[0.5;0]; %initial condition 
xr=[0;0];% Desired States/Output
xmin=-0.5236;
xmax=0.5236;
ur=(pinv(B)*(eye(n_s)-A)*xr);
ud=[0;a_bar];
Q=[200 0;0 0.001]; QN=Q; R=0.000001*eye(m_s);
Fx=[1 0;0 0;-1 0;0 0];gx=[0.5236;0;0.5236;0]-Fx*xr;
Fu=[1 0;0 1;-1 0;0 -1];gu=[fmax;fmax;fmin;fmin]-Fu*ur;

x=zeros(n_s,NT+1); x(:,1)=x0;
Xk=zeros(n_s*(N+1),1); Xk(1:n_s,1)=x0-xr;
u=zeros(m_s,NT);
Uk=zeros(m_s*N,1);
zk=[Xk;Uk];

% constructing AX,BU,QX,RU,FX,gX,FU,gU,H
for i=1:N+1
    AX((i-1)*n_s+1:i*n_s,:)=A^(i-1);
    AW((i-1)*n_s+1:i*n_s,:)=[(1-(T/J)*b) 0;0 (1-(T/J)*b)]^(i-1);
end
for i=1:N+1
  for j=1:N
      if i>j
          BU((i-1)*n_s+1:i*n_s,(j-1)*m_s+1:j*m_s)=A^(i-j-1)*B;
          BW((i-1)*n_s+1:i*n_s,(j-1)*m_s+1:j*m_s)=[(1-(T/J)*b) 0;0 (1-(T/J)*b)]^(i-j-1)*(T/J);
      else
          BU((i-1)*n_s+1:i*n_s,(j-1)*m_s+1:j*m_s)=zeros(n_s,m_s);
          BW((i-1)*n_s+1:i*n_s,(j-1)*m_s+1:j*m_s)=zeros(n_s,m_s);
      end    
  end
end

QX=Q;RU=R;
FX=Fx;gX=gx;FU=Fu;gU=gu;
for i=1:N-1
  QX=blkdiag(QX,Q); RU=blkdiag(RU,R);
  FX=blkdiag(FX,Fx);gX=[gX;gx];
  FU=blkdiag(FU,Fu);gU=[gU;gu];
end
QX=blkdiag(QX,QN);
FX=blkdiag(FX,Fx);
gX=[gX;gx];
H=blkdiag(QX,RU);
%%
tic
% simulating system with MPC
sum_err(1)=0;
for k=1:NT
    %optimizers
  xk=x(:,k)-xr;
  fun = @(z)z'*H*z;
  Feq=[eye((N+1)*n_s) -BU];
  geq=AX*xk;
  F=[];
  g=[];
  lb=[xmin*ones(1,(N+1)),-200*ones(1,(N+1)),fmin*ones(1,N*m_s)];
  ub=[xmax*ones(1,(N+1)),200*ones(1,(N+1)),fmax*ones(1,N*m_s)];
  z=fmincon(fun,zk,F,g,Feq,geq,lb,ub)
  u(:,k)=z((N+1)*n_s+1:(N+1)*n_s+m_s,1); %control input from optimizer/controller
  Uk=z((N+1)*n_s+3:(N+1)*n_s+m_s*N,1);

  %system Dynamics
    w_ref=sqrt(abs(u(:,k))/q);
   w1_ref=w_ref(1)*0.1047;%from rpm to rad/s
   w2_ref=w_ref(2)*0.1047;%from rpm to rad/s
   error1(k+1)= w1_ref-w1(k);
   error2(k+1)= w2_ref-w2(k);
   integral1(k+1)=integral1(k)+(Ki*T*(error1(k+1)+error1(k)))/2-Ks*(w1_p-w1(k));  
   integral2(k+1)=integral2(k)+(Ki*T*(error2(k+1)+error2(k)))/2-Ks*(w2_p-w2(k));
   Te1=Kp*error1(k+1)+integral1(k+1);
   Te2=Kp*error2(k+1)+integral2(k+1);
   w1(k+1)=(T/J)*Te1+w1(k)-(T/J)*b*w1(k);
   w2(k+1)=(T/J)*Te2+w2(k)-(T/J)*b*w2(k);
   w1_p=w1(k+1);
   w2_p=w2(k+1);
   w1_rpm=w1(k+1)*9.5493;
   w2_rpm=w2(k+1)*9.5493;
   force1(k)=q*(w1_rpm^2); %Thrust Force of BLDC1 depends on the RPM of the speed 
   force2(k)=q*(w2_rpm^2);
   ua=[force1(k);force2(k)];

  a_bar=(gr*(m1*l1-m2*l2+mb*l_b(k)))/(m1*(l1^2)+m2*(l2^2)+mb*(l_b(k)^2));
  b_bar=(l1)/(m1*(l1^2)+m2*(l2^2)+mb*(l_b(k)^2));
  c_bar=(l2)/(m1*(l1^2)+m2*(l2^2)+mb*(l_b(k)^2));

  B_d=[-b_bar*(T^2)/2 c_bar*(T^2)/2;-b_bar*T c_bar*T];

   x(:,k+1)=A*x(:,k)+B_d*ua+[(T^2)/2;T]*a_bar;%System Response in the presence of disturbance and optimal control inputs
  
   %if applied force is greater than static force, the mass will move otherwise it will not move
  Fb=abs(mb*gr*sin(x(1,k)));
  FN=abs(u_s*mb*gr*cos(x(1,k)));
  if Fb>FN 
  l_b(k+1)=l_b(k)+T*vb(k)+(((T^2)*gr*x(1,k))/2)-(((T^2)*gr*u_k)/2);
  vb(k+1)=vb(k)+(T*gr*x(1,k))-(T*gr*u_k);
  if l_b(k+1)>l1
      l_b(k+1)=l1;
      vb(k+1)=0;
  elseif l_b(k+1)<-l2
      l_b(k+1)=-l2;
      vb(k+1)=0;
  end
  else
      l_b(k+1)=0;
      vb(k+1)=0;

  end
   
   zk=z;
err=xr(1)-x(1,k);
 sum_err(k+1)=sum_err(k)+k*abs(err)^2;
end  
toc
x(1,:)=x(1,:)*180/pi;

n = 0:T:Time;
plot(n,x(1,:))
title('Balancing using LMPC control')
xlabel('Time (sec)')
ylabel('angle (degree)')
legend('Simulated')

figure
plot(n,x(2,:))
xlabel('Time (sec)')
ylabel('angular velocity (rad/s)')
legend('Simulated')

figure
u_p=[0 u(1,:)];
plot(n,u_p)

hold on
u_p_2=[0 u(2,:)];
plot(n,u_p_2)
uin=c_bar*u_p_2-b_bar*u_p;
hold on
plot(n,uin)
title('Control Input')
xlabel('Time (sec)')
ylabel('Torque(Nm)')
legend('Simulated')
