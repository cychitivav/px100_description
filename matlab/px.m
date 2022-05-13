clc, clear, close all
%% Robot parameters
q1 = 1;
q2 = 1;
q3 = 1;
q4 = 1;

L1 = 44.5; %mm
L2 = 101; %mm
L3 = 101; %mm
L4 = 109; %mm (TCP)
Lm = 31.5; %mm
%% Robot creation 
% DH = [THETA D A ALPHA SIGMA OFFSET] 
DH = [q1	L1	0	            -pi/2	0	0;
      q2	0	sqrt(L2^2+Lm^2)	0	    0	-atan(L2/Lm);
      q3	0	L3	            0	        0	atan(L2/Lm);
      q4	0	L4	            0	    0	0];

for i=1:size(DH)
    L(i) = Link(DH(i,:));
end

PX = SerialLink(L,'name','Filoberta','tool',trotx(-pi/2)*troty(pi/2))
%% Transformation matrices
syms  q1 q2 q3 q4 q5 
HTB = vpa(PX.fkine([q1 q2 q3 q4]))
%% Plots
q  = [0 0 0 0];
figure
title('Home position')
PX.plot(q,'noa','jaxes','notiles','floorlevel',0,'noshadow')
axis tight
view([30 30])

figure
sgtitle('Others configurations')
subplot(2,2,1)
PX = SerialLink(PX,'name','PX1');
PX.plot([pi/6 pi/3 -pi 3*pi/4],'noa','notiles','floorlevel',0,'noshadow','noname')
axis tight
view([30 30])

subplot(2,2,2)
PX = SerialLink(PX,'name','PX2');
PX.plot([pi/3 pi/6 pi/7 3*pi/7],'noa','notiles','floorlevel',0,'noshadow','noname')
axis tight
view([30 30])

subplot(2,2,3)
PX = SerialLink(PX,'name','PX3');
PX.plot([-pi/6 pi/5 pi/4 pi],'noa','notiles','floorlevel',0,'noshadow','noname')
axis tight
view([-30 30])

subplot(2,2,4)
PX = SerialLink(PX,'name','PX4');
PX.plot([pi pi/9 -pi/2 -3*pi/4],'noa','notiles','floorlevel',0,'noshadow','noname')
axis tight
view([30 30])
