close all;
%theta_1=[0,0,0,0];
%  theta_2=[1.78,1.57,1.28,1.78];
%  theta_3=[-0.3,0,0.27,-0.3];
% theta_1=filterStates(9,:);
% theta_2=filterStates(10,:);
% 
% thetaBody=ones(391,1)*1.57;
% phiBody=filterStates(15,:);
% thetaBody=filterStates(16,:);
% psiBody=filterStates(17,:);
% theta_1=filterStates(18,:);
% theta_2=filterStates(19,:);
% theta_3=filterStates(20,:);
% theta_4=filterStates(21,:);

phiBody=xSmooth(15,:);
% theta_3=filterStates(11,:);
% theta_4=filterStates(12,:);
thetaBody=xSmooth(16,:);
psiBody=xSmooth(17,:);
theta_1=xSmooth(18,:);
theta_2=xSmooth(19,:);
theta_3=xSmooth(20,:);
theta_4=xSmooth(21,:);

% theta_1=ones(391,1)*0;
% theta_2=ones(391,1)*0;
% theta_3=ones(391,1)*0;
% theta_4=ones(391,1)*0;

% L1=0.14;
% L2=0.465;
% L3=0.66;
% N=4;
%N=315;
index=1;
% gui=uicontrol('Style','slider','Min',0,'Max',N,'Value',0,'SliderStep',[1/N,1/N]);
gui=uicontrol('Style','text');
%btn = uicontrol('Style', 'pushbutton', 'String', 'Stop'); 
temp=1;
%figure(8);
[ p1,p2,p3,p4,p5,p6,p7,p8,p9,p10] = getPositions();
for i=1:1:N-1
% sliderVal=round(gui.Value)+1;
% 
% if(sliderVal~=temp)
%     cla;
%     temp=sliderVal;
% end
pause(0.001);
cla;
%index=index+50;
index=index+1;
phi_body=phiBody(index);
theta_body=thetaBody(index)+3.14;
psi_body=psiBody(index);
theta1=theta_1(index);
theta2=theta_2(index);
theta3=theta_3(index);
theta4=theta_4(index);

[p1,p2,p3,p4,p5,p6,p7,p8,p9,p10] = getPositions();
p1=subs(p1);p2=subs(p2);p3=subs(p3);p7=subs(p7);p8=subs(p8);

h=plot3([0;p1(1)],[0;p1(2)],[0;p1(3)]); 
%c=h.Color;
grid on;
xlim([-1 2]);
ylim([-1 2]);
zlim([-2 2]);
title('Graph showing angle theta tracked with EKF');
xlabel('x'); % x-axis label
ylabel('y');
zlabel('z');
h.Color='green';
h.LineWidth=1;
h.Marker='o';
h.MarkerSize=3;
h.MarkerEdgeColor='black';
h.MarkerFaceColor='black';
hold on

i=plot3([p1(1);p2(1)],[p1(2);p2(2)],[p1(3);p2(3)]);
i.Color='blue';
i.LineWidth=1;
i.Marker='o';
i.MarkerSize=3;
i.MarkerEdgeColor='black';
i.MarkerFaceColor='black';

j=plot3([p2(1);p3(1)],[p2(2);p3(2)],[p2(3);p3(3)]);
j.Color='m';
j.LineWidth=1;
j.Marker='o';
j.MarkerSize=3;
j.MarkerEdgeColor='black';
j.MarkerFaceColor='black';

k=plot3([p1(1);p7(1)],[p1(2);p7(2)],[p1(3);p7(3)]);
k.Color='red';
k.LineWidth=1;
k.Marker='o';
k.MarkerSize=3;
k.MarkerEdgeColor='black';
k.MarkerFaceColor='black';

l=plot3([p7(1);p8(1)],[p7(2);p8(2)],[p7(3);p8(3)]);
l.Color='c';
l.LineWidth=1;
l.Marker='o';
l.MarkerSize=3;
l.MarkerEdgeColor='black';
l.MarkerFaceColor='black';

end