close all;
clear all;
load('run');
%load('Test');
%  start_f=180;
% end_f=460;
% 
%  start_f=20;
% end_f=300;
% 
  start_f=1;
  end_f=301;

%  start_f=40;
%  end_f=340;
N=end_f-start_f;
index=start_f;
phiBody=xSmooth(15,:);
% theta_3=filterStates(11,:);
% theta_4=filterStates(12,:);
thetaBody=xSmooth(16,:);
psiBody=xSmooth(17,:);
theta_1=xSmooth(18,:);
theta_2=xSmooth(19,:);
theta_3=xSmooth(20,:);
theta_4=xSmooth(21,:);
[ p1,p2,p3,p4,p5,p6,p7,p8,p9,p10] = getPositions();
% gui=uicontrol('Style','slider','Min',0,'Max',N,'Value',0,'SliderStep',[1/N,1/N]);
%btn = uicontrol('Style', 'pushbutton', 'String', 'Stop'); 

writerObj = VideoWriter('Filter_run.avi');%create the video file

open(writerObj);%open the video

for i=start_f:1:end_f%loop through data
    cla;
    index=i;

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
view([0 0])
xlim([-1 2]);
ylim([-1 2]);
zlim([-2 2]);
title('Filter Camera Reconstruction for Run');
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
  

        daspect([1 1 1])%set the aspect ratio

        set(gcf, 'Position', get(0, 'Screensize'));%make it full screen

        writeVideo(writerObj, getframe(gcf));     % write the current image

end

close(writerObj);%close the file