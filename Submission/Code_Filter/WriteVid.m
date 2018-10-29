close all;
clear all;
load('3D_run');
%load('Test');
%  start_f=180;
% end_f=460;
 start_f=40;
 end_f=340;

N=end_f-start_f;

% gui=uicontrol('Style','slider','Min',0,'Max',N,'Value',0,'SliderStep',[1/N,1/N]);

pt1=[points(:,1),points(:,3),-points(:,2)];
pt2=[points(:,4),points(:,6),-points(:,5)];
pt3=[points(:,7),points(:,9),-points(:,8)];
pt4=[points(:,10),points(:,12),-points(:,11)];
pt5=[points(:,13),points(:,15),-points(:,14)];
pt6=[points(:,16),points(:,18),-points(:,17)];
pt7=[points(:,19),points(:,21),-points(:,20)];
pt8=[points(:,22),points(:,24),-points(:,23)];
pt9=[points(:,25),points(:,27),-points(:,26)];
pt10=[points(:,28),points(:,30),-points(:,29)];
pt11=[points(:,31),points(:,33),-points(:,32)];
pt12=[points(:,34),points(:,36),-points(:,35)];
pt13=[points(:,37),points(:,39),-points(:,38)];
pt14=[points(:,40),points(:,42),-points(:,41)];

%btn = uicontrol('Style', 'pushbutton', 'String', 'Stop'); 
 temp=1;
% 
pt1=[pt1(start_f:end_f,1),pt1(start_f:end_f,2),pt1(start_f:end_f,3)];
pt2=[pt2(start_f:end_f,1),pt2(start_f:end_f,2),pt2(start_f:end_f,3)];
pt3=[pt3(start_f:end_f,1),pt3(start_f:end_f,2),pt3(start_f:end_f,3)];
pt4=[pt4(start_f:end_f,1),pt4(start_f:end_f,2),pt4(start_f:end_f,3)];
pt5=[pt5(start_f:end_f,1),pt5(start_f:end_f,2),pt5(start_f:end_f,3)];
pt6=[pt6(start_f:end_f,1),pt6(start_f:end_f,2),pt6(start_f:end_f,3)];
pt7=[pt7(start_f:end_f,1),pt7(start_f:end_f,2),pt7(start_f:end_f,3)];
pt8=[pt8(start_f:end_f,1),pt8(start_f:end_f,2),pt8(start_f:end_f,3)];
pt9=[pt9(start_f:end_f,1),pt9(start_f:end_f,2),pt9(start_f:end_f,3)];
pt10=[pt10(start_f:end_f,1),pt10(start_f:end_f,2),pt10(start_f:end_f,3)];
pt11=[pt11(start_f:end_f,1),pt11(start_f:end_f,2),pt11(start_f:end_f,3)];
pt12=[pt12(start_f:end_f,1),pt12(start_f:end_f,2),pt12(start_f:end_f,3)];
pt13=[pt13(start_f:end_f,1),pt13(start_f:end_f,2),pt13(start_f:end_f,3)];
%pt13=[pt13(start_f,1),pt13(start_f,2),pt13(start_f:end_f,3)];
%pt13=[pt13(start_f:end_f,1)*0,pt13(start_f:end_f,3)*0,-pt13(start_f:end_f,2)*0];
pt14=[pt14(start_f:end_f,1),pt14(start_f:end_f,2),pt14(start_f:end_f,3)];

writerObj = VideoWriter('External_run.avi');%create the video file

open(writerObj);%open the video

for i=1:1:N-1%loop through data

    cla;%clear the figure

   index=i;

%    PLOT DATA HERE
h=plot3([pt13(index,1);(pt1(index,1)+pt2(index,1))/2],[pt13(index,2);(pt1(index,2)+pt2(index,2))/2],[pt13(index,3);(pt1(index,3)+pt2(index,3))/2]); 
%h=plot3([pt13(index,1);pt1(index,1)],[pt13(index,2);pt1(index,2)],[pt13(index,3);pt1(index,3)]); drawnow

% else
% %h=plot3([pt14(index,1)+0.07;(pt3(index,1)+pt4(index,1))/2],[pt14(index,2);(pt3(index,2)+pt4(index,2))/2],[pt14(index,3);(pt3(index,3)+pt4(index,3))/2]); drawnow
% h=plot3([pt14(index,1);pt7(index,1)],[pt14(index,2);pt7(index,2)],[pt14(index,3);pt7(index,3)]); drawnow

%end
%h=plot3([pt13(index,1)+0.07;(pt1(index,1)+pt2(index,1))/2],[0;0],[pt13(index,3);(pt1(index,3)+pt2(index,3))/2]); drawnow
%c=h.Color;
grid on;
view([0 0]);
 xlim([-5 5]);
 ylim([-9 -4]);
 zlim([-1 3]);
title('External Camera Reconstruction for Run');
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

%if ~isnan(pt13(index,1))
%h=plot3([pt13(index,1)+0.07;(pt1(index,1)+pt2(index,1))/2],[pt13(index,2);(pt1(index,2)+pt2(index,2))/2],[pt13(index,3);(pt1(index,3)+pt2(index,3))/2]); drawnow
%h=plot3([pt13(index,1);pt2(index,1)],[pt13(index,2);pt2(index,2)],[pt13(index,3);pt2(index,3)]); drawnow

%else
%h=plot3([pt14(index,1)+0.07;(pt3(index,1)+pt4(index,1))/2],[pt14(index,2);(pt3(index,2)+pt4(index,2))/2],[pt14(index,3);(pt3(index,3)+pt4(index,3))/2]); drawnow
% h=plot3([pt14(index,1);pt10(index,1)],[pt14(index,2);pt10(index,2)],[pt14(index,3);pt10(index,3)]); drawnow
% 
%end
h.Color='green';
h.LineWidth=1;
h.Marker='o';
h.MarkerSize=3;
h.MarkerEdgeColor='black';
h.MarkerFaceColor='black';
% 
% if ~isnan(pt1(index,1))
 %i=plot3([pt1(index,1);pt5(index,1)],[pt1(index,2);pt5(index,2)],[pt1(index,3);pt5(index,3)]);drawnow
i=plot3([(pt1(index,1)+pt2(index,1))/2;pt5(index,1)],[(pt1(index,2)+pt2(index,2))/2;pt5(index,2)],[(pt1(index,3)+pt2(index,3))/2;pt5(index,3)]);
% else
% i=plot3([pt3(index,1);pt7(index,1)],[pt3(index,2);pt7(index,2)],[pt3(index,3);pt7(index,3)]);drawnow
% end
 i.Color='blue';
 i.LineWidth=1;
 i.Marker='o';
 i.MarkerSize=3;
 i.MarkerEdgeColor='black';
 i.MarkerFaceColor='black';
% 
%if ~isnan(pt5(index,1))
j=plot3([pt5(index,1);pt9(index,1)],[pt5(index,2);pt9(index,2)],[pt5(index,3);pt9(index,3)]);
% j=plot3([pt7(index,1);pt11(index,1)],[pt7(index,2);pt11(index,2)],[pt7(index,3);pt11(index,3)]);drawnow  
% %j=plot3([pt5(index,1);pt9(index,1)],[0;0],[pt5(index,3);pt9(index,3)]);drawnow
%end
j.Color='m';
j.LineWidth=1;
j.Marker='o';
j.MarkerSize=3;
j.MarkerEdgeColor='black';
j.MarkerFaceColor='black';
% 
% if ~isnan(pt2(index,1))
 %k=plot3([pt2(index,1);pt6(index,1)],[pt2(index,2);pt6(index,2)],[pt2(index,3);pt6(index,3)]);drawnow
k=plot3([(pt1(index,1)+pt2(index,1))/2;pt6(index,1)],[(pt1(index,2)+pt2(index,2))/2;pt6(index,2)],[(pt1(index,3)+pt2(index,3))/2;pt6(index,3)]);
% else
% k=plot3([pt4(index,1);pt8(index,1)],[pt4(index,2);pt8(index,1)],[pt4(index,3);pt8(index,3)]);drawnow
% end
k.Color='red';
 k.LineWidth=1;
 k.Marker='o';
 k.MarkerSize=3;
 k.MarkerEdgeColor='black';
 k.MarkerFaceColor='black';

%if ~isnan(pt6(index,1))
l=plot3([pt6(index,1);pt10(index,1)],[pt6(index,2);pt10(index,2)],[pt6(index,3);pt10(index,3)]);
%else
% l=plot3([pt8(index,1);pt12(index,1)],[pt8(index,2);pt12(index,2)],[pt8(index,3);pt12(index,3)]);drawnow
%end
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