%% Kalman Smoother
% This script will run the backward smoothing on the EKF estimates.
% It assumes that 'ekf_walk.m' or 'ekf_run.m' has been run.

%% Backward recursion
xSmooth = xEst;
PSmooth = Pcov;
for j = N-1:-1:1
    
    % Calculate Smoothing Gain
    A = Pcov(:,:,j)*bigPhi(:,:,j)'/(Ppred(:,:,j+1));
%     A(1,1)
    
    % Smoothing
    xSmooth(:,j) = xEst(:,j) + A*(xSmooth(:,j+1) - xPred(:,j+1));
    
    % Smoothed Covariance
    PSmooth(:,:,j) = Pcov(:,:,j) + A*(PSmooth(:,:,j+1) - Ppred(:,:,j+1))*A';
end

figure(9);
subplot(3,7,1);
plot((rad2deg(xSmooth(1,:))));
hold on;
title('1-ddphi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,2);
plot((rad2deg(xSmooth(2,:))));
hold on;
title('2-ddtheta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,3);
plot((rad2deg(xSmooth(3,:))));
hold on;
title('3-ddpsi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,4);
plot((rad2deg(xSmooth(4,:))));
hold on;
title('4-ddtheta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,5);
plot((rad2deg(xSmooth(5,:))));
hold on;
title('5-ddtheta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,6);
plot((rad2deg(xSmooth(6,:))));
hold on;
title('6-ddtheta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,7);
plot((rad2deg(xSmooth(7,:))));
hold on;
title('7-ddtheta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,8);
plot((rad2deg(xSmooth(8,:))));
hold on;
title('8-dphi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,9);
plot((rad2deg(xSmooth(9,:))));
hold on;
title('9-dtheta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,10);
plot((rad2deg(xSmooth(10,:))));
hold on;
title('10-dpsi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,11);
plot((rad2deg(xSmooth(11,:))));
hold on;
title('11-dtheta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,12);
plot((rad2deg(xSmooth(12,:))));
hold on;
title('12-dtheta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,13);
plot((rad2deg(xSmooth(13,:))));
hold on;
title('13-dtheta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,14);
plot((rad2deg(xSmooth(14,:))));
hold on;
title('21-dtheta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,15);
plot((rad2deg(xSmooth(15,:))));
hold on;
title('15-phi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,16);
plot((rad2deg(xSmooth(16,:))));
hold on;
title('16-theta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,17);
plot((rad2deg(xSmooth(17,:))));
hold on;
title('17-psi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,18);
plot((rad2deg(xSmooth(18,:))));
hold on;
title('18-theta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,19);
plot((rad2deg(xSmooth(19,:))));
hold on;
title('19-theta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,20);
plot((rad2deg(xSmooth(20,:))));
hold on;
title('20-theta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,21);
plot((rad2deg(xSmooth(21,:))));
hold on;
title('21-theta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle psi(body)','Location','northeast');

figure(8);
subplot(2,2,1);
plot((rad2deg(xSmooth(18,:))));
hold on;
title('Graph showing angle theta1 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta1','Angle theta1-smooth','Location','northeast');

subplot(2,2,2);
plot((rad2deg(xSmooth(19,:))));
hold on;
title('Graph showing angle theta2 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta2','Angle theta2-smooth','Location','northeast');

subplot(2,2,3);
plot((rad2deg(xSmooth(20,:))));
hold on;
title('Graph showing angle theta3 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta3','Angle theta3-smooth','Location','northeast');

subplot(2,2,4);
plot((rad2deg(xSmooth(21,:))));
hold on;
title('Graph showing angle theta4 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta4','Angle theta4-smooth','Location','northeast');
% 
% figure(9)
% subplot(3,1,1);
% plot((rad2deg(filterStates(15,:))));
% hold on;
% title('Graph showing angle phi(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle phi(body)','Location','northeast');
% 
% subplot(3,1,2);
% plot((rad2deg(xSmooth(16,:))));
% title('Graph showing angle theta(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta(body)','Angle theta(body)-smooth','Location','northeast');
% 
% subplot(3,1,3);
% plot((rad2deg(xSmooth(17,:))));
% title('Graph showing angle psi(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle psi(body)','Location','northeast');

% figure(8);
% subplot(2,2,1);
% plot((rad2deg(xSmooth(12,:))));
% hold on;
% title('Graph showing angle theta1 tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta1','Angle theta1-smooth','Location','northeast');
% 
% subplot(2,2,2);
% plot((rad2deg(xSmooth(13,:))));
% hold on;
% title('Graph showing angle theta2 tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta2','Angle theta2-smooth','Location','northeast');
% 
% subplot(2,2,3);
% plot((rad2deg(xSmooth(14,:))));
% hold on;
% title('Graph showing angle theta3 tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta3','Angle theta3-smooth','Location','northeast');
% 
% subplot(2,2,4);
% plot((rad2deg(xSmooth(15,:))));
% hold on;
% title('Graph showing angle theta4 tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta4','Angle theta4-smooth','Location','northeast');

% figure(9)
% subplot(3,1,1);
% plot((rad2deg(filterStates(15,:))));
% hold on;
% title('Graph showing angle phi(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle phi(body)','Location','northeast');
% 
% subplot(3,1,2);
% plot((rad2deg(xSmooth(16,:))));
% title('Graph showing angle theta(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle theta(body)','Angle theta(body)-smooth','Location','northeast');
% 
% subplot(3,1,3);
% plot((rad2deg(xSmooth(17,:))));
% title('Graph showing angle psi(body) tracked with EKF');
% xlabel('Sample time'); % x-axis label
% ylabel('deg');
% legend('Angle psi(body)','Location','northeast');