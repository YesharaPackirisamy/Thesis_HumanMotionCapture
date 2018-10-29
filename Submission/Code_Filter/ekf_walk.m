%% RUN EKF_Walk
% This script requires derive equations and the data from cameras and IMU

clear
close all
T = 0.01; %(100 Hz)              @frame rate=100fps; imu logged at 100Hz%
load('testData.mat');

load('CamMatrix.mat');

c_FL=[stereoParams_Front.CameraParameters1.IntrinsicMatrix(3,1);stereoParams_Front.CameraParameters1.IntrinsicMatrix(3,2)];     %principal point (specific to camera)
c_FR=[stereoParams_Front.CameraParameters2.IntrinsicMatrix(3,1);stereoParams_Front.CameraParameters2.IntrinsicMatrix(3,2)];
c_BL=[stereoParams_Back.CameraParameters1.IntrinsicMatrix(3,1);stereoParams_Back.CameraParameters1.IntrinsicMatrix(3,2)];     %principal point (specific to camera)
c_BR=[stereoParams_Back.CameraParameters2.IntrinsicMatrix(3,1);stereoParams_Back.CameraParameters2.IntrinsicMatrix(3,2)];             %principal point (specific to camera)

start_f=160;
end_f=550;

data_pt5_FL=[FL_pt5(start_f:end_f,1)-c_FL(1,:),FL_pt5(start_f:end_f,2)-c_FL(2,:)];
data_pt5_FR=[FR_pt5(start_f:end_f,1)-c_FR(1,:),FR_pt5(start_f:end_f,2)-c_FR(2,:)];
data_pt9_FL=[FL_pt9(start_f:end_f,1)-c_FL(1,:),FL_pt9(start_f:end_f,2)-c_FL(2,:)];
data_pt9_FR=[FR_pt9(start_f:end_f,1)-c_FR(1,:),FR_pt9(start_f:end_f,2)-c_FR(2,:)];
data_pt6_FL=[FL_pt6(start_f:end_f,1)-c_FL(1,:),FL_pt6(start_f:end_f,2)-c_FL(2,:)];
data_pt6_FR=[FR_pt6(start_f:end_f,1)-c_FR(1,:),FR_pt6(start_f:end_f,2)-c_FR(2,:)];
data_pt10_FL=[FL_pt10(start_f:end_f,1)-c_FL(1,:),FL_pt10(start_f:end_f,2)-c_FL(2,:)];
data_pt10_FR=[FR_pt10(start_f:end_f,1)-c_FR(1,:),FR_pt10(start_f:end_f,2)-c_FR(2,:)];
data_pt7_BL=[-(BL_pt7(start_f:end_f,1)-c_BL(1,:)),-(BL_pt7(start_f:end_f,2)-c_BL(2,:))];
data_pt7_BR=[-(BR_pt7(start_f:end_f,1)-c_BR(1,:)),-(BR_pt7(start_f:end_f,2)-c_BR(2,:))];
data_pt11_BL=[-(BL_pt11(start_f:end_f,1)-c_BL(1,:)),-(BL_pt11(start_f:end_f,2)-c_BL(2,:))];
data_pt11_BR=[-(BR_pt11(start_f:end_f,1)-c_BR(1,:)),-(BR_pt11(start_f:end_f,2)-c_BR(2,:))];
data_pt8_BL=[-(BL_pt8(start_f:end_f,1)-c_BL(1,:)),-(BL_pt8(start_f:end_f,2)-c_BL(2,:))];
data_pt8_BR=[-(BR_pt8(start_f:end_f,1)-c_BR(1,:)),-(BR_pt8(start_f:end_f,2)-c_BR(2,:))];
data_pt12_BL=[-(BL_pt12(start_f:end_f,1)-c_BL(1,:)),-(BL_pt12(start_f:end_f,2)-c_BL(2,:))];
data_pt12_BR=[-(BR_pt12(start_f:end_f,1)-c_BR(1,:)),-(BR_pt12(start_f:end_f,2)-c_BR(2,:))];
data_mag= [Magnetometer(start_f:end_f,1),0*Magnetometer(start_f:end_f,2),Magnetometer(start_f:end_f,3)];
data_gyro=[Gyroscope(start_f:end_f,1)-1.58,-Gyroscope(start_f:end_f,2)-0.2885,Gyroscope(start_f:end_f,3)+0.6927];

N =length(data_pt5_FL);
%% Constants
I = eye(21);%N by N where N is the number of states
%Lengths
L1= 0.24;
L2=0.46;
L3=0.485;
L4= 0.16;
L5=0.64;
L6=0.34;
L7=0.45;
L8=0.49;
L9=0.64;
L10=0.34;

g = -9.81;

%% Initial Values
states = zeros(21,1); 
i=1;
states(i) = 0;   i=i+1;     %ddphi_body
states(i) = 0;   i=i+1;     %ddtheta_body
states(i) = 0;   i=i+1;     %ddpsi_body
states(i) = 0;   i=i+1;     %ddtheta1             
states(i) = 0;   i=i+1;       %ddtheta2
states(i) = 0;   i=i+1;       %ddtheta3 
states(i) = 0;   i=i+1;       %ddtheta4
states(i) = 0;   i=i+1;      %dphi_body
states(i) = 0;   i=i+1;      %dtheta_body
states(i) = 0;   i=i+1;      %dpsi_body
states(i) = 0;   i=i+1;      %dtheta1
states(i) = 0;   i=i+1;      %dtheta2
states(i) = 0;   i=i+1;      %dtheta3
states(i) = 0;   i=i+1;      %dtheta4
states(i) = 0;   i=i+1;      %phi_body
states(i)=pi/2;  i=i+1;   %theta_body
states(i) = 0;   i=i+1;      %psi_body
states(i)=0;     i=i+1;      %theta1
states(i)=0;     i=i+1;      %theta2
states(i)=0;     i=i+1;      %theta3
states(i)=0;                %theta4

%% Filter Parameters - P, Q and R
% INITIAL COVARIANCE
covP=eye(21);
% 
% PROCESS NOISE
Q = struct;
Q.angAcc(1)=7853E-3.^2;
Q.angAcc(2)=7853E-3.^2;
Q.angAcc(3)=7853E-3.^2;
Q.angAcc(4)=785E-2.^2;
Q.angAcc(5)=785E-3.^2;
Q.angAcc(6)=785E-2.^2;
Q.angAcc(7)=785E-3.^2;

Q.angVel=Q.angAcc*0.01;
Q.angPos=Q.angVel*0.01;  
QMat = diag([Q.angAcc,Q.angVel, Q.angPos]);
% 
%MEASUREMENT NOISE
R_mag=0.3^2;
R_gyro=ones(1,3)*(1E-3)^2;
R1=20^2;
R2=20^2;
R3=20^2;
R4=20^2;
R5=20^2;
R6=20^2;
R7=20^2;
R8=20^2;

count1=0;count2=0;count3=0;count4=0;count5=0;count6=0;count7=0;
count8=0;count9=0;count10=0;count11=0;count12=0;
count13=0;count14=0;count15=0;count16=0;

for i=1:1:N
    %%PREDICTION
    H = [];
    h = [];
    h0=[NaN,NaN];
    h1=[NaN,NaN];
    h2=[NaN,NaN];
    h3=[NaN,NaN];
    h4=[NaN,NaN];
    h5=[NaN,NaN];
    h6=[NaN,NaN];
    h7=[NaN,NaN];
    h8=[NaN,NaN];
    h9=[NaN,NaN];
    h10=[NaN,NaN];
    h11=[NaN,NaN];
    h12=[NaN,NaN];
    h13=[NaN,NaN];
    h14=[NaN,NaN];
    h15=[NaN,NaN];
    h16=[NaN,NaN];
    
    temp = [];
    states = num2cell(states);
    [ddphi_body,ddtheta_body,ddpsi_body,ddtheta1, ddtheta2, ddtheta3, ddtheta4,dphi_body,dtheta_body,dpsi_body,dtheta1, dtheta2, dtheta3, dtheta4,phi_body,theta_body,psi_body,theta1, theta2, theta3, theta4]=deal(states{:});
    %projection of the state
    states=state_predict_function(T,ddphi_body,ddpsi_body,ddtheta1,ddtheta2,ddtheta3,ddtheta4,ddtheta_body,dphi_body,dpsi_body,dtheta1,dtheta2,dtheta3,dtheta4,dtheta_body,phi_body,psi_body,theta1,theta2,theta3,theta4,theta_body);%%%%%%%%%% T=0
    xPred(:,i) = states;
    %Jacobian of the predction update
    Fmatrix=Fmatrix_function(T);%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    bigPhi(:,:,i) = Fmatrix;
    % Project of the error covariance
    covP = (Fmatrix)*(covP*(Fmatrix')) + QMat;
    Ppred(:,:,i) = covP;
    %% %%%%%%%%UPDATE%%%%%%%%%%%%%%%
    % ALWAYS
    zk=[];
    z0=[data_mag(i,:)';data_gyro(i,:)'];
    z1 = [data_pt5_FL(i,:)'];
    z2= [data_pt5_FR(i,:)'];
    z3= [data_pt9_FL(i,:)'];
    z4= [data_pt9_FR(i,:)'];
    z5 =[data_pt7_BL(i,:)'];
    z6= [data_pt7_BR(i,:)'];
    z7= [data_pt11_BL(i,:)'];
    z8= [data_pt11_BR(i,:)'];
    z9 = [data_pt6_FL(i,:)'];
    z10= [data_pt6_FR(i,:)'];
    z11= [data_pt10_FL(i,:)'];
    z12= [data_pt10_FR(i,:)'];
    z13 =[data_pt8_BL(i,:)'];
    z14= [data_pt8_BR(i,:)'];
    z15= [data_pt12_BL(i,:)'];
    z16= [data_pt12_BR(i,:)'];
    temp=[];
    
    if ~isnan(z0(1))
    measurement=true;
    h0 = h0_eqn(dphi_body,dpsi_body,dtheta_body,phi_body,psi_body,theta_body);
    H0 = H0_matrix(dphi_body,dpsi_body,phi_body,psi_body,theta_body);
    h =  h0;
    H = H0;
    zk = z0;
    temp=[temp,R_mag,R_mag,R_mag,R_gyro];
    end
    
    if ~isnan(z1(1))
    measurement=true;
    count1=count1+1;
    h1 = h1_eqn(L1,L2,theta1);
    H1 = H1_matrix(L1,L2,theta1);
    h =[h; h1];
    H = [H;H1];
    zk = [zk;z1];
    temp=[temp,R1,R1];
    end
%    
    if ~isnan(z2(1))
    measurement=true;
    count2=count2+1;
    h2 = h2_eqn(L1,L2,theta1);
    H2 = H2_matrix(L1,L2,theta1);
    h = [h; h2];
    H = [H; H2];
    zk = [zk;z2];
    temp=[temp,R2,R2];
    end
%     
    if ~isnan(z3(1))
    measurement=true;
    count3=count3+1;
    h3 = h3_eqn(L1,L2,L3,theta1,theta2);
    H3 = H3_matrix(L1,L2,L3,theta1,theta2);
    h = [h;h3];
    H = [H;H3];
    zk = [zk;z3];
    temp=[temp,R3,R3];
    end
    
    if ~isnan(z4(1))
    measurement=true;
    count4=count4+1;
     h4 = h4_eqn(L1,L2,L3,theta1,theta2);
    H4 = H4_matrix(L1,L2,L3,theta1,theta2);
    h = [h;h4];
    H = [H;H4];
    zk = [zk;z4];
    temp=[temp,R4,R4];
    end
      
    if ~isnan(z5(1))
    measurement=true;
    count5=count5+1;
    h5 = h5_eqn(L4,L5,theta1);
    H5 = H5_matrix(L4,L5,theta1);
    h = [h; h5];
    H = [H; H5];
    zk = [zk;z5];
    temp=[temp,R5,R5];
    end    
      
    if ~isnan(z6(1))
    measurement=true;
   count6=count6+1;
    h6 = h6_eqn(L4,L5,theta1);
    H6 = H6_matrix(L4,L5,theta1);
    h = [h; h6];
    H = [H; H6];
    zk = [zk;z6];
    temp=[temp,R6,R6];
    end
    
    if ~isnan(z7(1))
    measurement=true;
    count7=count7+1;
    h7 = h7_eqn(L4,L5,L6,theta1,theta2);
    H7 = H7_matrix(L4,L5,L6,theta1,theta2);
    h = [h;h7];
    H = [H;H7];
    zk = [zk;z7];
    temp=[temp,R7,R7];
    end  
    
    if ~isnan(z8(1))
    measurement=true;
   count8=count8+1;
    h8 = h8_eqn(L4,L5,L6,theta1,theta2);
    H8 = H8_matrix(L4,L5,L6,theta1,theta2);
    h = [h;h8];
    H = [H;H8];
    zk = [zk;z8];
    temp=[temp,R8,R8];
    end
%     
    if ~isnan(z9(1))
    measurement=true;
    count9=count9+1;
    h9 = h9_eqn(L1,L7,theta3);
    H9 = H9_matrix(L1,L7,theta3);
    h = [h;h9];
    H = [H;H9];
    zk = [zk;z9];
    temp=[temp,R1,R1];
    end
%    
    if ~isnan(z10(1))
    measurement=true;
    count10=count10+1;
    h10 = h10_eqn(L1,L7,theta3);
    H10 = H10_matrix(L1,L7,theta3);
    h = [h;h10];
    H = [H;H10];
    zk = [zk;z10];
    temp=[temp,R2,R2];
    end
%     
    if ~isnan(z11(1))
    measurement=true;
    count11=count11+1;
    h11 = h11_eqn(L1,L7,L8,theta3,theta4);
    H11 = H11_matrix(L1,L7,L8,theta3,theta4);
    h = [h;h11];
    H = [H;H11];
    zk = [zk;z11];
    temp=[temp,R3,R3];
    end
    
    if ~isnan(z12(1))
    count12=count12+1;
    measurement=true;
    h12 = h12_eqn(L1,L7,L8,theta3,theta4);
    H12 = H12_matrix(L1,L7,L8,theta3,theta4);
    h = [h;h12];
    H = [H;H12];
    zk = [zk;z12];
    temp=[temp,R4,R4];
    end
      
    if ~isnan(z13(1))
    measurement=true;
    count13=count13+1;
    h13 = h13_eqn(L4,L9,theta3);
    H13 = H13_matrix(L4,L9,theta3);
    h = [h; h13];
    H = [H; H13];
    zk = [zk;z13];
    temp=[temp,R5,R5];
    end    
      
    if ~isnan(z14(1))
    measurement=true;
    count14=count14+1;
      h14 = h14_eqn(L4,L9,theta3);
    H14 = H14_matrix(L4,L9,theta3);
    h = [h; h14];
    H = [H; H14];
    zk = [zk;z14];
    temp=[temp,R6,R6];
    end
    
    if ~isnan(z15(1))
    measurement=true;
    count15=count15+1;
    h15 = h15_eqn(L4,L9,L10,theta3,theta4);
    H15 = H15_matrix(L4,L9,L10,theta3,theta4);
    h = [h;h15];
    H = [H;H15];
    zk = [zk;z15];
    temp=[temp,R7,R7];
    end  
    
    if ~isnan(z16(1))
    measurement=true;
    count16=count16+1;
    h16 = h16_eqn(L4,L9,L10,theta3,theta4);
    H16= H16_matrix(L4,L9,L10,theta3,theta4);

    h = [h;h16];
    H = [H;H16];
    zk = [zk;z16];
    temp=[temp,R8,R8];
    end
    big_h0(:,i) = h0;
    big_z0(:,i) = z0; 
    big_h1(:,i) = h1;
    big_h2(:,i)=h2;
    big_z1(:,i) = z1;
    big_z2(:,i)=z2;
    big_h3(:,i) = h3;
    big_z3(:,i) = z3;
    big_h4(:,i) = h4;
    big_z4(:,i) = z4;
    big_h5(:,i) = h5;
    big_z5(:,i)=z5;
    big_h6(:,i) = h6;
    big_z6(:,i)=z6;
    big_h7(:,i) = h7;
    big_z7(:,i) = z7;
    big_h8(:,i) = h8;
    big_z8(:,i) = z8;
    big_h9(:,i) = h9;
    big_z9(:,i) = z9;
    big_h10(:,i) = h10;
    big_z10(:,i)=z10;
    big_h11(:,i) = h11;
    big_z11(:,i)=z11;
    big_h12(:,i) = h12;
    big_z12(:,i) = z12;
    big_h13(:,i) = h13;
    big_z13(:,i) = z13;
    big_h14(:,i) = h14;
    big_z14(:,i) = z14;
    big_h15(:,i) = h15;
    big_z15(:,i) = z15;
    big_h16(:,i) = h16;
    big_z16(:,i) = z16;
      
    
    R=diag(temp);
    
    % Kalman Gain
    if(measurement==true)
        
    K=(covP*H')/(H*covP*H'+R);
    
    % Correction
    states=states+K*(zk-h);
    

    % New covariance
    covP=(I-K*H)*covP;
    
    %Store covariance for smoother
    Pcov(:,:,i) = covP;
    
    xEst(:,i)=states;
    end
   filterStates(:,i)=states;
   measurement=false;
end

figure(1);
subplot(2,2,1);
plot(big_h1(2,:));
hold on;
plot(big_z1(2,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Position tracked with EKF for FL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFL(rightKnee)','cameraDataFL(rightKnee)','Location','northeast');
%plot(theta_cam_new(:,1));

%figure(2);
subplot(2,2,2);
plot(big_h2(2,:));
hold on;
plot(big_z2(2,:));
hold on;
title('Graph showing Position tracked with EKF for FR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(rightKnee)','cameraDataFR(rightKnee)','Location','northeast');

%figure(3);
subplot(2,2,3);
plot(big_h3(2,:));
hold on;
plot(big_z3(2,:));
hold on;
title('Graph showing Position tracked with EKF for FL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFL(rightFoot)','cameraDataFL(rightFoot)','Location','northeast');

%figure(4);
subplot(2,2,4);
plot(big_h4(2,:));
hold on;
plot(big_z4(2,:));
hold on;
title('Graph showing Position tracked with EKF for FR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(rightFoot)','cameraDataFR(rightFoot)','Location','northeast');

figure(2);
subplot(2,2,1);
plot(big_h5(2,:));
hold on;
plot(big_z5(2,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateBL(rightCalf)','cameraDataBL(rightCalf)','Location','northeast');
%plot(theta_cam_new(:,1));

%figure(2);
subplot(2,2,2);
plot(big_h6(2,:));
hold on;
plot(big_z6(2,:));
hold on;
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(rightCalf)','cameraDataFR(rightCalf)','Location','northeast');

%figure(3);
subplot(2,2,3);
plot(big_h7(2,:));
hold on;
plot(big_z7(2,:));
hold on;
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(rightHeel)','cameraDataFR(rightHeel)','Location','northeast');

%figure(4);
subplot(2,2,4);
plot(big_h8(2,:));
hold on;
plot(big_z8(2,:));
hold on;
title('Graph showing Position tracked with EKF for BR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(rightHeel)','cameraDataFR(rightHeel)','Location','northeast');

figure(3);
subplot(2,2,1);
plot(big_h9(2,:));
hold on;
plot(big_z9(2,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Position tracked with EKF for FL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFL(lefttKnee)','cameraDataFL(leftKnee)','Location','northeast');
%plot(theta_cam_new(:,1));

%figure(2);
subplot(2,2,2);
plot(big_h10(2,:));
hold on;
plot(big_z10(2,:));
hold on;
title('Graph showing Position tracked with EKF for FR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(leftKnee)','cameraDataFR(lefttKnee)','Location','northeast');

%figure(3);
subplot(2,2,3);
plot(big_h11(2,:));
hold on;
plot(big_z11(2,:));
hold on;
title('Graph showing Position tracked with EKF for FL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFL(leftFoot)','cameraDataFL(leftFoot)','Location','northeast');

%figure(4);
subplot(2,2,4);
plot(big_h12(2,:));
hold on;
plot(big_z12(2,:));
hold on;
title('Graph showing Position tracked with EKF for FR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(leftFoot)','cameraDataFR(leftFoot)','Location','northeast');

figure(4);
subplot(2,2,1);
plot(big_h13(2,:));
hold on;
plot(big_z13(2,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateBL(leftCalf)','cameraDataBL(leftCalf)','Location','northeast');
%plot(theta_cam_new(:,1));

%figure(2);
subplot(2,2,2);
plot(big_h14(2,:));
hold on;
plot(big_z14(2,:));
hold on;
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(leftCalf)','cameraDataFR(leftCalf)','Location','northeast');

%figure(3);
subplot(2,2,3);
plot(big_h15(2,:));
hold on;
plot(big_z15(2,:));
hold on;
title('Graph showing Position tracked with EKF for BL');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(leftHeel)','cameraDataFR(leftHeel)','Location','northeast');

%figure(4);
subplot(2,2,4);
plot(big_h16(2,:));
hold on;
plot(big_z16(2,:));
hold on;
title('Graph showing Position tracked with EKF for BR');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('estimateFR(leftHeel)','cameraDataFR(leftHeel)','Location','northeast');

figure(6);
subplot(1,3,1);
plot(filterStates(8,:));
hold on;
%plot(theta_cam(:,1));
title('Graph showing Velocity (dphi-body) tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('rad/s');
legend('filterStates','Location','northeast');

subplot(1,3,2);
plot(filterStates(9,:));
hold on;
%plot(theta_cam(:,1));
title('Graph showing Velocity (dtheta-body) tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('rad/s');
legend('filterStates','Location','northeast');

subplot(1,3,3);
plot(filterStates(10,:));
hold on;
%plot(theta_cam(:,1));
title('Graph showing Velocity (dpsi-body) tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('rad/s');
legend('filterStates','Location','northeast');

figure(8);
subplot(2,2,1);
plot((rad2deg(filterStates(18,:))));
hold on;
title('Graph showing angle theta1 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta1','Location','northeast');

subplot(2,2,2);
plot((rad2deg(filterStates(19,:))));
hold on;
title('Graph showing angle theta2 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta2','Location','northeast');

subplot(2,2,3);
plot((rad2deg(filterStates(20,:))));
hold on;
title('Graph showing angle theta3 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta3','Location','northeast');

subplot(2,2,4);
plot((rad2deg(filterStates(21,:))));
hold on;
title('Graph showing angle theta4 tracked with EKF');
xlabel('Sample time'); % x-axis label
ylabel('deg');
legend('Angle theta4','Location','northeast');

figure(9);
subplot(3,7,1);
plot((rad2deg(filterStates(1,:))));
hold on;
title('1-ddphi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,2);
plot((rad2deg(filterStates(2,:))));
hold on;
title('2-ddtheta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,3);
plot((rad2deg(filterStates(3,:))));
hold on;
title('3-ddpsi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,4);
plot((rad2deg(filterStates(4,:))));
hold on;
title('4-ddtheta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,5);
plot((rad2deg(filterStates(5,:))));
hold on;
title('5-ddtheta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,6);
plot((rad2deg(filterStates(6,:))));
hold on;
title('6-ddtheta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,7);
plot((rad2deg(filterStates(7,:))));
hold on;
title('7-ddtheta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,8);
plot((rad2deg(filterStates(8,:))));
hold on;
title('8-dphi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,9);
plot((rad2deg(filterStates(9,:))));
hold on;
title('9-dtheta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,10);
plot((rad2deg(filterStates(10,:))));
hold on;
title('10-dpsi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,11);
plot((rad2deg(filterStates(11,:))));
hold on;
title('11-dtheta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,12);
plot((rad2deg(filterStates(12,:))));
hold on;
title('12-dtheta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,13);
plot((rad2deg(filterStates(13,:))));
hold on;
title('13-dtheta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,14);
plot((rad2deg(filterStates(14,:))));
hold on;
title('21-dtheta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,15);
plot((rad2deg(filterStates(15,:))));
hold on;
title('15-phi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,16);
plot((rad2deg(filterStates(16,:))));
hold on;
title('16-theta(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,17);
plot((rad2deg(filterStates(17,:))));
hold on;
title('17-psi(body)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

subplot(3,7,18);
plot((rad2deg(filterStates(18,:))));
hold on;
title('18-theta(1)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle phi(body)','Location','northeast');

subplot(3,7,19);
plot((rad2deg(filterStates(19,:))));
hold on;
title('19-theta(2)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle theta(body)','Location','northeast');

subplot(3,7,20);
plot((rad2deg(filterStates(20,:))));
hold on;
title('20-theta(3)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');
subplot(3,7,21);
plot((rad2deg(filterStates(21,:))));
hold on;
title('21-theta(4)');
xlabel('Sample time'); % x-axis label
ylabel('deg');
%legend('Angle psi(body)','Location','northeast');

figure(10);
subplot(2,3,1);
plot(big_h0(1,:));
hold on;
plot(big_z0(1,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Body Angle residual in x');
xlabel('Sample time'); % x-axis label
ylabel('angle');
legend('estimate(phi-body)','IMUdata(phi-body)','Location','northeast');

subplot(2,3,2);
plot(big_h0(3,:));
hold on;
plot(big_z0(3,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing Body Angle residual in z');
xlabel('Sample time'); % x-axis label
ylabel('angle');
legend('estimate(psi-body)','IMUdata(psi-body)','Location','northeast');

subplot(2,3,3);
plot(big_h0(3,:));
hold on;
plot(big_z0(3,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing dtheta residual in x');
xlabel('Sample time'); % x-axis label
ylabel('angle');
legend('estimate(dphi-body)','IMUdata(dphi-body)','Location','northeast');

subplot(2,3,4);
plot(big_h0(4,:));
hold on;
plot(big_z0(4,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing residual in y');
xlabel('Sample time'); % x-axis label
ylabel('angle');
legend('estimate(dtheta-body)','IMUdata(dtheta-body)','Location','northeast');

subplot(2,3,5);
plot(big_h0(5,:));
hold on;
plot(big_z0(5,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing residual in z');
xlabel('Sample time'); % x-axis label
ylabel('angle');
legend('estimate(dpsi-body)','IMUdata(dpsi-body)','Location','northeast');

figure(13);
subplot(2,2,1);
plot(big_z1(2,:));
hold on;
plot(big_z2(2,:));
hold on;
%plot(rad2deg(phi_cam_left(:,1)));
title('Graph showing frequency of markers for Right Knee');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('RightKnee(FL)','RightKnee(FR)','Location','northeast');
%plot(theta_cam_new(:,1));

%figure(2);
subplot(2,2,2);
plot(big_z3(2,:));
hold on;
plot(big_z4(2,:));
hold on;
title('Graph showing frequency of markers for Right Foot');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('RightFoot(FL)','RightFoot(FR)','Location','northeast');

%figure(3);
subplot(2,2,3);
plot(big_z5(2,:));
hold on;
plot(big_z6(2,:));
hold on;
title('Graph showing frequency of markers for Right Calf');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('RightCalf(BL)','RightCalf(BR)','Location','northeast');

%figure(4);
subplot(2,2,4);
plot(big_z7(2,:));
hold on;
plot(big_z8(2,:));
hold on;
title('Graph showing frequency of markers for Right Heel');
xlabel('Sample time'); % x-axis label
ylabel('pixels');
legend('RightCalf(BL)','RightCalf(BR)','Location','northeast');