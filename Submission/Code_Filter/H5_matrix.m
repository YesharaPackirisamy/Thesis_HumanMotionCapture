function H5_matrix_eq = H5_matrix(L4,L5,theta1)
%H5_MATRIX
%    H5_MATRIX_EQ = H5_MATRIX(L4,L5,THETA1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:07:13

t2 = cos(theta1);
t3 = sin(theta1);
H5_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,L5.*(L5.*1.0e2+t3.*7.0+L4.*t2.*1.0e2).*1.0./(L4.*1.576952828265726e16+L5.*t2.*1.576952828265726e16+L5.*t3.*2.7805933092777e15+1.94641531649439e14).^2.*1.431488511176131e33,0.0,0.0,0.0,0.0,0.0,0.0],[2,21]);
