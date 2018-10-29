function H2_matrix_eq = H2_matrix(L1,L2,theta1)
%H2_MATRIX
%    H2_MATRIX_EQ = H2_MATRIX(L1,L2,THETA1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:33:03

t2 = cos(theta1);
t3 = sin(theta1);
H2_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,L2.*(L2.*1.0e2-t3.*7.0+L1.*t2.*1.0e2).*1.0./(L1.*1.154436750835558e17+L2.*t2.*1.154436750835558e17+L2.*t3.*3.31029411452487e16-2.317205880167409e15).^2.*(-8.520315680662861e34),0.0,0.0,0.0,0.0,0.0,0.0],[2,21]);
