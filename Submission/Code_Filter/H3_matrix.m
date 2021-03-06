function H3_matrix_eq = H3_matrix(L1,L2,L3,theta1,theta2)
%H3_MATRIX
%    H3_MATRIX_EQ = H3_MATRIX(L1,L2,L3,THETA1,THETA2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:33:09

t2 = theta1+theta2;
t3 = sin(t2);
t4 = sin(theta1);
t5 = cos(t2);
t6 = cos(theta1);
t7 = cos(theta2);
t8 = L1.*1.154436750835558e17;
t9 = L3.*t5.*1.154436750835558e17;
t10 = L3.*t3.*3.31029411452487e16;
t11 = L2.*t6.*1.154436750835558e17;
t12 = L2.*t4.*3.31029411452487e16;
t13 = t8+t9+t10+t11+t12-2.317205880167409e15;
t14 = 1.0./t13.^2;
H3_matrix_eq = reshape([0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,t14.*(L2.*t4.*-7.0-L3.*t3.*7.0+L2.^2.*1.0e2+L3.^2.*1.0e2+L1.*L2.*t6.*1.0e2+L1.*L3.*t5.*1.0e2+L2.*L3.*t7.*2.0e2).*(-8.546016931614521e34),0.0,L3.*t14.*(L3.*1.0e2-t3.*7.0+L1.*t5.*1.0e2+L2.*t7.*1.0e2).*(-8.546016931614521e34),0.0,0.0,0.0,0.0],[2,21]);
