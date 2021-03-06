function h3_eqn = h3_eqn(L1,L2,L3,theta1,theta2)
%H3_EQN
%    H3_EQN = H3_EQN(L1,L2,L3,THETA1,THETA2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:33:08

t2 = theta1+theta2;
t3 = cos(t2);
t4 = sin(t2);
t5 = cos(theta1);
t6 = sin(theta1);
h3_eqn = [0.0;(L1.*7.58692698884342e19+L3.*t3.*7.58692698884342e19+L2.*t5.*7.58692698884342e19-L3.*t4.*5.22724259375581e19-L2.*t6.*5.22724259375581e19+3.659069815629067e18)./(L1.*1.154436750835558e17+L3.*t3.*1.154436750835558e17+L2.*t5.*1.154436750835558e17+L3.*t4.*3.31029411452487e16+L2.*t6.*3.31029411452487e16-2.317205880167409e15)-4.872934537130378e2];
