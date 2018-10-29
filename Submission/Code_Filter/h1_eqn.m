function h1_eqn = h1_eqn(L1,L2,theta1)
%H1_EQN
%    H1_EQN = H1_EQN(L1,L2,THETA1)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:33:00

t2 = cos(theta1);
t3 = sin(theta1);
h1_eqn = [0.0;(L1.*7.58692698884342e19+L2.*t2.*7.58692698884342e19-L2.*t3.*5.22724259375581e19+3.659069815629067e18)./(L1.*1.154436750835558e17+L2.*t2.*1.154436750835558e17+L2.*t3.*3.31029411452487e16-2.317205880167409e15)-4.872934537130378e2];
