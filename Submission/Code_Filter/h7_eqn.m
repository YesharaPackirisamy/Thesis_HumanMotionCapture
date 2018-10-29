function h7_eqn = h7_eqn(L4,L5,L6,theta1,theta2)
%H7_EQN
%    H7_EQN = H7_EQN(L4,L5,L6,THETA1,THETA2)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:33:17

t2 = theta1+theta2;
t3 = cos(t2);
t4 = sin(t2);
t5 = cos(theta1);
t6 = sin(theta1);
h7_eqn = [0.0;(L4.*6.882681645895632e21+L6.*t3.*6.882681645895632e21+L5.*t5.*6.882681645895632e21+L6.*t4.*8.167050230778512e21+L5.*t6.*8.167050230778512e21+5.716935161544958e20)./(L4.*1.440932386346743e19+L6.*t3.*1.440932386346743e19+L5.*t5.*1.440932386346743e19+L6.*t4.*2.515156836085391e17+L5.*t6.*2.515156836085391e17+1.760609785259774e16)-4.873995693270018e2];
