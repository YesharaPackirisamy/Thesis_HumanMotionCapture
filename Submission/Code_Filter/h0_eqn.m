function h0_eqn = h0_eqn(dphi_body,dpsi_body,dtheta_body,phi_body,psi_body,theta_body)
%H0_EQN
%    H0_EQN = H0_EQN(DPHI_BODY,DPSI_BODY,DTHETA_BODY,PHI_BODY,PSI_BODY,THETA_BODY)

%    This function was generated by the Symbolic Math Toolbox version 7.0.
%    22-Oct-2018 17:32:57

t2 = cos(psi_body);
t3 = sin(theta_body);
t4 = cos(theta_body);
t5 = sin(phi_body);
t6 = sin(psi_body);
t7 = cos(phi_body);
h0_eqn = [t2.*t3.*(1.13e2./5.0)+t2.*t4.*(3.3e1./1.0e2)-t3.*t5.*t6.*(3.3e1./1.0e2)+t4.*t5.*t6.*(1.13e2./5.0);t3.*t6.*(1.13e2./5.0)+t4.*t6.*(3.3e1./1.0e2)+t2.*t3.*t5.*(3.3e1./1.0e2)-t2.*t4.*t5.*(1.13e2./5.0);t7.*(t3.*3.3e1-t4.*2.26e3).*(-1.0./1.0e2);dphi_body.*t4-dpsi_body.*t3.*t7;dtheta_body+dpsi_body.*t5;dphi_body.*t3+dpsi_body.*t4.*t7];
