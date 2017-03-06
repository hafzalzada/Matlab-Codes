function [Cnew,Inew,Anew,K_change] = stateDynamics(C,I,A,R,C0,I0,nu, chi, lambda, eta,alpha,beta,tau,...
    omega, theta, iota, sigma,wi_1, wi_2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt, S_Ct, S_It,S_At)

%S_Kt, S_Ct, S_It and S_At are shocks to the system, 
%where S=S_t-1+mu_t (in runningfile)
%R= complexity of system 
%K=investment
K_change = nu*R-(chi*C+lambda*I+eta*A)-S_Kt;%

Cnew = -alpha*A+beta*K_change + tau*I - omega*R +C0 -S_Ct;
Inew = sigma*C+ - theta*R + iota*K_change + wi_1*phi_1*A - wi_2*phi_2*A+ I0 -S_It;

Anew = gamma*R + delta*K_change-epsilon*abs(C)-psi*I - S_At;