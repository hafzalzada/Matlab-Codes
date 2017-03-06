function [x,C,I,A,K_change,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,wi_1, wi_2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt, S_Ct, S_It,S_At,S0_Ct, S0_It, S0_At, S0_Kt, wC,wI,wA,wK)


% Choose x such that R = 1./(1 - x);

NUM = 10;

xCvec = linspace(eps,1-eps,NUM); 
xIvec = linspace(eps,1-eps,NUM); 
xAvec = linspace(eps,1-eps,NUM); 
xKvec = linspace(eps,1-eps,NUM); 

Loss = zeros(NUM,NUM,NUM,NUM); 

R_vals=zeros(NUM,NUM,NUM,NUM); 

for iC=1:NUM
      for iI = 1:NUM
          for iA = 1:NUM
              for iK = 1:NUM
                  
                  
                L = zeros(S,1);
                Cnew = C;
                Inew = I;
                Anew = A;
                Knew = K;
                

                for j=1:S


                    cur_R = xCvec(iC)*(Cnew-C_target)+xIvec(iI)*(Inew-I_target)+xAvec(iA)*(Anew-A_target)+xKvec(iK)*(Knew-K_target);
                    

                     
                    [Cnew,Inew,Anew,Knew] = stateDynamics(Cnew,Inew,Anew,cur_R,C0,I0,nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,wi_1, wi_2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt, S_Ct, S_It,S_At);



                    [L(j)]=(DF^j)*lossFunction(Cnew,Inew,Anew,Knew,C_target,I_target,A_target,K_target,wC,wI,wA,wK);

                end  


                Loss(iC, iI, iA, iK) = sum(L);
                R_vals(iC,iI,iA,iK) = cur_R;
                   
              end
          end
      end
end

[~, ind] = min(Loss(:)); %Finding at which position the  smallest value in Loss vector  is 

[iC, iI, iA, iK] = ind2sub(size(Loss), ind);


x=xCvec(iC);
y=xIvec(iI); 
z=xAvec(iA);
l=xKvec(iK);


R = R_vals(iC,iI,iA,iK);


[C,I,A,K_change] = stateDynamics(C,I,A,R,C0,I0,nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,wi_1, wi_2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt, S_Ct, S_It,S_At);


end