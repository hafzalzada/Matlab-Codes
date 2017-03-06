
DF = 0.8;
ShockSize = 2;
S_val = 10;
Step_Size=100; 

wC = 0.2;
wI =0.2;
wA =0.3; 
wK =0.3;


%Initial Values for C and I for StateDynamics File
C0 = 1;
I0 = 1;
A0=0;
K0=0;

%Target values for C, I, A and K for Loss Function File
C_target = 20;
I_target =20;
A_target = 30;
K_target = 30;

C_init = C_target;
I_init = I_target;
A_init =A_target;
K_init = K_target;


%Values for C, I, A and K in StateDynamics File (= deviations in paper)

C = C_init; 
I = I_init; 
A = A_init;
K = K_init; 

Steps = Step_Size; %How far the x axis is going
S = S_val; %How far the agent can see
Impulse1 = zeros(Steps,9);


%Persistances of shocks with respect to C, I, A and K
rho_C=0.6;
rho_I=0;
rho_A=0;
rho_K=0;



%Shocks vectors with respect to C, I, A and K
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z=zeros(Steps, 9);
for i=2:Steps
    if C<0
        
        alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
        
        
    end
    C;
    if i==2
        
        %Shocks with respect to C, I, A and K
        mu_Ct=ShockSize; 
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z(i,:) = [x C I A K R y z l];
    
end
Z;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM WITH NO SHOCK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Values for C, I, A and K in StateDynamics File (= deviations in paper)



C = C_init;
I = I_init; 
A = A_init;
K = K_init; 

Steps=Step_Size;
S = S_val;

Impulse1 = zeros(Steps,9);
%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0;
rho_K=0;
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);
Z0=zeros(Steps,9);

for i=2:Steps
    if C<0
          alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
    end
    C;
    
    
    if i==2
        
        % Shocks with respect to C, I, A and K
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
   
        

        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z0(i,:) = [x C I A K R y z l];
    
end
Z0;




for j = 1:Steps
    
    
    Impulse1(j,:)=Z(j,:)-Z0(j,:);
    
    if Impulse1(j,:)< 1e-4
        Impulse1(j,:)=0;
    end
    
end
Impulse1;




%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init;
I = I_init;
A = A_init;
K = K_init; 





Steps=Step_Size; 
S = S_val;

Impulse2 = zeros(Steps,9);

%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0.4;
rho_A=0;
rho_K=0;

%Shocks vectors with respect to C, I, A and K
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);


Z2=zeros(Steps, 9);
for i=2:Steps
    if C<0
            alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
        
        
        
    end
    C;
    if i==2
        
        %Shocks with respect to C, I, A and K
        mu_Ct=0; 
        mu_It=ShockSize;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    Z2(i,:) = [x C I A K R y z l];
    
end
Z2;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM WITH NO SHOCK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init; 
I = I_init; 
A = A_init;
K = K_init;  
Steps=Step_Size;
S = S_val;

Impulse2 = zeros(Steps,9);

%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0;
rho_K=0;

%Shocks vectors with respect to C, I, A and K
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z02=zeros(Steps, 9);
for i=2:Steps
    if C<0
            alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
    end
    C;
    
    
    if i==2
        
        %Shocks with respect to C, I, A and K
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z02(i,:) = [x C I A K R y z l ];
    
end




for j = 2:Steps
    
    
    Impulse2(j,:)=Z2(j,:)-Z02(j,:);
    
    
    if Impulse2(j,:)< 1e-4
        Impulse2(j,:)=0;
    end
    
end
Impulse2;



%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init; 
I = I_init;
A = A_init; 
K = K_init; 

Steps=Step_Size; 
S = S_val;
Impulse3 = zeros(Steps,9);
%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0.8;
rho_K=0;



%Shocks vectors with respect to C, I, A and K
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z3=zeros(Steps, 9);

for i=2:Steps
    if C<0
        
        alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
    end
    C;
    if i==2
        
        %Shocks with respect to C, I, A and K
        mu_Ct=0; %Considering shock to C only
        mu_It=0;
        mu_At=ShockSize;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R,y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z3(i,:) = [x C I A K R y z l ];
    
end
Z3;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM WITH NO SHOCK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init;
I = I_init; 
A = A_init;
K = K_init; 




Steps=Step_Size; 
S = S_val; 
Impulse3 = zeros(Steps,9);

%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0;
rho_K=0;
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z03 =zeros(Steps, 9);
for i=2:Steps
    
    if C<0
            alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
    end
    C;
    
    if i==2
        
        % Shocks with respect to C, I, A and K
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R, y,z,l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z03(i,:) = [x C I A K R y z l ];
    
end
Z03;




for j = 1:Steps
    
    
    Impulse3(j,:)=Z3(j,:)- Z03(j,:);
    if Impulse3(j,:)< 1e-4
        Impulse3(j,:)=0;
    end
    
end
Impulse3;



%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init; 
I = I_init;
A = A_init;
K = K_init; 



Steps=Step_Size;
S = S_val; 
Impulse4 = zeros(Steps,9);

%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0;
rho_K=0.4;



%Shocks vectors with respect to C, I, A and K
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z4=zeros(Steps, 9);

for i=2:Steps
    if C<0
            alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
    end
    C;
    
    
    if i==2
        
        %Shocks with respect to C, I, A and K
        mu_Ct=0; 
        mu_It=0;
        mu_At=0;
        mu_Kt=ShockSize;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R ,y, z, l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z4(i,:) = [x C I A K R y z l ];
    
end
Z4;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% SYSTEM WITH NO SHOCK %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%




%Values for C, I, A and K in StateDynamics File (= deviations in paper)


C = C_init; 
I = I_init;
A = A_init;
K = K_init; 






Steps=Step_Size; 
S = S_val;
Impulse4 = zeros(Steps,9);

%Persistances of shocks with respect to C, I, A and K
rho_C=0;
rho_I=0;
rho_A=0;
rho_K=0;
S_Ct= zeros(Steps,1);
S_It= zeros(Steps,1);
S_At= zeros(Steps,1);
S_Kt= zeros(Steps,1);

S0_Ct= zeros(Steps,1);
S0_It= zeros(Steps,1);
S0_At= zeros(Steps,1);
S0_Kt= zeros(Steps,1);

Z04=zeros(Steps, 9);
for i=2:Steps
    if C<0
        
          alpha = 0.8;
        beta = 0.3;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.6;
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1; 
        delta = 0.3;
        epsilon = 0.8; 
        psi = 0.1;
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8;
        lambda=0.2;
        eta= 0.1;
        
        
    elseif C>0
        
        
        %Effectiveness of variables in Cnew
        
        alpha = 0.1;
        beta = 0.5;
        tau =  0.8;
        omega=0.1;
        
        %Effectiveness of variables in Inew
        sigma = 0.1;
        theta = 0.1;
        iota = 0.4; 
        phi_1 = 0.6;
        phi_2 = 0.5;
        %w_I1 and w_I2 should add to give 1
        w_I1=0.8;
        w_I2=0.2;
        
        
        %Effectiveness of variables in Anew
        
        gamma = 0.1;
        delta = 0.8; 
        epsilon = 0.1; 
        psi = 0.1; 
        
        %Variables for  Knew
        nu=0.3;
        chi=0.8; 
        lambda=0.2; 
        eta= 0.1; 
        
        
    end
    C;
    
    
    if i==2
        
        % Shocks with respect to C, I, A and K
        mu_Ct=0; %Considering shock to C only
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(1)=0;
        S_It(1)=0;
        S_At(1)=0;
        S_Kt(1)=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R, y, z, l]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
    else
        mu_Ct=0;
        mu_It=0;
        mu_At=0;
        mu_Kt=0;
        
        S_Ct(i)=rho_C*S_Ct(i-1)+mu_Ct;
        S_It(i)=rho_I*S_It(i-1)+mu_It;
        S_At(i)=rho_A*S_At(i-1)+mu_At;
        S_Kt(i)=rho_K*S_Kt(i-1)+mu_Kt;
        
        [x,C,I,A,K,R]=optimizationExpectedUtility(S,DF, C,I,A,K,C0,I0,A0,K0,C_target, I_target, A_target, K_target, nu, chi, lambda, eta,alpha,beta,tau, omega, theta, iota, sigma,w_I1, w_I2, phi_1, phi_2, gamma, delta, epsilon, psi, S_Kt(i), S_Ct(i), S_It(i),S_At(i),S0_Ct(i), S0_It(i), S0_At(i), S0_Kt(i), wC,wI,wA,wK);
        
    end
    
    Z04(i,:) = [x C I A K R y z l];
    
end
Z04;




for j = 1:Steps
    
    Impulse4(j,:)=Z4(j,:)-Z04(j,:);
    if Impulse4(j,:)< 1e-4
        Impulse4(j,:)=0;
    end
    
end
Impulse4;


hold all

figure(1)

subplot(3,2,1); semilogx([Impulse1(:,2) Impulse2(:,2)]);title('C')
legend('ImpulseC','ImpulseI')
subplot(3,2,2); semilogx([Impulse1(:,3) Impulse2(:,3)]);title('I')
legend('ImpulseC','ImpulseI')
subplot(3,2,3); semilogx([Impulse1(:,4) Impulse2(:,4)]);title('A')
legend('ImpulseC','ImpulseI')
subplot(3,2,4); semilogx([Impulse1(:,5) Impulse2(:,5)]);title('K')
legend('ImpulseC','ImpulseI')
subplot(3,2,5); semilogx([Impulse1(:,6) Impulse2(:,6)]);title('R')
legend('ImpulseC','ImpulseI')

figure(2)

subplot(3,2,1); semilogx([ Impulse3(:,2), Impulse4(:,2)]);title('C')
legend('ImpulseA', 'ImpulseK')
subplot(3,2,2); semilogx([Impulse3(:,3), Impulse4(:,3)]);title('I')
legend('ImpulseA', 'ImpulseK')
subplot(3,2,3); semilogx([Impulse3(:,4), Impulse4(:,4)]);title('A')
legend('ImpulseA', 'ImpulseK')
subplot(3,2,4); semilogx([Impulse3(:,5), Impulse4(:,5)]);title('K')
legend('ImpulseA', 'ImpulseK')
subplot(3,2,5); semilogx([Impulse3(:,6), Impulse4(:,6)]);title('R')
legend('ImpulseA', 'ImpulseK')
