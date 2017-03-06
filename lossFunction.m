function [L]=lossFunction(Cnew,Inew,Anew,K_change,C_target,I_target,A_target,K_target,wC,wI,wA,wK)


L_C=max(0,C_target-Cnew);
L_I=max(0,I_target-Inew);
L_A=max(0,A_target-Anew);
L_K=max(0,K_change-K_target); 




L=wC*L_C + wI*L_I + wA*L_A + wK*L_K; 