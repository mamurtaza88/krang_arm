clear;
close all;
%%
F1 = [zeros(3,3) eye(3);zeros(3,3) zeros(3,3)];
F2 = [zeros(3,3) eye(3);zeros(3,3) zeros(3,3)];
F3 = zeros(1,1);

%%
% F = blkdiag(F1,F2,F3);
% F_Speed_Full = blkdiag(F1,F2,F3);
F_Speed_Full = blkdiag(F1);
%%
G1 = [zeros(3,3);eye(3,3)];
G2 = [zeros(3,3);eye(3,3)];
G3 = eye(1);
%%

% G_Speed_Full = blkdiag(G1,G2,G3);
G_Speed_Full = blkdiag(G1);
 %%
 KpEE = 750;
 KvEE = 250;
 
 KpOr = 150.0;
 KvOr = 50;
 
 KpCOM = 5;
 KvCOM = 2;
 KvSpeedReg = 2;
 
 KpEEL = eye(3)*KpEE;
 KvEEL = eye(3)*KvEE;
 
 KpEER = eye(3)*KpEE;
 KvEER = eye(3)*KvEE;
 
 
 KpOrL = eye(3)*KpOr;
 KvOrL = eye(3)*KvOr;
 
 KpOrR = eye(3)*KpOr;
 KvOrR = eye(3)*KvOr;
 
 KpCOM1 = eye(3)*KpCOM;
 KvCOM1 = eye(3)*KvCOM;
 
%  [-KpTh -KvTh],[-KpEE -KvEE],[-KpOr -KvOr],[-KvReg*eye(2)]
%  K =  blkdiag([-KpEEL -KvEEL], [-KpEER -KpEER],[-KpOrL -KvOrL],[-KpOrR -KvOrR],[-KpCOM -KvCOM],[-KvSpeedReg*eye(3)]);  
% K =  blkdiag([-KpEEL -KvEEL], [-KpEER -KpEER],[-KpOrL -KvOrL],[-KpOrR -KvOrR],[-KpTh -KvTh],[-KvSpeedReg*eye(5)]);  
 
%  Fcl = F_Speed_Full+G_Speed_Full*K;
 
 %%

QQ_Full = eye(size(F_Speed_Full));

[P_Speed_Full,K,~,INFO] = icare(F_Speed_Full,G_Speed_Full,QQ_Full);

% P_Speed_Full = lyap(Fcl,QQ_Full);

P1_Full = round(P_Speed_Full,5);
% writematrix(P1_Pose,'P_space_JointPose.txt','Delimiter',' ')
% writematrix(F_Pose,'F_space_JointPose.txt','Delimiter',' ')
% writematrix(G_Pose,'G_space_JointPose.txt','Delimiter',' ')
% 
% writematrix(P1_Speed,'P_space_SpeedReg.txt','Delimiter',' ')
% writematrix(F_Speed,'F_space_SpeedReg.txt','Delimiter',' ')
% writematrix(G_Speed,'G_space_SpeedReg.txt','Delimiter',' ')


writematrix(P1_Full,'P_space_SpeedReg_Full.txt','Delimiter',' ')
writematrix(F_Speed_Full,'F_space_SpeedReg_Full.txt','Delimiter',' ')
writematrix(G_Speed_Full,'G_space_SpeedReg_Full.txt','Delimiter',' ')


% LfV_x = eta'*(F'*P+P*F)*eta;
% LgV_x = 2*eta'*P*G;
% V_x = eta'*P*eta;

% lambda_minQ = min(eig(QQ_Speed));
% lambda_maxP = max(eig(P1_Speed));

% lambda_minQ = min(eig(QQ_Pose));
% lambda_maxP = max(eig(P1_Pose));