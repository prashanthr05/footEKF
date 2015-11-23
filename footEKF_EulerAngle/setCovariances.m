function [kalmanQParams,kalmanRParams,k,c] = setCovariances(filter)

% process model variances
% param ordering : [a_Q, omega_Q,	f_Q,	mu_Q,	phi_Q,	k_Q, c_Q]
% kalmanQParams = [4.0,	10.0,	5.0,	8.0,	0.5,	0.025, 0.0025];  


kalmanQParams = [4.0,	2.0,	5.0,	0.5,	0.0,	0.025, 0.0025];  
% kalmanQParams = [2.5,	0.25,	1,	0.25,	0.0,	0.025, 0.0025];  


% measurement model variances
% param ordering : [f_R,    mu_R,   a_R,    omega_R]
% % kalmanRParams = [1.5,    2.75,   1.25,   4.5];
kalmanRParams = [0.8072,    0.0021,   0.0092,   0.0762];

% initial stiffness (defined but not used for all experiments)
%k = [k_xx; k_yy; k_zz]
k = [0.025; 0.025; 0.025];

% initial damping (defined but not used for all experiments)
%c = [c_xx; c_yy; c_zz]
c = [0.025; 0.025; 0.025];


end
