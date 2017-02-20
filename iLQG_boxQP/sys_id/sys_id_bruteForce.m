c_x = 25:35;
c_a = 15:25;
Iz = 0.021;
mu = 0.57;
mu_spin = 0.20;

ramp = 0.16;

all_comb = combvec(c_x,c_a,Iz,mu,mu_spin,ramp);

% load('sys_id/experiments/steer_ramp_1L.mat');
% x_1L = zeros(6,size(stateData,1));
% for i = 1:size(stateData,1)
%     x_1L(:,i) = stateData(i).X';
% end
% % normalize to origin
% x_1L = bsxfun(@minus,x_1L,[x_1L(1,1);x_1L(2,1);x_1L(3,1);0;0;0]);

load('sys_id/experiments/steer_ramp_2L.mat');
x_2L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_2L(:,i) = stateData(i).X';
end
% normalize to origin
x_2L = bsxfun(@minus,x_2L,[x_2L(1,1);x_2L(2,1);x_2L(3,1);0;0;0]);

load('sys_id/experiments/steer_ramp_3L.mat');
x_3L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_3L(:,i) = stateData(i).X';
end
% normalize to origin
x_3L = bsxfun(@minus,x_3L,[x_3L(1,1);x_3L(2,1);x_3L(3,1);0;0;0]);

load('sys_id/experiments/steer_ramp_4L.mat');
x_4L = zeros(6,size(stateData,1));
for i = 1:size(stateData,1)
    x_4L(:,i) = stateData(i).X';
end
% normalize to origin
x_4L = bsxfun(@minus,x_4L,[x_4L(1,1);x_4L(2,1);x_4L(3,1);0;0;0]);

min_cm = inf;
h = waitbar(0);
for i = 1:length(all_comb)
    param = all_comb(1:5,i);
    ramp = all_comb(6,i);
%     [sim_x_1L, sim_u] = inc_steer_sim(1,1,ramp,param);
    [sim_x_2L, sim_u] = inc_steer_sim(2,1,0.20,param);
    [sim_x_3L, sim_u] = inc_steer_sim(3,1,ramp,param);
    [sim_x_4L, sim_u] = inc_steer_sim(4,1,ramp,param);
    
%     P = sim_x_1L(1:2,:).';
%     Q = x_1L(1:2,:).';
%     cm = DiscreteFrechetDist(P,Q);
    
    P = sim_x_2L(1:2,:).';
    Q = x_2L(1:2,:).';
    cm = DiscreteFrechetDist(P,Q);
    
    P = sim_x_3L(1:2,:).';
    Q = x_3L(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    P = sim_x_4L(1:2,:).';
    Q = x_4L(1:2,:).';
    cm = cm + DiscreteFrechetDist(P,Q);
    
    if cm < min_cm
        min_comb = all_comb(:,i);
        min_cm = cm;
    end
    percent = double(i)/length(all_comb);
    waitbar(percent,h,sprintf('%f ', min_comb));
end