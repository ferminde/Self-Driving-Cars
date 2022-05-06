%% Plot Trajectory

% RUN

load('TestTrack.mat');
load('ROB535_ControlProject_part1_Team20.mat')


x0 = [287,5,-176,0,2,0]; % Not Necessary
sol_1 = forwardIntegrateControlInput(ROB535_ControlProject_part1_input, x0);

%function info = getTrajectoryInfo(Y,U,Xobs,t_update,TestTrack)
result = getTrajectoryInfo(sol_1, ROB535_ControlProject_part1_input)
traj = result.Y;
X = traj(:,1);
Y = traj(:,3);

cline = TestTrack.cline;
X_cline = cline(1,:);
Y_cline = cline(2,:);

figure(1);

br = TestTrack.br;
bl = TestTrack.bl;
br_x = br(1,:);
br_y = br(2,:);
bl_x = bl(1,:);
bl_y = bl(2,:);

hold on

plot(X,Y);
plot(br_x, br_y)
plot(bl_x, bl_y)
plot(X_cline, Y_cline);
legend('trajectory optimized', 'forwardcontrol', 'right', 'left');

grid
hold off
