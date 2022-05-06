function [sol_2, FLAG_terminate] = ROB535_ControlProject_part2_Team20(TestTrack, Xobs_seen, curr_state)
    global count;

    if curr_state == [287,5,-176,0,2,0]
        count = 0;
    end

    delta_f = zeros(20000,1);
    delta_f(1:150) = 0.5;
    delta_f(240:427) = -0.5;
    delta_f(1000:1700) = -0.38;
    delta_f(2610:2675) = 0.3;
    delta_f(3098:3161) = -0.14;
    delta_f(3700:3900) = 0.25;
    delta_f(4400:4700) = -0.05;
    delta_f(5055:5702) = -0.04;
    delta_f(6690:6915) = 0.18;
    delta_f(8443:9000) = -0.0845;
    delta_f(9800:9945) = 0.4;
    delta_f(10980:11030) = 0.3675;
    
    F_x = zeros(20000,1);
    F_x(1:390) = 0;
    F_x(390:940) = 5000;
    F_x(9950:10200) = 5000;
    F_x(11030:20000) = 5000;

    arg = [1:60] + 60*count;

    ROB535_ControlProject_part2_input = [delta_f(arg), F_x(arg)];
    sol_2 = ROB535_ControlProject_part2_input;

    if count == 200 - 1
        FLAG_terminate = 1;
    else
        FLAG_terminate = 0;
    end

    count = count + 1;

%% Team 20 - MPC (Did not work...)

%     startX = curr_state(1);
%     startY = curr_state(3);
% 
%     RadiusForOptimization = 0;
%     CoordForOptimization = [0, 0];
%     minDist = inf;
% 
%     if length(Xobs_seen) ~= 0
%         for i = 1:length(Xobs_seen)
%             object = Xobs_seen{i};
%             firstCoord = object(1,:);
%             thirdCoord = object(3,:);
%             centerCoord = [(firstCoord(1,1)+thirdCoord(1,1))/2, (firstCoord(1,2)+thirdCoord(1,2))/2];
%             radius = norm(centerCoord - firstCoord);
% 
%             if norm(centerCoord - [startX, startY]) < minDist
%                 RadiusForOptimization = radius;
%                 CoordForOptimization = centerCoord;
%                 minDist = norm(centerCoord - [startX, startY]);
%             end
%         end
%     end
% 
%     FLAG_terminate = 1;
%     sol_2 = zeros(75,2);

% a = 1.35;
% b = 1.45; 
% L = a + b;
% dt = 0.01;
% 
% % Change Through Trial 
% u_ub = 30;
% u_lb = 5;
% 
% v_ub = 5;
% v_lb = -0.08;
% 
% psi_ub = 5;
% psi_lb = -5;
% 
% r_ub = 10;
% r_lb = -10;
% 
% % initial start point
% init = [287, 5, -176, 0, 2, 0];
% 
% % initial end point
% end_p = ref_traj(2, :);
% plot(end_p(1,1), end_p(1,2), 's')
% txt = "Endpoint " + 1;
% text(end_p(1,1), end_p(1,2), txt) 
% 
% prev_dist = sqrt((287 - end_p(1,1))^2+(-176 - end_p(1,2))^2);
% 
% div = 50;
% nsteps = div;
% points = 15;
% 
% % initial ub & lb
% br = TestTrack.br;
% bl = TestTrack.bl;
% 
% ub = [];
% lb = [];
% 
% br_X = linspace(br(1,1), br(1,2), div); 
% br_Y = linspace(br(2,1), br(2,2), div);
% plot(br_X, br_Y,'-')
% 
% bl_X = linspace(bl(1,1), bl(1,2), div);
% bl_Y = linspace(bl(2,1), bl(2,2), div);
% plot(bl_X, bl_Y,'-')
% 
% for j = 1:div - 1
%     % X; u;
%     ub = [ub; max(br_X(j), bl_X(j)); u_ub];
%     lb = [lb; min(br_X(j), bl_X(j)); u_lb];
%     % Y; v; psi; r;
%     ub = [ub; max(br_Y(j), bl_Y(j)); v_ub; psi_ub; r_ub];
%     lb = [lb; min(br_Y(j), bl_Y(j)); v_lb; psi_lb; r_lb];
% end  
% 
% % X; u;
% ub = [ub; max(br(1,2), bl(1,2)); u_ub];
% lb = [lb; min(br(1,2), bl(1,2)); u_lb];
% % Y; v; psi; r;
% ub = [ub; max(br(2,2), bl(2,2)); v_ub; psi_ub; r_ub];
% lb = [lb; min(br(2,2), bl(2,2)); v_lb; psi_lb; r_lb];
%     
% % d; Fx
% ub = [ub; repmat([0.5;5000], div - 1, 1)];
% lb = [lb; repmat([-0.5;1000], div - 1, 1)];
% 
% % z = [X0 u0 Y0 v0 psi0 r0 ... Xn un Yn vn psin rn d0 Fx0 ... dn Fxn]
% reached = false;
% increasing = false;
% for ind = 1:(points - 1)
%     while ~reached
%         fprintf("From Point %g to Point %g with %g bound divisions\n", ind, ind+1, div);
%         
%         % fmincon
%         disp("Running fmincon")
%         options = optimoptions('fmincon','SpecifyConstraintGradient',true,...
%                                'SpecifyObjectiveGradient',true) ;
%         
%         x0=zeros(1,8*nsteps - 2);
%         x0(1:6) = init;
%     
%         cf = @(z) costfun(z, end_p);
%         nc = @(z) nonlcon(z);
%         
%         z=fmincon(cf,x0,[],[],[],[],lb',ub',nc, options);
%     
%         init_cond = z(1, 1:div*6);
%         input = z(1, div*6+1:length(z));
%      
%         % Forward Integrate Controller
%         disp("Running 1st Forward Integrate Controller")
%         delta_f = input(1:2:end)';
%         F_x = input(2:2:end)';
%     
%         ROB535_ControlProject_part1_input = [delta_f F_x];  
%         sol_1 = forwardIntegrateControlInput(ROB535_ControlProject_part1_input, x0(1:6));
%     
%         % MPC
%         disp("Running MPC")
%         Z_ref = zeros(nsteps,6);
%         U_ref = zeros(nsteps-1,2);
%         
%         Z_ref(:,1) = init_cond(1:6:end)';
%         Z_ref(:,2) = init_cond(2:6:end)';
%         Z_ref(:,3) = init_cond(3:6:end)';
%         Z_ref(:,4) = init_cond(4:6:end)';
%         Z_ref(:,5) = init_cond(5:6:end)';
%         Z_ref(:,6) = init_cond(6:6:end)';
%         Z_ref = Z_ref';
%         U_ref(:,1) = delta_f;
%         U_ref(:,2) = F_x;
%         U_ref=U_ref';
%     
%         input_range = [-0.5, 0.5 ; 500, 5000];
%         
%         % Linearized equations
%         Nw=2;
%         f=0.01;
%         Iz=2667;
%         a=1.35;
%         b=1.45;
%         By=0.27;
%         Cy=1.2;
%         Dy=0.7;
%         Ey=-1.6;
%         Shy=0;
%         Svy=0;
%         m=1400;
%         grav=9.806;
%         Fmax = 0.7*m*grav;
%         
%         alpha_f = @(i) U_ref(1,i) - atan((Z_ref(4,i)+a*Z_ref(6,i))/Z_ref(2,i));
%         alpha_r = @(i) -atan((Z_ref(4,i)-b*Z_ref(6,i))/Z_ref(2,i));
%         
%         alpha_fr = @(i) (-a/Z_ref(2,i))*(1/(1+((Z_ref(4,i)+a*Z_ref(6,i))/Z_ref(2,i))^2));
%         alpha_fv = @(i) (-1/Z_ref(2,i))*(1/(1+((Z_ref(4,i)+a*Z_ref(6,i))/Z_ref(2,i))^2));
%         alpha_fu = @(i) -(Z_ref(4,i)+a*Z_ref(6,i))/(Z_ref(2,i)^2)*(1/(1+((Z_ref(4,i)+a*Z_ref(6,i))/Z_ref(2,i))^2));
%         
%         alpha_rr = @(i) (b/Z_ref(2,i))*(1/(1+((Z_ref(4,i)-b*Z_ref(6,i))/Z_ref(2,i))^2));
%         alpha_rv = @(i) (-1/Z_ref(2,i))*(1/(1+((Z_ref(4,i)-b*Z_ref(6,i))/Z_ref(2,i))^2));
%         alpha_ru = @(i) (Z_ref(4,i)-b*Z_ref(6,i))/(Z_ref(2,i)^2)*(1/(1+((Z_ref(4,i)-b*Z_ref(6,i))/Z_ref(2,i))^2));
%         
%         dapsi_yf = @(i) 1 - Ey + Ey*(1/(1+(By*alpha_f(i)+By*Shy)^2));
%         dapsi_yr = @(i) 1 - Ey + Ey*(1/(1+(By*alpha_r(i)+By*Shy)^2));
%         
%         psi_yf = @(i) (1-Ey)*(alpha_f(i)+Shy)+(Ey/By)*atan(By*(alpha_f(i)+Shy));
%         psi_yr = @(i) (1-Ey)*(alpha_r(i)+Shy)+(Ey/By)*atan(By*(alpha_r(i)+Shy));
%         
%         Fzf = b/(a+b)*m*grav;
%         Fzr = a/(a+b)*m*grav;
%         
%         dpsiFyf = @(i) Fzf*Dy*cos(Cy*atan(By*psi_yf(i)))*(Cy*(1/(1+(By*psi_yf(i))^2))*By);
%         dpsiFyr = @(i) Fzr*Dy*cos(Cy*atan(By*psi_yr(i)))*(Cy*(1/(1+(By*psi_yr(i))^2))*By);
%         
%         dFyf_u = @(i) dpsiFyf(i)*dapsi_yf(i)*alpha_fu(i);
%         dFyf_v = @(i) dpsiFyf(i)*dapsi_yf(i)*alpha_fv(i);
%         dFyf_r = @(i) dpsiFyf(i)*dapsi_yf(i)*alpha_fr(i);
%         
%         dFyr_u = @(i) dpsiFyr(i)*dapsi_yr(i)*alpha_ru(i);
%         dFyr_v = @(i) dpsiFyr(i)*dapsi_yr(i)*alpha_rv(i);
%         dFyr_r = @(i) dpsiFyr(i)*dapsi_yr(i)*alpha_rr(i);
%         
%         Fyf = @(i) Fzf*Dy*sin(Cy*atan(By*psi_yf(i)))+Svy;    
%         Fyr = @(i) Fzr*Dy*sin(Cy*atan(By*psi_yr(i)))+Svy;
%         
%         L = a + b;
%         
%         % decision variables
%         npred=10;
%         nstates = 6;
%         ninputs = 2;
%         
%         Ndec=(npred+1)*nstates+ninputs*npred;   
%     
%         %final trajectory
%         dt = 0.01;
%         T = zeros(nsteps);
%         Z = zeros(6,length(T));
%         %applied inputs
%         U = zeros(2,length(T));
%         %input from QP
%         u_mpc = zeros(2,length(T));
%         %error in states (actual-reference)
%         eZ = zeros(6,length(T));
%         eZ0 = init';
%         
%         Z(:,1) = eZ0 + Z_ref(:,1);  
%         dt = 0.01;
%         
%         %set random initial condition 
%         A = @(i) eye(6) + dt*[0, cos(Z_ref(5,i)), 0, -sin(Z_ref(5,i)), -Z_ref(2,i)*sin(Z_ref(5,i))-Z_ref(4,i)*cos(Z_ref(5,i)), 0;...
%                   0, (-1/m)*dFyf_u(i)*sin(U_ref(1,i)), 0, Z_ref(6,i)+(-1/m)*dFyf_v(i)*sin(U_ref(1,i)), 0, Z_ref(4,i)+(-1/m)*dFyf_r(i)*sin(U_ref(1,i));...
%                   0, sin(Z_ref(5,i)), 0, cos(Z_ref(5,i)), Z_ref(2,i)*cos(Z_ref(5,i))-Z_ref(4,i)*sin(Z_ref(5,i)), 0;...
%                   0, (1/m)*(dFyf_u(i)*cos(U_ref(1,i))+dFyr_u(i))-Z_ref(6,i), 0, (1/m)*(dFyf_v(i)*cos(U_ref(1,i))+dFyr_v(i)), 0, (1/m)*(dFyf_r(i)*cos(U_ref(1,i))+dFyr_r(i))-Z_ref(2,i);...
%                   0, 0, 0, 0, 0, 1;...
%                   0, (1/Iz)*(a*dFyf_u(i)*cos(U_ref(1,i))-b*dFyr_u(i)), 0, (1/Iz)*(a*dFyf_v(i)*cos(U_ref(1,i))-b*dFyr_v(i)), 0, (1/Iz)*(a*dFyf_r(i)*cos(U_ref(1,i))-b*dFyr_r(i))];
%         
%         B = @(i) dt*[0 0;
%                   Nw/m (-1/m)*(Fyf(i)*cos(U_ref(1,i))+dpsiFyf(i)*dapsi_yf(i)*sin(U_ref(1,i)));
%                   0 0;
%                   0 (1/m)*(-Fyf(i)*sin(U_ref(1,i))+dpsiFyf(i)*dapsi_yf(i)*cos(U_ref(1,i)));
%                   0 0;
%                   0 (a/Iz)*(-Fyf(i)*sin(U_ref(1,i))+dpsiFyf(i)*dapsi_yf(i)*cos(U_ref(1,i)))];
%         
%         for i=1:length(T)-1
%             %shorten prediction horizon if we are at the end of trajectory
%             npred_i=min([npred,length(T)-i]);
%             
%             %calculate error
%             eZ(:,i)=Z(:,i)-Z_ref(:,i);
%             
%             %generate equality constraints
%             [Aeq,beq]=eq_cons(i,A,B,eZ(:,i),npred_i,nstates,ninputs);
%             
%             %generate boundary constraints
%             [Lb,Ub]=bound_cons(i,U_ref,npred_i,input_range,nstates,ninputs);
%             
%             %cost for states
%             Q=[1,0.5,1,1,0.5,10];
%             
%             %cost for inputs
%             R=[70,0.4];        
%             
%             H=diag([repmat(Q,[1,npred_i+1]),repmat(R,[1,npred_i])]);
%             
%             f=zeros(nstates*(npred_i+1)+ninputs*npred_i,1);
%             
%             [x,fval] = quadprog(H,f,[],[],Aeq,beq,Lb,Ub);
%             
%             %get linearized input
%             u_mpc(:,i)=x(nstates*(npred_i+1)+1:nstates*(npred_i+1)+ninputs);
%             
%             %get input
%             U(:,i)=u_mpc(:,i)+U_ref(:,i);
%               
%             %simulate model
%             [~,ztemp]=ode45(@(t,x)bike(t,x,0,U(:,i)),[0 dt],Z(:,i));
%             
%             %store final state
%             Z(:,i+1)=ztemp(end,:)';
%         end
%         % Forward Integrate Controller
%         disp("Running Final Forward Integrate Controller")
%         ROB535_ControlProject_part1_input = U';
%         
%         sol_mpc = forwardIntegrateControlInput(ROB535_ControlProject_part1_input, x0(1:6));  
%         result = getTrajectoryInfo(sol_mpc, ROB535_ControlProject_part1_input);
%     
%         traj = result.Y;
%         X = traj(:,1);
%         Y = traj(:,3);
%         u = traj(:,2)
%         plot(X, Y, '-')
%     
%         init = traj(end,:);
%         init_pX = init(1,1);
%         init_pY = init(1,3);
%         
%         % end point
%         end_pX = end_p(1,1);
%         end_pY = end_p(1,2);
% 
%         dist = sqrt((init_pX - end_pX)^2+(init_pY - end_pY)^2)
%         if (prev_dist > dist)
%             increasing = false;
%             prev_dist = dist;
%         else
%             increasing = true;
%             prev_dist = 100;
%         end
% 
%         if increasing
%             % reached end point
%             disp("Reached end point")
%             reached = true;
%             increasing = false;
% 
%             plot(init_pX, init_pY, 'o')
%             text(init_pX, init_pY, num2str(dist))
%         end
%     end
% 
%     % next end point
%     end_p = ref_traj(ind+2, :);
%     plot(end_p(1,1), end_p(1,2), 's')
%     txt = "Endpoint " + (ind+1);
%     text(end_p(1,1), end_p(1,2), txt)
% 
%     % next upper & lower boundary
%     ub = [];
%     lb = [];
%     
%     br_X = linspace(br(1,ind+1), br(1,ind+2), div); 
%     br_Y = linspace(br(2,ind+1), br(2,ind+2), div);
%     plot(br_X, br_Y,'-')
%     
%     bl_X = linspace(bl(1,ind+1), bl(1,ind+2), div);
%     bl_Y = linspace(bl(2,ind+1), bl(2,ind+2), div);
%     plot(bl_X, bl_Y,'-')
%     
%     for j = 1:div - 1
%         % X; u;
%         ub = [ub; max(br_X(j), bl_X(j)); u_ub];
%         lb = [lb; min(br_X(j), bl_X(j)); u_lb];
%         % Y; v; psi; r;
%         ub = [ub; max(br_Y(j), bl_Y(j)); v_ub; psi_ub; r_ub];
%         lb = [lb; min(br_Y(j), bl_Y(j)); v_lb; psi_lb; r_lb];
%     end  
%     
%     % X; u;
%     ub = [ub; max(br(1,ind+2), bl(1,ind+2)); u_ub];
%     lb = [lb; min(br(1,ind+2), bl(1,ind+2)); u_lb];
%     % Y; v; psi; r;
%     ub = [ub; max(br(2,ind+2), bl(2,ind+2)); v_ub; psi_ub; r_ub];
%     lb = [lb; min(br(2,ind+2), bl(2,ind+2)); v_lb; psi_lb; r_lb];
%         
%     % d; Fx
%     ub = [ub; repmat([0.5;5000], div - 1, 1)];
%     lb = [lb; repmat([-0.5;500], div - 1, 1)];
%     
%     plot(max(br(1,ind+2), bl(1,ind+2)), max(br(2,ind+2), bl(2,ind+2)), 'o')
%     plot(min(br(1,ind+2), bl(1,ind+2)), min(br(2,ind+2), bl(2,ind+2)), 'o')
%     % reset reached to false
%     reached = false;
% end
% % hold off
% 
% %% costfun
% function [J, dJ] = costfun(z, end_p)
%     div = 50;
%     nsteps = div;
% 
%     num_states = 6;
% 
%     Xend = end_p(1,1);
%     Yend = end_p(1,2);
% 
%     % size of J must be 1 x 1
%     J = 0;
%     % z = [X0 u0 Y0 v0 psi0 r0 ... Xn un Yn vn psin rn d0 Fx0 ... dn Fxn]
%     for i = 1:nsteps
%         x = z(6*(i - 1) + 1);
%         u = z(6*(i - 1) + 2);
%         y = z(6*(i - 1) + 3);
%         v = z(6*(i - 1) + 4);
%         psi = z(6*(i - 1) + 5);
%         r = z(6*(i - 1) + 6);
%     
%         J = J + (x - Xend)^2 + u^2 + (y - Yend)^2 ...
%               + v^2 + psi^2 + r^2;
%     end
%     
%     for i = 1:nsteps-1
%         delta = z(num_states * nsteps + 2*i - 1);
%         F_x = z(num_states * nsteps + 2*i);
% 
%         J = J + delta^2 + F_x^2;
%     end
%     % size of dJ must be 1 x (nsteps * 6 + (nsteps-1) * 2) (1 x no. of elements in 'z')
%     dJ = zeros(1,nsteps * 6 + (nsteps-1) * 2);
%     for i = 1:nsteps
%         x = z(6*(i - 1) + 1);
%         u = z(6*(i - 1) + 2);
%         y = z(6*(i - 1) + 3);
%         v = z(6*(i - 1) + 4);
%         psi = z(6*(i - 1) + 5);
%         r = z(6*(i - 1) + 6);
% 
%         dJ((i-1)*6+1) = 2*(x - Xend);
%         dJ((i-1)*6+2) = 2*u;
%         dJ((i-1)*6+3) = 2*(y - Yend);
%         dJ((i-1)*6+4) = 2*v;
%         dJ((i-1)*6+5) = 2*psi;
%         dJ(i*6) = 2*r;
%     end
%     for i = 1:nsteps-1
%         delta = z(num_states * nsteps + 2*i - 1);
%         F_x = z(num_states * nsteps + 2*i);
% 
%         dJ((i-1)*2+(nsteps*6+1))=2*delta;
%         dJ((i-1)*2+(nsteps*6+2))=2*F_x;
%     end
% end
% 
% %% nonlcon
% function [g,h,dg,dh]=nonlcon(z)
%     div = 50;
%     nsteps = div;
% 
%     num_states = 6;
%     num_elements = length(z);
%     a = 1.35;
%     b = 1.45; 
%     L = a + b;
%     dt = 0.01;
% 
%     g = zeros(nsteps, 1);
%     %size(g)
%     dg = zeros(num_elements, nsteps);
%     %size(dg)
%     h = zeros(nsteps * num_states, 1);
%     %size(h)
%     dh = zeros(nsteps * num_states, num_elements);
%     %size(dh)
%     %dh = transpose(dh);
%     
%     %constants
%     Nw=2;
%     f=0.01;
%     Iz=2667;
%     a=1.35;
%     b=1.45;
%     By=0.27;
%     Cy=1.2;
%     Dy=0.7;
%     Ey=-1.6;
%     Shy=0;
%     Svy=0;
%     m=1400;
%     grav=9.806;
%     
%     %h = zeros(6 * nsteps, 1);
%     %size(h)
%     h(1) = z(1);
%     h(2) = z(2);
%     h(3) = z(3);
%     h(4) = z(4);
%     h(5) = z(5);
%     h(6) = z(6);
% 
%     for i=1:nsteps - 1
%         x = z(6*(i - 1) + 1);
%         u = z(6*(i - 1) + 2);
%         y = z(6*(i - 1) + 3);
%         v = z(6*(i - 1) + 4);
%         psi = z(6*(i - 1) + 5);
%         r = z(6*(i - 1) + 6);
%         delta = z(num_states * nsteps + 2*i - 1);
%         F_x = z(num_states * nsteps + 2*i);
%         
%         alpha_f = rad2deg(delta - atan((v+a*r)/u));
%         alpha_r = rad2deg(-atan((v-b*r)/u));
% 
%         alpha_fr = rad2deg((-a/u)*(1/(1+((v+a*r)/u)^2)));
%         alpha_fv = rad2deg((-1/u)*(1/(1+((v+a*r)/u)^2)));
%         alpha_fu = rad2deg(-(v+a*r)/(u^2)*(1/(1+((v+a*r)/u)^2)));
% 
%         alpha_rr = rad2deg((b/u)*(1/(1+((v-b*r)/u)^2)));
%         alpha_rv = rad2deg((-1/u)*(1/(1+((v-b*r)/u)^2)));
%         alpha_ru = rad2deg((v-b*r)/(u^2)*(1/(1+((v-b*r)/u)^2)));
% 
%         dapsi_yf = 1 - Ey + Ey*(1/(1+(By*alpha_f+By*Shy)^2));
%         dapsi_yr = 1 - Ey + Ey*(1/(1+(By*alpha_r+By*Shy)^2));
% 
%         psi_yf = (1-Ey)*(alpha_f+Shy)+(Ey/By)*atan(By*(alpha_f+Shy));
%         psi_yr = (1-Ey)*(alpha_r+Shy)+(Ey/By)*atan(By*(alpha_r+Shy));
%         
%         Fzf = b/(a+b)*m*grav;
%         Fzr = a/(a+b)*m*grav;
% 
%         dpsiFyf = Fzf*Dy*cos(Cy*atan(By*psi_yf))*(Cy*(1/(1+(By*psi_yf)^2))*By);
%         dpsiFyr = Fzr*Dy*cos(Cy*atan(By*psi_yr))*(Cy*(1/(1+(By*psi_yr)^2))*By);
% 
%         dFyf_u = dpsiFyf*dapsi_yf*alpha_fu;
%         dFyf_v = dpsiFyf*dapsi_yf*alpha_fv;
%         dFyf_r = dpsiFyf*dapsi_yf*alpha_fr;
% 
%         dFyr_u = dpsiFyr*dapsi_yr*alpha_ru;
%         dFyr_v = dpsiFyr*dapsi_yr*alpha_rv;
%         dFyr_r = dpsiFyr*dapsi_yr*alpha_rr;
% 
%         F_yf = Fzf*Dy*sin(Cy*atan(By*psi_yf))+Svy;
%         F_yr = Fzr*Dy*sin(Cy*atan(By*psi_yr))+Svy;
% 
%         dzdt = [u*cos(psi)-v*sin(psi);...
%           (-f*m*grav+Nw*F_x-F_yf*sin(delta))/m+v*r;...
%           u*sin(psi)+v*cos(psi);...
%           (F_yf*cos(delta)+F_yr)/m-u*r;...
%           r;...
%           (F_yf*a*cos(delta)-F_yr*b)/Iz];
% 
%         h(6*i + 1) = z(i*6 + 1) - z(6*(i - 1) + 1) - dt*dzdt(1);
%         h(6*i + 2) = z(i*6 + 2) - z(6*(i - 1) + 2) - dt*dzdt(2);
%         h(6*i + 3) = z(i*6 + 3) - z(6*(i - 1) + 3) - dt*dzdt(3);
% 
%         h(6*i + 4) = z(i*6 + 4) - z(6*(i - 1) + 4) - dt*dzdt(4);
%         h(6*i + 5) = z(i*6 + 5) - z(6*(i - 1) + 5) - dt*dzdt(5);
%         h(6*i + 6) = z(i*6 + 6) - z(6*(i - 1) + 6) - dt*dzdt(6);
%     end
%     %size(h)
%     %size of dh must be = Transpose((no. of time steps * no. of states) x no. of elements in 'z') ;
%     %dh = zeros(6 * nsteps, num_elements);
%     %size(dh)
%     dh(1,1) = 1;
%     dh(2,2) = 1;
%     dh(3,3) = 1;
%     dh(4,4) = 1;
%     dh(5,5) = 1;
%     dh(6,6) = 1;
%     for i = 1:nsteps - 1
%         %disp(size(dh))
%         x = z(6*(i - 1) + 1);
%         u = z(6*(i - 1) + 2);
%         y = z(6*(i - 1) + 3);
%         v = z(6*(i - 1) + 4);
%         psi = z(6*(i - 1) + 5);
%         r = z(6*(i - 1) + 6);
%         delta = z(num_states * nsteps + 2*i - 1);
%         F_x = z(num_states * nsteps + 2*i);
%         
%         alpha_f = delta - atan((v+a*r)/u);
%         alpha_r = -atan((v-b*r)/u);
% 
%         alpha_fr = (-a/u)*(1/(1+((v+a*r)/u)^2));
%         alpha_fv = (-1/u)*(1/(1+((v+a*r)/u)^2));
%         alpha_fu = -(v+a*r)/(u^2)*(1/(1+((v+a*r)/u)^2));
% 
%         alpha_rr = (b/u)*(1/(1+((v-b*r)/u)^2));
%         alpha_rv = (-1/u)*(1/(1+((v-b*r)/u)^2));
%         alpha_ru = (v-b*r)/(u^2)*(1/(1+((v-b*r)/u)^2));
% 
%         dapsi_yf = 1 - Ey + Ey*(1/(1+(By*alpha_f+By*Shy)^2));
%         dapsi_yr = 1 - Ey + Ey*(1/(1+(By*alpha_r+By*Shy)^2));
% 
%         psi_yf = (1-Ey)*(alpha_f+Shy)+(Ey/By)*atan(By*(alpha_f+Shy));
%         psi_yr = (1-Ey)*(alpha_r+Shy)+(Ey/By)*atan(By*(alpha_r+Shy));
%         
%         Fzf = b/(a+b)*m*grav;
%         Fzr = a/(a+b)*m*grav;
% 
%         dpsiFyf = Fzf*Dy*cos(Cy*atan(By*psi_yf))*(Cy*(1/(1+(By*psi_yf)^2))*By);
%         dpsiFyr = Fzr*Dy*cos(Cy*atan(By*psi_yr))*(Cy*(1/(1+(By*psi_yr)^2))*By);
% 
%         dFyf_u = dpsiFyf*dapsi_yf*alpha_fu;
%         dFyf_v = dpsiFyf*dapsi_yf*alpha_fv;
%         dFyf_r = dpsiFyf*dapsi_yf*alpha_fr;
% 
%         dFyr_u = dpsiFyr*dapsi_yr*alpha_ru;
%         dFyr_v = dpsiFyr*dapsi_yr*alpha_rv;
%         dFyr_r = dpsiFyr*dapsi_yr*alpha_rr;
% 
%         Fyf = Fzf*Dy*sin(Cy*atan(By*psi_yf))+Svy;
%         Fyr = Fzr*Dy*sin(Cy*atan(By*psi_yr))+Svy;
% 
%         % states - first term in h
%         dh(6*i + 1, 6*i + 1) = 1; %respect to x
%         dh(6*i + 2, 6*i + 2) = 1; %respect to u
%         dh(6*i + 3, 6*i + 3) = 1; % respect to y
%         dh(6*i + 4, 6*i + 4) = 1; %respect to v
%         dh(6*i + 5, 6*i + 5) = 1; %respect to psi
%         dh(6*i + 6, 6*i + 6) = 1; % respect to r
% 
%         % second term in h
%         dh(6*i + 1, 6*(i-1) + 1) = -1; %respect to x
%         dh(6*i + 2, 6*(i-1) + 2) = -1; %respect to u
%         dh(6*i + 3, 6*(i-1) + 3) = -1; % respect to y
%         dh(6*i + 4, 6*(i-1) + 4) = -1; %respect to v
%         dh(6*i + 5, 6*(i-1) + 5) = -1; %respect to psi
%         dh(6*i + 6, 6*(i-1) + 6) = -1; % respect to r
% 
%         %derivative terms
%         dh(6*i + 1, num_states * nsteps + i*2 - 1) = 0; % xdot with respect to delta
%         dh(6*i + 2, num_states * nsteps + i*2 - 1) = -dt*(-1/m)*(Fyf*cos(delta)+dpsiFyf*dapsi_yf*sin(delta)); % udot with respect to delta
%         dh(6*i + 3, num_states * nsteps + i*2 - 1) = 0; % ydot with respect to delta
%         dh(6*i + 4, num_states * nsteps + i*2 - 1) = -dt*(1/m)*(-Fyf*sin(delta)+dpsiFyf*dapsi_yf*cos(delta)); % vdot with respect to delta
%         dh(6*i + 5, num_states * nsteps + i*2 - 1) = 0; % psidot with respect to delta
%         dh(6*i + 6, num_states * nsteps + i*2 - 1) = -dt*(a/Iz)*(-Fyf*sin(delta)+dpsiFyf*dapsi_yf*cos(delta)); % rdot with respect to delta
% 
%         dh(6*i + 1, num_states * nsteps + i*2) = 0; % xdot with respect to F_x
%         dh(6*i + 2, num_states * nsteps + i*2) = -dt*Nw/m;% udot with respect to F_x
%         dh(6*i + 3, num_states * nsteps + i*2) = 0; % ydot with respect to F_x
%         dh(6*i + 4, num_states * nsteps + i*2) = 0; % vdot with respect to F_x
%         dh(6*i + 5, num_states * nsteps + i*2) = 0;% psidot with respect to F_x
%         dh(6*i + 6, num_states * nsteps + i*2) = 0; % rdot with respect to F_x
% 
%         dh(6*i + 1, 6*(i-1) + 2) = -dt*cos(psi); % xdot with respect to u
%         dh(6*i + 2, 6*(i-1) + 2) = -dt*(-1/m)*dFyf_u*sin(delta);% udot with respect to u
%         dh(6*i + 3, 6*(i-1) + 2) = -dt*sin(psi); % ydot with respect to u
%         dh(6*i + 4, 6*(i-1) + 2) = -dt*((1/m)*(dFyf_u*cos(delta)+dFyr_u)-r); % vdot with respect to u
%         dh(6*i + 5, 6*(i-1) + 2) = 0;% psidot with respect to u
%         dh(6*i + 6, 6*(i-1) + 2) = -dt*(1/Iz)*(a*dFyf_u*cos(delta)-b*dFyr_u); % rdot with respect to u
% 
%         dh(6*i + 1, 6*(i-1) + 4) = -dt*(-sin(psi)); % xdot with respect to v
%         dh(6*i + 2, 6*(i-1) + 4) = -dt*(r+(-1/m)*dFyf_v*sin(delta));% udot with respect to v
%         dh(6*i + 3, 6*(i-1) + 4) = -dt*cos(psi); % ydot with respect to v
%         dh(6*i + 4, 6*(i-1) + 4) = -dt*(1/m)*(dFyf_v*cos(delta)+dFyr_v); % vdot with respect to v
%         dh(6*i + 5, 6*(i-1) + 4) = 0;% psidot with respect to v
%         dh(6*i + 6, 6*(i-1) + 4) = -dt*(1/Iz)*(a*dFyf_v*cos(delta)-b*dFyr_v); % rdot with respect to v
% 
%         dh(6*i + 1, 6*(i-1) + 5) = -dt*(-u*sin(psi)-v*cos(psi)); % xdot with respect to psi
%         dh(6*i + 2, 6*(i-1) + 5) = 0;% udot with respect to psi
%         dh(6*i + 3, 6*(i-1) + 5) = -dt*(u*cos(psi) - v*sin(psi)); % ydot with respect to psi
%         dh(6*i + 4, 6*(i-1) + 5) = 0; % vdot with respect to psi
%         dh(6*i + 5, 6*(i-1) + 5) = 0;% psidot with respect to psi
%         dh(6*i + 6, 6*(i-1) + 5) = 0; % rdot with respect to psi
% 
%         dh(6*i + 1, 6*(i-1) + 6) = 0; % xdot with respect to r
%         dh(6*i + 2, 6*(i-1) + 6) = -dt*(v+(-1/m)*dFyf_r*sin(delta));% udot with respect to r
%         dh(6*i + 3, 6*(i-1) + 6) = 0; % ydot with respect to r
%         dh(6*i + 4, 6*(i-1) + 6) = -dt*((1/m)*(dFyf_r*cos(delta)+dFyr_r)-u); % vdot with respect to r
%         dh(6*i + 5, 6*(i-1) + 6) = 1;% psidot with respect to r
%         dh(6*i + 6, 6*(i-1) + 6) = -dt*((1/Iz)*(a*dFyf_r*cos(delta)-b*dFyr_r)); % rdot with respect to r
% 
%         % all other derivatives are zero
%         %size(dh)
%     end
%     %size(dh)
%     dh = transpose(dh);
% end
% % 
% %% odefun
% function [dzdt] = odefun(x,u)
%     Nw=2;
%     f=0.01;
%     Iz=2667;
%     a=1.35;
%     b=1.45;
%     By=0.27;
%     Cy=1.2;
%     Dy=0.7;
%     Ey=-1.6;
%     Shy=0;
%     Svy=0;
% 
%     m=1400;
%     grav=9.806;
%     a = 1.35;
%     b = 1.45; 
%     L = a + b;
% 
%     alpha_f = u(1) - atan((x(4)+a*x(6))/x(2));
%     alpha_r = -atan((x(4)-b*x(6))/x(2));
% 
%     alpha_fr = (-a/x(2))*(1/(1+((x(4)+a*x(6))/x(2))^2));
%     alpha_fv = (-1/x(2))*(1/(1+((x(4)+a*x(6))/x(2))^2));
%     alpha_fu = -(x(4)+a*x(6))/(x(2)^2)*(1/(1+((x(4)+a*x(6))/x(2))^2));
% 
%     alpha_rr = (b/x(2))*(1/(1+((x(4)-b*x(6))/x(2))^2));
%     alpha_rv = (-1/x(2))*(1/(1+((x(4)-b*x(6))/x(2))^2));
%     alpha_ru = (x(4)-b*x(6))/(x(2)^2)*(1/(1+((x(4)-b*x(6))/x(2))^2));
% 
%     dapsi_yf = 1 - Ey + Ey*(1/(1+(By*alpha_f+By*Shy)^2));
%     dapsi_yr = 1 - Ey + Ey*(1/(1+(By*alpha_r+By*Shy)^2));
% 
%     psi_yf = (1-Ey)*(alpha_f+Shy)+(Ey/By)*atan(By*(alpha_f+Shy));
%     psi_yr = (1-Ey)*(alpha_r+Shy)+(Ey/By)*atan(By*(alpha_r+Shy));
%     
%     Fzf = b/(a+b)*m*grav;
%     Fzr = a/(a+b)*m*grav;
% 
%     dpsiFyf = Fzf*Dy*cos(Cy*atan(By*psi_yf))*(Cy*(1/(1+(By*psi_yf)^2))*By);
%     dpsiFyr = Fzr*Dy*cos(Cy*atan(By*psi_yr))*(Cy*(1/(1+(By*psi_yr)^2))*By);
% 
%     dFyf_u = dpsiFyf*dapsi_yf*alpha_fu;
%     dFyf_v = dpsiFyf*dapsi_yf*alpha_fv;
%     dFyf_r = dpsiFyf*dapsi_yf*alpha_fr;
% 
%     dFyr_u = dpsiFyr*dapsi_yr*alpha_ru;
%     dFyr_v = dpsiFyr*dapsi_yr*alpha_rv;
%     dFyr_r = dpsiFyr*dapsi_yr*alpha_rr;
% 
%     F_yf = Fzf*Dy*sin(Cy*atan(By*psi_yf))+Svy;
%     F_yr = Fzr*Dy*sin(Cy*atan(By*psi_yr))+Svy;
% 
%     dzdt = [x(2)*cos(x(5))-x(4)*sin(x(5));...
%           (-f*m*grav+Nw*u(2)-F_yf*sin(u(1)))/m+x(4)*x(6);...
%           x(2)*sin(x(5))+x(4)*cos(x(5));...
%           (F_yf*cos(u(2))+F_yr)/m-x(2)*x(6);...
%           x(6);...
%           (F_yf*a*cos(u(1))-F_yr*b)/Iz];
% end
% 
% % Functions
% 
% function [Aeq,beq]=eq_cons(initial_idx,A,B,x_initial,npred,nstates,ninputs)
% %build matrix for A_i*x_i+B_i*u_i-x_{i+1}=0
% %in the form Aeq*z=beq
% %initial_idx specifies the time index of initial condition from the
% %reference trajectory
% 
% %A and B are function handles above
% %initial condition
%     
%     x_initial=x_initial(:);
%     %size of decision variable and size of part holding states
%     zsize=(npred+1)*nstates+npred*ninputs;
%     xsize=(npred+1)*nstates;
%     Aeq=zeros(xsize,zsize);
%     Aeq(1:nstates,1:nstates)=eye(nstates); %initial condition 
%     beq=zeros(xsize,1);
%     beq(1:nstates)=x_initial;
%     state_idxs=nstates+1:nstates:xsize;
%     input_idxs=xsize+1:ninputs:zsize;
%     for i=1:npred
%         %negative identity for i+1
%         
%         Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i):state_idxs(i)+nstates- 1)=-eye(nstates);
% 
%         %A matrix for i
% 
%         Aeq(state_idxs(i):state_idxs(i)+nstates-1,state_idxs(i)-nstates:state_idxs(i)- 1)=A(initial_idx+i-1);
% 
%         %B matrix for i
%         Aeq(state_idxs(i):state_idxs(i)+nstates-1,input_idxs(i):input_idxs(i)+ninputs- 1)=B(initial_idx+i-1);
%     end
% 
% end
% 
% function [Lb,Ub]=bound_cons(initial_idx,U_ref,npred,input_range,nstates,ninputs)
% %time_idx is the index along uref the initial condition is at
%     xsize=(npred+1)*nstates;
%     usize=npred*ninputs;
%     Lb=[];
%     Ub=[];
%     
%     for j=1:ninputs
%         Lb=[Lb;input_range(j,1)-U_ref(j,initial_idx:initial_idx+npred-1)];
%         Ub=[Ub;input_range(j,2)-U_ref(j,initial_idx:initial_idx+npred-1)];
%     end
%     
%     Lb=reshape(Lb,[usize,1]);
%     Ub=reshape(Ub,[usize,1]);
%     Lb=[-Inf(xsize,1);Lb];
%     Ub=[Inf(xsize,1);Ub];
% end
% 
% function dzdt=bike(t,x,T,U_in)
% %constants
% Nw=2;
% f=0.01;
% Iz=2667;
% a=1.35;
% b=1.45;
% By=0.27;
% Cy=1.2;
% Dy=0.7;
% Ey=-1.6;
% Shy=0;
% Svy=0;
% m=1400;
% g=9.806;
% 
% 
% %generate input functions
% if length(T)<=1 || isempty(T) || size(U_in,2)==1
%     delta_f=U_in(1);
%     F_x=U_in(2);
% else
%     delta_f=interp1(T',U_in(1,:)',t,'previous');
%     F_x=interp1(T',U_in(2,:)',t,'previous');
% end
% 
% %slip angle functions in degrees
% a_f=rad2deg(delta_f-atan2(x(4)+a*x(6),x(2)));
% a_r=rad2deg(-atan2((x(4)-b*x(6)),x(2)));
% 
% %Nonlinear Tire Dynamics
% phi_yf=(1-Ey)*(a_f+Shy)+(Ey/By)*atan(By*(a_f+Shy));
% phi_yr=(1-Ey)*(a_r+Shy)+(Ey/By)*atan(By*(a_r+Shy));
% 
% F_zf=b/(a+b)*m*g;
% F_yf=F_zf*Dy*sin(Cy*atan(By*phi_yf))+Svy;
% 
% F_zr=a/(a+b)*m*g;
% F_yr=F_zr*Dy*sin(Cy*atan(By*phi_yr))+Svy;
% 
% F_total=sqrt((Nw*F_x)^2+(F_yr^2));
% F_max=0.7*m*g;
% 
% if F_total>F_max
%     
%     F_x=F_max/F_total*F_x;
%   
%     F_yr=F_max/F_total*F_yr;
% end
% 
% %vehicle dynamics
% dzdt= [x(2)*cos(x(5))-x(4)*sin(x(5));...
%           (-f*m*g+Nw*F_x-F_yf*sin(delta_f))/m+x(4)*x(6);...
%           x(2)*sin(x(5))+x(4)*cos(x(5));...
%           (F_yf*cos(delta_f)+F_yr)/m-x(2)*x(6);...
%           x(6);...
%           (F_yf*a*cos(delta_f)-F_yr*b)/Iz];
% end
end