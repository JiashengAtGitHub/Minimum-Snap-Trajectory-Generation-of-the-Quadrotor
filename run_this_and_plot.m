clear all;close all
l=0.225 ; % This is quadrotor's wing length. Needs to be specified here for plotting.
Trajectory_Generator_Minimum_Snap ; sim ('Trajectory_Tracking_Control_Structure3_Modifying')
%% plot
fontsize=15;
markersize=10;
% data
% source: x_all=[x_phase2(1:end-1,:);x_phase3(1:end-1,:);x_phase4(1:end-1,:);x_phase5];  % pay attention
n=numel(x_all(:,1)); % number of samples  
time_all=(0:0.01:(n-1)*0.01)';                       % pay attention: source:(n-1)*0.01
dataNum=max(size(time_all));
p_all=x_all(:,1:3); % position at all time
v_all=x_all(:,4:6); % velocity at all time
eta_all=x_all(:,7:9); % euler angle at all time
omega_all=x_all(:,10:12); % anguler velocity w.r.t body frame at all time

% positions of rotors, pen tips, etc.
for i=1:n                                 % pay attention
  phi=eta_all(i,1);theta=eta_all(i,2);psi=eta_all(i,3);
  Rbn=[cos(theta)*cos(psi) sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi) cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
       cos(theta)*sin(psi) sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi) cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
      -sin(theta)         sin(phi)*cos(theta)                            cos(phi)*cos(theta)                           ];
  p_rotor1(i,:)=Rbn*[l 0 0]'+p_all(i,:)';
  p_rotor2(i,:)=Rbn*[0 l 0]'+p_all(i,:)';
  p_rotor3(i,:)=Rbn*[-l 0 0]'+p_all(i,:)';
  p_rotor4(i,:)=Rbn*[0 -l 0]'+p_all(i,:)';
  p_z(i,:)=Rbn*[0 0 l]'+p_all(i,:)';    
  p_nib(i,:)=Rbn*[0 1.8 -0.1]'*1.25*l+p_all(i,:)';
 % source: p_nib(i,:)=Rbn*[0 0 -1.25*l]'+pn_all(i,:)';  
end
    
%% plot trajectory of center of mass
figure;
hold on; box on; axis equal
set(get(gca,'title'), 'string', 'Trajectory', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'x (m)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', fontsize);
set(get(gca, 'zlabel'), 'String', 'z (m)', 'fontSize', fontsize);
plot3(p_d(:,1),p_d(:,2),p_d(:,3))        % trajectory of reference positions
plot3(p_all(:,1),p_all(:,2),p_all(:,3),'k--'); % trajectory of actual postions
view([20,20])
plot3(p_all(1,1),p_all(1,2),p_all(1,3), 'marker', 'o', 'markersize', markersize); % initial position
plot3(p_all(end,1),p_all(end,2),p_all(end,3), 'marker', 'o', 'markersize', markersize, 'MarkerFaceColor', 'b'); % final position
set(gca,'xlim',get(gca,'xlim')+[-1,1])
set(gca,'ylim',get(gca,'ylim')+[-1,1])
set(gca,'zlim',get(gca,'zlim')+[-1,1])

 %% plot rotors' positions at a given frequency 
 % (This part was created for crossing-the-window project since I wanted to see the exact postions of 4 rotors)
% for i=[1 dataNum]               % modify it to determine the frequency you want rotors to be plot 
% plot3([p_rotor1(i,1) p_rotor3(i,1)],[p_rotor1(i,2) p_rotor3(i,2)],[p_rotor1(i,3) p_rotor3(i,3)],'r')
% plot3(p_rotor1(i,1),p_rotor1(i,2),p_rotor1(i,3),'r*')
% plot3(p_rotor3(i,1),p_rotor3(i,2),p_rotor3(i,3),'rx')
% plot3([p_rotor2(i,1) p_rotor4(i,1)],[p_rotor2(i,2) p_rotor4(i,2)],[p_rotor2(i,3) p_rotor4(i,3)],'g')
% plot3(p_rotor2(i,1),p_rotor2(i,2),p_rotor2(i,3),'g*')
% plot3(p_rotor4(i,1),p_rotor4(i,2),p_rotor4(i,3),'gx')
% plot3([p_all(i,1) p_z(i,1)],[p_all(i,2) p_z(i,2)],[p_all(i,3) p_z(i,3)],'b')
% end
 %% horizontal window 
% plot3([-0.2 0.2],[-0.6 -0.6],[0 0],'k-','LineWidth',3)
% plot3([-0.2 0.2],[0.6 0.6],[0 0],'k-','LineWidth',3)
% plot3([-0.2 -0.2],[-0.6 0.6],[0 0],'k-','LineWidth',3)
% plot3([0.2 0.2],[-0.6 0.6],[0 0],'k-','LineWidth',3)
 % vertical window
% plot3([-0.2 0.2],[0 0],[-0.6 -0.6],'k-','LineWidth',3)
% plot3([-0.2 0.2],[0 0],[0.6 0.6],'k-','LineWidth',3)
% plot3([-0.2 -0.2],[0 0],[-0.6 0.6],'k-','LineWidth',3)
% plot3([0.2 0.2],[0 0],[-0.6 0.6],'k-','LineWidth',3)
 % inclined window (60 degree to horizontal plane)
% R=[cos(-pi/6) 0 -sin(-pi/6); 0  0  0; sin(-pi/6) 0 cos(-pi/6)];
% my_matrix=[R*[0.2 0 0.6]' R*[0.2 0 -0.6]' R*[-0.2 0 -0.6]' R*[-0.2 0 0.6]' R*[0.2 0 0.6]']
% plot3(my_matrix(1,:),my_matrix(2,:),my_matrix(3,:),'k-','LineWidth',3)
 % vertical wall
% plot3([0 0],[-1 1],[-1 -1],'k-','LineWidth',2)
% plot3([0 0],[-1 1],[1 1],'k-','LineWidth',2)
% plot3([0 0],[-1 -1],[-1 1],'k-','LineWidth',2)
% plot3([0 0],[1 1],[-1 1],'k-','LineWidth',2)

%% trajectory and attitude animation
xlim=get(gca,'xlim'); xwidth=max(xlim)-min(xlim);
ylim=get(gca,'ylim'); ywidth=max(ylim)-min(ylim);
zlim=get(gca,'zlim'); zwidth=max(zlim)-min(zlim);
scale=1.25*l;% 1.25 was got from my own test. I have no idea about the correlation.Shiyu: scale=1/8*min([xwidth,ywidth,zwidth])
%--------------------------------------------------------------------------
% plot drones at all waypoints
for i=times 
fcn_QuadcopterAnimation(-1, x_all(1+100*i,1:3), fcn_Euler2Rotation(x_all(1+100*i,7), x_all(1+100*i,8), x_all(1+100*i,9)), scale);
end
%--------------------------------------------------------------------------
hObject=fcn_QuadcopterAnimation(-1, x_all(1,1:3), fcn_Euler2Rotation(x_all(1,7), x_all(1,8), x_all(1,9)), scale); % create object
for i=1:5:dataNum
    pni=p_all(i,:)';
    etai=eta_all(i,:)';
    Ri=fcn_Euler2Rotation(etai(1), etai(2), etai(3));
    fcn_QuadcopterAnimation(hObject, pni, Ri, scale);
%     if p_nib(i,2)> 0.5061 % 9 % board position on y axes
%     plot3(p_nib(i,1),p_nib(i,2),p_nib(i,3),'k.','MarkerSize',10)
%     end
    pause(0.1); % pause is inaccurate. pause (0.01) does not work. pause(0.1) is acceptable. In the future I may use a timer
% disp(strcat(num2str(i/dataNum*100, '%3.1f'),'% animation')); % display % of animation
end
%--------------------------------------------------------------------------
% Position x-y-z
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Position', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'x (m)', 'fontSize', fontsize);
plot(time_all,p_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'y (m)', 'fontSize', fontsize);
plot(time_all,p_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'z (m)', 'fontSize', fontsize);
plot(time_all,p_all(:,3));

% velocity
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Linear velocity', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'v_x (m/s)', 'fontSize', fontsize);
plot(time_all,v_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', 'v_y (m/s)', 'fontSize', fontsize);
plot(time_all,v_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', 'v_z (m/s)', 'fontSize', fontsize);
plot(time_all,v_all(:,3));

% Euler angles
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Euler angles', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\phi (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,1)/pi*180);
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\theta (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,2)/pi*180);
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', '\psi (deg)', 'fontSize', fontsize);
plot(time_all,eta_all(:,3)/pi*180);

% omega
figure;
subplot(3,1,1);
hold on; box on
set(get(gca,'title'), 'string', 'Angular velocity', 'fontsize', fontsize)
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\omega_x (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,1));
subplot(3,1,2);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'ylabel'), 'String', '\omega_y (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,2));
subplot(3,1,3);
hold on; box on
set(gca, 'fontSize', fontsize)
set(get(gca, 'xlabel'), 'String', 'time (s)', 'fontSize', fontsize);
set(get(gca, 'ylabel'), 'String', '\omega_z (rad/s)', 'fontSize', fontsize);
plot(time_all,omega_all(:,3));


%% trajectories of 4 rotors
% plot3(p_rotor1(:,1),p_rotor1(:,2),p_rotor1(:,3),'r');
% plot3(p_rotor2(:,1),p_rotor2(:,2),p_rotor2(:,3),'g');
% plot3(p_rotor3(:,1),p_rotor3(:,2),p_rotor3(:,3));
% plot3(p_rotor4(:,1),p_rotor4(:,2),p_rotor4(:,3));




