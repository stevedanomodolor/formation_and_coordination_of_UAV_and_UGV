function visualize_results(file_name, ax_h, n_robots,n_data,robot_naming)

T = readtable(file_name);

rsi = zeros(1,n_robots); % robot start index
ep = n_robots*(2*n_data)+5;
df = (2*n_data);
rsi = 6:df:ep;
% % 
% % % figures and axes definition
% vel_fig = figure;
% ace_fig = figure;
% input_fig = figure;
% inputf_fig = figure;
% inputo_fig = figure;
% inputn_fig = figure;
% dist_fig = figure;
% ref_fig = figure;
% 
% 
% vel_ax = axes('Parent',vel_fig);
% ace_ax = axes('Parent',ace_fig);
% input_ax = axes('Parent',input_fig);
% inputf_ax = axes('Parent',inputf_fig);
% inputo_ax = axes('Parent',inputo_fig);
% inputn_ax = axes('Parent',inputn_fig);
% dist_ax = axes('Parent',dist_fig);
% ref_axe = axes('Parent',ref_fig);

t = table2array(T(:,1));
cqx = table2array(T(:,2));
cqy = table2array(T(:,3));
cpx = table2array(T(:,4));
cpy = table2array(T(:,5));

% compute centroid data
cent_qx =  mean(table2array(T(:,rsi)),2);
cent_qy =  mean(table2array(T(:,rsi+1)),2);
cent_px =  mean(table2array(T(:,rsi+2)),2);
cent_py =   mean(table2array(T(:,rsi+3)),2);
cent_ax =  mean(table2array(T(:,rsi+4)),2);
cent_ay =   mean(table2array(T(:,rsi+5)),2);

figure
% plot command vs ref 
ref_ax1 = subplot(2,2,1);
plot(t,cqx,'DisplayName','Command_x')
hold on 
plot(ref_ax1,t,cent_qx,'DisplayName','Centroid_x')
hold off
legend(ref_ax1)
ref_ax2 = subplot(2,2,2);
plot(ref_ax2,t,cqy,'DisplayName','Command_y')
hold on 
plot(ref_ax2,t,cent_qy,'DisplayName','Centroid_y')
hold off
legend(ref_ax2)
ref_ax3 = subplot(2,2,3);
plot(ref_ax3,t,cpx,'DisplayName','Command_{vx}')
hold on 
plot(ref_ax3,t,cent_px,'DisplayName','Centroid_{vx}')
hold off
legend(ref_ax3)
ref_ax4 = subplot(2,2,4);
plot(ref_ax4,t,cpy,'DisplayName','Command_{vy}')
hold on 
plot(ref_ax4,t,cent_py,'DisplayName','Centroid_{vy}')
hold off
legend(ref_ax4)
sgtitle("Reference vs real: centroid command")

%% Distance plot 
dist_fig = figure;
dist_axes = axes('Parent',dist_fig);
names = [];
for i = 1:(n_robots)
    for j = 1: n_robots
        if i ~=j 
            hold(dist_axes,"on");
            x1 = table2array(T(:,rsi(i)));
            x2 = table2array(T(:,rsi(j)));
            y1 = table2array(T(:,rsi(i)+1));
            y2 = table2array(T(:,rsi(j)+1));
            dist = vecnorm([(x1-x2)';(y1-y2)']);
            n_ = "D"+ num2str(i) +num2str(j);

            names = [names,n_];
            plot(dist_axes, t, dist,"DisplayName",n_);

        end

    end
end
hold(dist_axes,"off");
legend(dist_axes);

%% vel acce
vel_fig = figure;
vel_ax1 = axes('Parent',vel_fig);
vel_ax2 = axes('Parent',vel_fig);
vel_ax3 = axes('Parent',vel_fig);
subplot(2,2,1,vel_ax1);
subplot(2,2,2,vel_ax2);
subplot(2,2,[ 3 4],vel_ax3);
sgtitle(vel_fig,"Velocity")


%% accel acce
acel_fig = figure;
acel_ax1 = axes('Parent',acel_fig);
acel_ax2 = axes('Parent',acel_fig);
acel_ax3 = axes('Parent',acel_fig);
subplot(2,2,1,acel_ax1);
subplot(2,2,2,acel_ax2);
subplot(2,2,[ 3 4],acel_ax3);
sgtitle(acel_fig,"Acceleration")

%% input 
input_fig = figure;
input_ax1 = axes('Parent',input_fig);
input_ax2 = axes('Parent',input_fig);
input_ax3 = axes('Parent',input_fig);
input_ax4 = axes('Parent',input_fig);
input_ax5 = axes('Parent',input_fig);
input_ax6 = axes('Parent',input_fig);
input_ax7 = axes('Parent',input_fig);
input_ax8 = axes('Parent',input_fig);
input_ax9 = axes('Parent',input_fig);
input_ax10 = axes('Parent',input_fig);
input_ax11 = axes('Parent',input_fig);
input_ax12 = axes('Parent',input_fig);
subplot(5,2,1,input_ax1);
subplot(5,2,2,input_ax2);
subplot(5,2,3,input_ax3);
subplot(5,2,4,input_ax4);
subplot(5,2,5,input_ax5);
subplot(5,2,6,input_ax6);
subplot(5,2,7,input_ax7);
subplot(5,2,8,input_ax8);
subplot(5,2,9,input_ax9);
subplot(5,2,10,input_ax10);
subplot(5,2,9,input_ax11);
subplot(5,2,10,input_ax12);
sgtitle(input_fig,"Acceleration")


end_point = zeros(n_robots,2);

% plot positions
for i = 1:n_robots


    x = table2array(T(:,rsi(i)));
    y = table2array(T(:,rsi(i)+1));
    vx = table2array(T(:,rsi(i)+2));
    vy = table2array(T(:,rsi(i)+3));
    ax = table2array(T(:,rsi(i)+4));
    ay = table2array(T(:,rsi(i)+5));
    inpx = table2array(T(:,rsi(i)+6));
    inpy = table2array(T(:,rsi(i)+7));
    inpfx = table2array(T(:,rsi(i)+8));
    inpfy = table2array(T(:,rsi(i)+9));
    inpox = table2array(T(:,rsi(i)+10));
    inpoy = table2array(T(:,rsi(i)+11));
    inpnx = table2array(T(:,rsi(i)+12));
    inpny = table2array(T(:,rsi(i)+13));
    inpnorix = table2array(T(:,rsi(i)+14));
    inpnoriy = table2array(T(:,rsi(i)+15));
    inpnintx = table2array(T(:,rsi(i)+14));
    inpninty = table2array(T(:,rsi(i)+15));
%     x = i*x/max(x);
%     y = i*y/max(y);
%     vx = i*vx/max(vx);
%     vy = i*vy/max(vy);
%     ax = i*ax/max(ax);
%     ay = i*ay/max(ay);
%     inpx = i*inpx/max(inpx);
%     inpy = i*inpy/max(inpy);
%     inpfx = i*inpfx/max(inpfx);
%     inpfy = i*inpfy/max(inpfy);
%     inpox = i*inpox/max(inpox);
%     inpoy = i*inpoy/max(inpoy);
%     inpnx = i*inpnx/max(inpnx);
%     inpny = i*inpny/max(inpny);
%      inpnx = i*inpnx/max(inpnx);
%     inpny = i*inpny/max(inpny);
    % position on map
    hold(ax_h, "on")
%     hold(vel_ax, "on")
    hold(vel_ax1, "on")
    hold(vel_ax2, "on")
    hold(vel_ax3, "on")

    hold(acel_ax1, "on")
    hold(acel_ax2, "on")
    hold(acel_ax3, "on")

    hold(input_ax1, "on")
    hold(input_ax2, "on")
    hold(input_ax3, "on")
    hold(input_ax4, "on")
    hold(input_ax5, "on")
    hold(input_ax6, "on")
    hold(input_ax7, "on")
    hold(input_ax8, "on")
    hold(input_ax9, "on")
    hold(input_ax10, "on")
    hold(input_ax11, "on")
    hold(input_ax12, "on")

    end_point(i,:) = [x(end),y(end) ];
   
    plot(ax_h, x,y,".",'DisplayName', [char(robot_naming(i))],"MarkerSize",7)
    % vel
    plot(vel_ax1, t,vx,".",'DisplayName', [char(robot_naming(i)) '_{vx}'])
    plot(vel_ax2, t,vy,".",'DisplayName', [char(robot_naming(i)) '_{vy}'])
    plot(vel_ax3, t,sqrt(vx.*vx+vy.*vy),".",'DisplayName', [char(robot_naming(i))])
    % Acel
    plot(acel_ax1, t,ax,".",'DisplayName', [char(robot_naming(i)) '_{ax}'])
    plot(acel_ax2, t,ay,".",'DisplayName', [char(robot_naming(i)) '_{ay}'])
    plot(acel_ax3, t,sqrt(ax.*ax+ay.*ay),".",'DisplayName', [char(robot_naming(i))])
    % Inputs 
    plot(input_ax1, t,inpx,".",'DisplayName', [char(robot_naming(i)) '_{inputx}'])
    plot(input_ax2, t,inpy,".",'DisplayName', [char(robot_naming(i)) '_{inputy}'])
    plot(input_ax3, t,inpfx,".",'DisplayName', [char(robot_naming(i)) '_{inputformx}'])
    plot(input_ax4, t,inpfy,".",'DisplayName', [char(robot_naming(i)) '_{inputformy}'])
    plot(input_ax5, t,inpox,".",'DisplayName', [char(robot_naming(i)) '_{inputobsx}'])
    plot(input_ax6, t,inpoy,".",'DisplayName', [char(robot_naming(i)) '_{inputobsy}'])
    plot(input_ax7, t,inpnx,".",'DisplayName', [char(robot_naming(i)) '_{inputnavx}'])
    plot(input_ax8, t,inpny,".",'DisplayName', [char(robot_naming(i)) '_{inputnav_y}'])
    plot(input_ax9, t,inpnorix,".",'DisplayName', [char(robot_naming(i)) '_{inputorix}'])
    plot(input_ax10, t,inpnoriy,".",'DisplayName', [char(robot_naming(i)) '_{inputoriy}'])
    plot(input_ax11, t,inpnintx,".",'DisplayName', [char(robot_naming(i)) '_{inputintx}'])
    plot(input_ax12, t,inpninty,".",'DisplayName', [char(robot_naming(i)) '_{inputinty}'])
    
 
  






end

for i = 1:n_robots
 
     n = i+1;
    if n > n_robots
        n = 1;
    end
    

    plot(ax_h, [end_point(i,1) end_point(n,1)],[end_point(i,2) end_point(n,2)],'DisplayName', [char(robot_naming(i))],"MarkerSize",7)



end

    hold(ax_h, "off")
    hold(vel_ax1, "off")
    hold(vel_ax2, "off")
    hold(vel_ax3, "off")
    hold(acel_ax1, "off")
    hold(acel_ax2, "off")
    hold(acel_ax3, "off")
    hold(input_ax1, "off")
    hold(input_ax2, "off")
    hold(input_ax3, "off")
    hold(input_ax4, "off")
    hold(input_ax5, "off")
    hold(input_ax6, "off")
    hold(input_ax7, "off")
    hold(input_ax8, "off")
        hold(input_ax9, "off")
    hold(input_ax10, "off")
   hold(input_ax11, "off")
    hold(input_ax12, "off")

    legend(vel_ax1)
    legend(vel_ax2)
    legend(vel_ax3)
    legend(acel_ax1)
    legend(acel_ax2)
    legend(acel_ax3)
    legend(input_ax1)
    legend(input_ax2)
    legend(input_ax3)
    legend(input_ax4)
    legend(input_ax5)
    legend(input_ax6)
     legend(input_ax7)
    legend(input_ax8)
      legend(input_ax9)
    legend(input_ax10)
        legend(input_ax11)
    legend(input_ax12)
end