function animation_data = PlotPosition(r, t, param, animation_data)

%plot position, given Pose
%parameters contains: [r_f,r_e,f,e,z=symbolic value]

%% init
%coords
x=r(1); y=r(2); z=r(3);
t1=t(1); t2=t(2); t3=t(3);
%rod lengths in m:
r_f=param(1); 
r_e=param(2);
hold on;

%triangular side lengths in m:
f=param(3); 
e=param(4);

%% Plot
%init
grid on
l=eval(f+2*r_f); %to adjust the view
axis([-l l -l l -l*2 l/2])
cla;

%calc and plot robot f plate
P_f1=[f/2,-f/(2*sqrt(3)),0];
P_f2=[0,f/cos(30*pi/180)/2,0];
P_f3=[-f/2,-f/(2*sqrt(3)),0];

line([P_f1(1) P_f2(1)],[P_f1(2) P_f2(2)],[P_f1(3) P_f2(3)],'LineWidth', 2, 'Color', [0.7 0.7 0.7])
line([P_f2(1) P_f3(1)],[P_f2(2) P_f3(2)],[P_f2(3) P_f3(3)],'LineWidth', 2, 'Color', [0.7 0.7 0.7])
line([P_f3(1) P_f1(1)],[P_f3(2) P_f1(2)],[P_f3(3) P_f1(3)],'LineWidth', 2, 'Color', [0.7 0.7 0.7])

hold on
%calc and plot robot e plate
P_e1=r'+[e/2,-e/(2*sqrt(3)),0];
P_e2=r'+[0,e/cos(30*pi/180)/2,0];
P_e3=r'+[-e/2,-e/(2*sqrt(3)),0];

line([P_e1(1) P_e2(1)],[P_e1(2) P_e2(2)],[P_e1(3) P_e2(3)],'LineWidth', 2, 'Color', [0.7 0.2 0.3])
line([P_e2(1) P_e3(1)],[P_e2(2) P_e3(2)],[P_e2(3) P_e3(3)],'LineWidth', 2, 'Color', [0.7 0.2 0.3])
line([P_e3(1) P_e1(1)],[P_e3(2) P_e1(2)],[P_e3(3) P_e1(3)],'LineWidth', 2, 'Color', [0.7 0.2 0.3])

hold on

plot3(x,y,z,'o','color','green')%r-pos

%calc and plot joints f
P_F1=[0,-f/(2*sqrt(3)),0];
%use rotation matrix to calc other points
deg=120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_F2=(R*P_F1')';
P_F3=(R*P_F2')';

hold on
plot3(P_F1(1),P_F1(2),P_F1(3),'o','color','green')%r-pos
hold on
plot3(P_F2(1),P_F2(2),P_F2(3),'o','color','green')%r-pos
hold on
plot3(P_F3(1),P_F3(2),P_F3(3),'o','color','green')%r-pos
hold on

P_J1=[0,-f/(2*sqrt(3))-r_f*cos(t1),-r_f*sin(t1)];
P_J2=(R*[0,-f/(2*sqrt(3))-r_f*cos(t2),-r_f*sin(t2)]')';
deg=-120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_J3=(R*[0,-f/(2*sqrt(3))-r_f*cos(t3),-r_f*sin(t3)]')';

line([P_F1(1) P_J1(1)],[P_F1(2) P_J1(2)],[P_F1(3) P_J1(3)],'color','black','LineWidth',3)
line([P_F2(1) P_J2(1)],[P_F2(2) P_J2(2)],[P_F2(3) P_J2(3)],'color','black','LineWidth',3)
line([P_F3(1) P_J3(1)],[P_F3(2) P_J3(2)],[P_F3(3) P_J3(3)],'color','black','LineWidth',3)

hold on
plot3(P_J1(1),P_J1(2),P_J1(3),'o','color','green')%r-pos
hold on
plot3(P_J2(1),P_J2(2),P_J2(3),'o','color','green')%r-pos
hold on
plot3(P_J3(1),P_J3(2),P_J3(3),'o','color','green')%r-pos
hold on

%calc and plot joint e
Transl=[0,-e/(2*sqrt(3)),0];
P_E1=Transl+[x,y,z];
deg=120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_E2=((R*Transl')+r)';
deg=-120;
R=[cos(deg*pi/180),-sin(deg*pi/180),0;sin(deg*pi/180),cos(deg*pi/180),0;0,0,1];
P_E3=((R*Transl')+r)';

line([P_J1(1) P_E1(1)],[P_J1(2) P_E1(2)],[P_J1(3) P_E1(3)],'color','yellow','LineWidth',2)
line([P_J2(1) P_E2(1)],[P_J2(2) P_E2(2)],[P_J2(3) P_E2(3)],'color','yellow','LineWidth',2)
line([P_J3(1) P_E3(1)],[P_J3(2) P_E3(2)],[P_J3(3) P_E3(3)],'color','yellow','LineWidth',2)

hold on
xlabel(['x = ' num2str(x)]);
ylabel(['y = ' num2str(y)]);
zlabel(['z = ' num2str(z)]);
 
 % Append the end effector position to the visited points matrix
    animation_data.visitedPoints = [animation_data.visitedPoints; [x, y, z]];

    % Calculate and display the workspace (visited points)
    workspace = unique(animation_data.visitedPoints, 'rows');

    % Use the provided figure handle or create a new figure
    if ~ishandle(animation_data.figure_handle)
        animation_data.figure_handle = figure;
    else
        figure(animation_data.figure_handle);
    end

    % Plot the workspace points (use 'hold on' to keep the current plot)
    hold on;
    scatter3(workspace(:, 1), workspace(:, 2), workspace(:, 3), 'filled', 'MarkerEdgeColor', 'k', 'MarkerFaceColor', 'g');

  


end





