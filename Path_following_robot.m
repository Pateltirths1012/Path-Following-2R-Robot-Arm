% Making a path following robot
% A 2R robot with 2 links each of 2 units

clc; close all; clear;
format compact;
set(0,'DefaultFigureWindowStyle','docked');

figure(1); hold on; grid on;
axis([-4.5 4.5 -4.5 4.5]);

% Set up the Video Writer
outputVideo = VideoWriter('2R_path_following_robot_simulation.avi'); % Create video file
outputVideo.FrameRate = 30; % Set frame rate
open(outputVideo); % Open the video file for writing

%Draw the fixed pivot at the origin
plot(0,0,'ro',"LineWidth",3,'tag','delete me');


% Click to enter the direction of the first link
[x,y,button] = ginput(1);
plot(x,y,'k+'); %plots the user input
theta1 = atan2(y,x);

% Making the first link
L1 = 2*[cos(theta1); sin(theta1)];
plot(L1(1),L1(2),'go','LineWidth',3);
plot([0 L1(1)],[0 L1(2)],'c--','LineWidth',2);

% Click to enter the direction of the second link
[x,y,button] = ginput(1);
plot(x,y,'k+'); %plots the user input
theta12 = atan2(y-L1(2),x-L1(1)); %this is the sum of theta1 & theta2

%Making the second link
L2 = 2*[cos(theta12); sin(theta12)];
plot(L1(1)+L2(1),L1(2)+L2(2),'mo','LineWidth',3);
plot([L1(1) L1(1)+L2(1)],[L1(2) L1(2)+L2(2)],'c--','LineWidth',2);


theta2 = theta12 - theta1;

%Path of the robot

xx = 1;
jj = 1;

while xx == 1
    [xf(jj), yf(jj),button] = ginput(1);
    plot(xf,yf,'y-','LineWidth',3);
    jj = jj + 1;
    if button ==3
        xx = 0 ; %path terminates when the user right clicks
    end
end

B = L1 + L2;

for kk = 1:length(xf)
    x_curr = B(1);
    y_curr = B(2);
    x_new = xf(kk)-x_curr;
    y_new = yf(kk)-y_curr;

    %Dividing the path into small steps
    x_step = x_new/100;
    y_step = y_new/100;

    for ii = 1:100
        %Writing the Jacobian of the 2R robot
        J_RR = [-2*sin(theta1)-2*sin(theta12), -2*sin(theta12)
                2*cos(theta1)+2*cos(theta12),  2*cos(theta12)];
        J_inv = inv(J_RR); %inverse of the Jacobian

        Thetadot = J_inv*[x_step;y_step];
        Theta1dot = Thetadot(1);
        Theta2dot = Thetadot(2);
        Theta12dot = Theta1dot + Theta2dot;

        theta1new = theta1 + Theta1dot;
        theta12new = theta12 + Theta12dot;

        O = [0,0];
        A = O + 2*[cos(theta1new);sin(theta1new)];
        B = A + 2*[cos(theta12new);sin(theta12new)];

        %clearing up the old positions of the robot
        delete(findobj('tag','delete me'));

        % Now, lets draw the moving robot
        plot([O(1) A(1) B(1)],[O(2) A(2) B(2)],'k-','LineWidth',2,'tag','delete me');
        plot(O(1),O(2),'ro','LineWidth',3,'tag','delete me');
        plot(A(1),A(2),'go','LineWidth',3,'tag','delete me');
        plot(B(1),B(2),'mo','LineWidth',3,'tag','delete me');
        drawnow;

        % Capture the frame and write it to the video
        frame = getframe(gcf);
        writeVideo(outputVideo, frame);


        %updating the variables
        theta1 = theta1new;
        theta12 = theta12new;
    end

end

%close the video writer object
close(outputVideo)
























