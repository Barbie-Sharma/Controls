clear
clc

turtlebot = importrobot('robotisTurtleBot3Burger.urdf');

distance = 1;
Kp = 2;

wheelR = 0.033; 
wheelSeparation = 0.16; 
maxVelocity = 0.3; 
time_step = 0.1; 
currentDist = 0; 
pose = [0 0 0]; 
velocities = []; 
distances = [];
tolerance = 0.001;

jointPos = homeConfiguration(turtlebot);

LId = find(contains({jointPos.JointName}, 'wheel_left_joint'));
RId = find(contains({jointPos.JointName}, 'wheel_right_joint'));

while currentDist < distance - tolerance
    error = distance - currentDist;
    v = Kp * error;
    v = min(v, maxVelocity); 

    % angular velocity 
    omega = v / wheelR;
    leftOmega = omega; 
    rightOmega = omega;

    % rotating wheels
    jointPos(LId).JointPosition = jointPos(LId).JointPosition + leftOmega * time_step;
    jointPos(RId).JointPosition = jointPos(RId).JointPosition + rightOmega * time_step;
    
    % differential drive
    deltaTheta = wheelR/wheelSeparation * (rightOmega - leftOmega) * time_step;
    deltaX = wheelR/2 * cos(pose(3) + deltaTheta / 2) * (leftOmega + rightOmega) * time_step;
    deltaY = wheelR/2 * sin(pose(3) + deltaTheta / 2) * (leftOmega + rightOmega) * time_step;

    % updating values
    pose = pose + [deltaX, deltaY, deltaTheta];
    currentDist = currentDist + sqrt(deltaX^2 + deltaY^2);

    % Collect data for plotting
    velocities = [velocities, v];
    distances = [distances, currentDist];

    % trvec2tform: Cartesian to homogeneous transformation tform.
    tform = trvec2tform([pose(1:2), 0]);

    % rotation given in axis-angle form, axang, to an orthonormal rotation matrix, rotm
    tform(1:3, 1:3) = axang2rotm([0 0 1 pose(3)]);

    % updating the base link
    setFixedTransform(turtlebot.Bodies{1}.Joint, tform); 
  
    show(turtlebot, jointPos);
    axis([-1.5 1.5 -1.5 1.5 0 1.5]);
    pause(time_step);
end

% Plot the results
plot(distances, velocities, 'LineWidth', 2);
xlabel('Distance (m)');
ylabel('Velocity (m/s)');
title('TurtleBot P Control During Forward Movement');



