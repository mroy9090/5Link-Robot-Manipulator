% Define the robot model
L1 = Link('d', 0, 'a', 1, 'alpha', 0, 'm', 10, 'r', [0 0 0]);
L2 = Link('d', 0, 'a', 1, 'alpha', 0, 'm', 10, 'r', [0 0 0]);
L3 = Link('d', 0, 'a', 0, 'alpha', 0, 'm', 10, 'r', [0 0 0]);
L4 = Link('d', -1, 'a', 0, 'alpha', 0, 'm',10, 'r', [0 0 0]);
L5 = Link('d', 0, 'a', 1, 'alpha', 0, 'm', 10, 'r', [0 0 0]);

robot = SerialLink([L1 L2 L3 L4 L5], 'name', '5 Link Manipulator');

% Define the initial joint angles and desired joint angles
q0 = [0 0 0 0 0];
qd = [90 45 -30 -90 0.5];

% Define the gains for the PD controller
Kp = 10;
Kd = 5;

% Set up the control loop
dt = 0.01;  % Time step
t = 0;      % Starting time
q = q0;     % Initial joint angles
q_dot = zeros(1, 5);  % Initial joint velocities
tau = zeros(1, 5);    % Initial joint torques

while norm(q - qd) > 0.01  % Continue the loop until the joint angles are close to the desired angles
    % Compute the joint torques using the PD controller
    tau = Kp*(qd - q) - Kd*q_dot;
    
    % Apply the torques to the robot model
    robot.plot(q);
    robot.animate(q_dot,tau);
    % robot = robot.fkine(q);
    % robot = robot.rne(q_dot, tau);
    
    % Update the joint angles and velocities using the Euler method
    q_dot = q_dot + tau*dt;
    q = q + q_dot*dt;
    
    % Update the time
    t = t + dt;
end

disp('Final joint angles:');
disp(q);