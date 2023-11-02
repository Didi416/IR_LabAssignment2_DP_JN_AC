classdef Lab2_Assignment < handle
    properties (Access = private)
        lab2GuiApp  % Instance of Lab2_GUI
    end

    properties (Constant)
        estopStatus = 'Run'; % Initialize to 'Run'
    end

    properties
        robot1 % Instance of the first robot
        robot2 % Instance of the second robot
        % Instances of grippers
        Claw1_1
        Claw1_2
        Claw2_1
        Claw2_2
    end

    %% Non-Static Methods
    methods
        %% Constructor
        function self = Lab2_Assignment()
            clc
            clf
            % Create an instance of Lab2_GUI
            % self.lab2GuiApp = Lab2_GUI();
            Environment_Lab2();
            self.initialiseRobots(-0.8,0,0);
            input("Press ENTER to start.");
            self.main();
        end
        %% Start GUI App and Enable eStop functionalities
        function startApp(self)
            % Start the Lab2_GUI app
            self.lab2GuiApp.UIFigure.Visible = 'on';
        end
        function toggleEstop(estopStatus)
            % Toggle the E-stop status between 'Run' and 'Stop'
            if strcmp(estopStatus, 'Run')
                estopStatus = 'Run';
            else
                estopStatus = 'Stop';
            end
        end
        %% Initialise Robots in Environment
        function initialiseRobots(self, X, Y, Z)
            % Initialise robot instances
            baseTr = transl(X, Y, Z);
            self.robot1 = LinearUR10(baseTr);
            self.robot2 = Robo();
            startQ1 = deg2rad([0,90,45,120,45,-90,0]);
            self.robot1.model.animate(startQ1);
            startQ2 = deg2rad([0,-65,100,-180,-90,0]);
            self.robot2.model.animate(startQ2)
            self.clawAttach();
        end
        %% Attach Gripper to Robot End Effectors
        function clawAttach(self)
            self.Claw1_1 = TheClaww();
            self.Claw1_2 = TheClaww();
            self.Claw2_1 = TheClaww();
            self.Claw2_2 = TheClaww();

            endQ1 = self.robot1.model.getpos();
            endQ2 = self.robot2.model.getpos();
            q = [pi/2,0,-pi/4];
            % Linear UR10 Gripper
            self.Claw1_1.model.base = self.robot1.model.fkine(endQ1).T * transl(0,0,0.09);
            self.Claw1_1.model.animate(q);
            drawnow;
            self.Claw1_2.model.base = self.robot1.model.fkine(endQ1).T * trotz(pi) * transl(0,0,0.09);
            self.Claw1_2.model.animate(q);
            drawnow;
            % ShelfElf Gripper
            self.Claw2_1.model.base = self.robot2.model.fkine(endQ2).T;
            self.Claw2_1.model.animate(q);
            drawnow;
            self.Claw2_2.model.base = self.robot2.model.fkine(endQ2).T * trotz(pi);
            self.Claw2_2.model.animate(q);
            drawnow;
        end

        %% Main Function
        function main(self)
            % inital placements of boxes
            boxPos = zeros(12,3);
            boxes = cell(1,12);
            verticesArray = zeros(24,3,6);
            x = -1.4;
            i = 1;

            for y = -0.2:0.6:1.0
                for z = 0.2:0.28:0.48
                    boxes{i} = PlaceObject('box.ply');
                    vertices = get(boxes{i}, 'Vertices');
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(x,y,z)';
                    set(boxes{i}, 'Vertices', transformedVertices(:,1:3));
                    verticesArray(:,:,i) = vertices;
                    boxPos(i,1) = x;
                    boxPos(i,2) = y;
                    boxPos(i,3) = z;
                    i = i+1;
                end
                for z = 1.0:0.28:1.28
                    boxes{i} = PlaceObject('box.ply');
                    vertices = get(boxes{i}, 'Vertices');
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(x,y,z)';
                    set(boxes{i}, 'Vertices', transformedVertices(:,1:3));
                    verticesArray(:,:,i) = vertices;
                    boxPos(i,1) = x;
                    boxPos(i,2) = y;
                    boxPos(i,3) = z;
                    i = i+1;
                end
            end

            elbowUp1 = deg2rad([0,90,45,120,45,-90,0]);
            elbowUp2 = deg2rad([0,-65,100,-180,-90,0]);

            for i = 1:2
                numSteps = 100;
                % x = boxPos(i,1) + 0.18;
                % y = boxPos(i,2);
                % z = boxPos(i,3) + 0.035;
                % 
                % % Calculate start and end pose for the RMRC
                % currentPos = self.robot1.model.getpos();
                % pickUpBox = transl(x, y, z) * troty(-pi/2);
                % pickUpBoxEndPose = self.robot1.model.ikcon(pickUpBox, elbowUp1);
                % robot1Trajectory = jtraj(currentPos, pickUpBoxEndPose, numSteps);
                % q = [pi/2,0,-pi/4];
                % for x = 1:numSteps
                %    endEffPos = self.robot1.model.getpos();
                %    self.robot1.model.animate(robot1Trajectory(x,:));
                %    drawnow();
                %    self.Claw1_1.model.base = self.robot1.model.fkine(endEffPos).T * transl(0,0,0.09);
                %    self.Claw1_1.model.animate(q);
                %    drawnow();
                %    self.Claw1_2.model.base = self.robot1.model.fkine(endEffPos).T * trotz(pi) * transl(0,0,0.09);
                %    self.Claw1_2.model.animate(q);
                %    drawnow();
                % end
                % disp("Robot arm positioned.");
                % self.gripperOpenClose(0);

                currentPos2 = self.robot2.model.getpos();
                pickup2 = transl(-0.5, 1.5, 0.8);
                pickUpPos2 = self.robot2.model.ikcon(pickup2,elbowUp2);

                % Call the RMRC function instead of using ikcon
                qMatrix = self.RMRC(self.robot2.model, currentPos2, pickUpPos2, numSteps, elbowUp2);
                for r = 1:numSteps
                    self.robot2.model.animate(qMatrix(r,:))
                    drawnow();
                end
            end
        end

        function qMatrix = RMRC(self, robot, currentPos, endPose, numSteps, elbowUp)
            t = 10;             % Total time (s)
            deltaT = 0.02;      % Control frequency
            steps = t/deltaT;   % No. of steps for simulation
            epsilon = 0.5;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory

            robotTrajectory = jtraj(currentPos, endPose, numSteps)
            % s = lspb(0,1,steps);                                                      % Trapezoidal trajectory scalar
            for i = 1:numSteps
                T = robot.fkine(robotTrajectory(i,:)).T;
                x(1,i) = T(1,4);                                                        % Points in x
                x(2,i) = T(2,4);                                                        % Points in y
                x(3,i) = T(3,4);                                                        % Points in z
                theta(1,i) = atan(T(2,1)/T(1,1));                                       % Roll angle
                theta(2,i) = atan(-T(3,1)/sqrt((T(3,2)^2)+(T(3,3)^2)));                 % Pitch angle
                theta(3,i) = atan(T(3,2)/T(3,3));                                       % Yaw angle
                disp("Built traj.")
            end

            T = [rpy2r(theta(1,1),theta(2,1),theta(3,1)) x(:,1);zeros(1,3) 1];          % Create transformation matrix of first point and angle
            qMatrix(1,:) = robot.ikcon(T,elbowUp);                                      % Solve joint angles to achieve first waypoint

            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:)).T;                                        % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);                                         	% Get position error from next waypoint
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           % Skew symmetric!
                linear_velocity = (1/deltaT)*deltaX;
                angular_velocity = [S(3,2);S(1,3);S(2,1)];                              % Check the structure of Skew Symmetric matrix!!
                % deltaTheta = tr2rpy(Rd*Ra');                                          % Convert rotation matrix to RPY angles
                xdot = W*[linear_velocity;angular_velocity];                          	% Calculate end-effector velocity to reach next waypoint.
                J = robot.jacob0(qMatrix(i,:));                                         % Get Jacobian at current joint state
                m(i) = sqrt(det(J*J'));
                if m(i) < epsilon  % If manipulability is less than given threshold
                    lambda = (1 - m(i)/epsilon)*5E-2;
                else
                    lambda = 0;
                end
                invJ = inv(J'*J + lambda *eye(6))*J';                                   % DLS Inverse
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation (you may need to transpose the         vector)
                for j = 1:6                                                             % Loop through joints 1 to 6
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)            % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                disp("Stage 2")
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
            end
        end

        function gripperOpenClose(self, state)
            if state % open
                q = [pi/2,0,-pi/4];
            else % close
                q = [pi/2,0,-pi/2];
            end
            gripperPos = self.Claw1_1.model.getpos();
            endGripperPos = q;
            gripperTraj = jtraj(gripperPos, endGripperPos, 30);
            for i = 1:30
                self.Claw1_1.model.animate(gripperTraj(i,:));
                drawnow();
                self.Claw1_2.model.animate(gripperTraj(i,:));
                drawnow();
            end
        end
    end

    %{
       function collisionDetected = IsCollisionWithEnvironment(currentTransform, environmentObjects)
           % Check for collisions between robot and environment objects
           % Implement your collision detection logic here
           collisionDetected = false; % Update this based on collision checking
       end        
    %}

end
