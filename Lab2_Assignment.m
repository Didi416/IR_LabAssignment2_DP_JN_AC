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
            Environment_Lab2Angie();
            self.initialiseRobots(-0.6,0,0);
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
            %--------------------------------------------------------------------------------------------%
            elbowUp1 = deg2rad([0,90,45,120,45,-90,0]);
            elbowUp2 = deg2rad([0,-100,100,-180,-90,0]);
            q = [pi/2,0,-pi/4];
            steps = 100;
            numSteps = 500;
            boxNum = 3;
            vertices = verticesArray(:,:,boxNum);

            % LinearUR10 movements - no RMRC
            x = boxPos(boxNum,1) + 0.18;
            y = boxPos(boxNum,2);
            z = boxPos(boxNum,3) + 0.035;

            % - point in front of box position
            xF = x + 0.2;
            currentPos = self.robot1.model.getpos();
            endTrans = transl(xF, y, z) * troty(-pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0)

            % - box position
            currentPos = self.robot1.model.getpos();
            endTrans = transl(x, y, z) * troty(-pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0)
            q = self.gripperOpenClose(0);

            % - back out in front of position
            currentPos = self.robot1.model.getpos();
            endTrans = transl(xF, y, z) * troty(-pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 1)

            % - counter near Robo
            currentPos = self.robot1.model.getpos();
            endTrans = transl(-0.2, 1.5, 0.8) * troty(pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 1)
            q = self.gripperOpenClose(1);

            % - default pos
            currentPos = self.robot1.model.getpos();
            endTrans = transl(-0.5, 1, 1) * troty(pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0)

            % Robo Movements - RMRC
            currentPos2 = self.robot2.model.fkine(self.robot2.model.getpos()).T;
            endTrans2 = transl(-0.2, 1.5, 1);
            qMatrix = self.RMRC(self.robot2.model, currentPos2, endTrans2, elbowUp2);
            for r = 1:numSteps
                self.robot2.model.animate(qMatrix(r,:))
                drawnow();
                endEffPos = self.robot2.model.getpos();
                self.Claw2_1.model.base = self.robot2.model.fkine(endEffPos).T;
                self.Claw2_1.model.animate(q);
                drawnow();
                self.Claw2_2.model.base = self.robot2.model.fkine(endEffPos).T * trotz(pi);
                self.Claw2_2.model.animate(q);
                drawnow();
            end

            currentPos2 = self.robot2.model.fkine(self.robot2.model.getpos()).T;
            endTrans2 = transl(0.2, 1, 1);
            qMatrix = self.RMRC(self.robot2.model, currentPos2, endTrans2, elbowUp2);
            for r = 1:numSteps
                pickUpPose = self.robot2.model.fkine(self.robot2.model.getpos).T * transl(0,0,0.2) * troty(pi/2);
                transformedVertices = [vertices, ones(size(vertices,1),1)] * pickUpPose';
                set(boxes{boxNum}, 'Vertices', transformedVertices(:,1:3));
                drawnow();

                self.robot2.model.animate(qMatrix(r,:))
                drawnow();
                endEffPos = self.robot2.model.getpos();
                self.Claw2_1.model.base = self.robot2.model.fkine(endEffPos).T;
                self.Claw2_1.model.animate(q);
                drawnow();
                self.Claw2_2.model.base = self.robot2.model.fkine(endEffPos).T * trotz(pi);
                self.Claw2_2.model.animate(q);
                drawnow();
            end
        end

        function UR10Move(self, robot, currentPos, endPose, numSteps, q, vertices, boxId, box)
            robot1Trajectory = jtraj(currentPos, endPose, numSteps);
            verts = vertices;
            for x = 1:numSteps
                if box == 1
                    pickUpPose = robot.fkine(robot.getpos).T * transl(0,0,0.2) * troty(pi/2);
                    transformedVertices = [verts, ones(size(verts,1),1)] * pickUpPose';
                    set(boxId, 'Vertices', transformedVertices(:,1:3));
                    drawnow();
                end
                robot.animate(robot1Trajectory(x,:));
                drawnow();
                endEffPos = robot.getpos();
                self.Claw1_1.model.base = robot.fkine(endEffPos).T * transl(0,0,0.09);
                self.Claw1_1.model.animate(q);
                drawnow();
                self.Claw1_2.model.base = robot.fkine(endEffPos).T * trotz(pi) * transl(0,0,0.09);
                self.Claw1_2.model.animate(q);
                drawnow();
            end
            disp("Robot arm positioned.");
        end
        function qMatrix = RMRC(self, robot, currentPos, endPose, elbowUp)
            t = 10;             % Total time (s)
            deltaT = 0.02;      % Control frequency
            steps = 500;   % No. of steps for simulation
            epsilon = 0.5;      % Threshold value for manipulability/Damped Least Squares
            W = diag([1 1 1 0.1 0.1 0.1]);    % Weighting matrix for the velocity vector

            m = zeros(steps,1);             % Array for Measure of Manipulability
            qMatrix = zeros(steps,6);       % Array for joint anglesR
            qdot = zeros(steps,6);          % Array for joint velocities
            theta = zeros(3,steps);         % Array for roll-pitch-yaw angles
            x = zeros(3,steps);             % Array for x-y-z trajectory

            s = lspb(0,1,steps);                                                        % Trapezoidal trajectory scalar
            for i = 1:steps
                x(1,i) = (1-s(i))*currentPos(1,4) + s(i)*endPose(1,4);                  % Points in x
                x(2,i) = (1-s(i))*currentPos(2,4) + s(i)*endPose(2,4);                  % Points in y
                x(3,i) = (1-s(i))*currentPos(3,4) + s(i)*endPose(3,4);                  % Points in z
                theta(1,i) = 0;                                       % Roll angle
                theta(2,i) = 0;                 % Pitch angle
                theta(3,i) = 0;                                       % Yaw angle
            end

            T = currentPos;          % Create transformation of first point and angle
            qMatrix(1,:) = robot.ikcon(T,elbowUp);                                      % Solve joint angles to achieve first waypoint

            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:)).T;                                        % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);   
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
                    if qMatrix(i,j) + deltaT*qdot(i,j) < robot.qlim(j,1)                 % If next joint angle is lower than joint limit...
                        qdot(i,j) = 0; % Stop the motor
                    elseif qMatrix(i,j) + deltaT*qdot(i,j) > robot.qlim(j,2)             % If next joint angle is greater than joint limit ...
                        qdot(i,j) = 0; % Stop the motor
                    end
                end
                qMatrix(i+1,:) = qMatrix(i,:) + deltaT*qdot(i,:);                       % Update next joint state based on joint velocities
            end
        end

        function q = gripperOpenClose(self, state) % 0 = close, 1 = open
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
