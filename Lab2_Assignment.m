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
        environmentVertices
        environmentFaces
        environmentFaceNormals
        detectCollision
    end

    %% Non-Static Methods
    methods
        %% Constructor
        function self = Lab2_Assignment()
            clc
            clf
            % Create an instance of Lab2_GUI
            % self.lab2GuiApp = Lab2_GUI();;
            [self.environmentVertices, self.environmentFaces, self.environmentFaceNormals] = Environment_Lab2();
            self.initialiseRobots(-0.5,0,0);
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
            startQ2 = deg2rad([0,-100,100,-180,-90,0]);
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
                    boxes{i} = PlaceObject('box.ply');                                              % Place boxes in positions
                    vertices = get(boxes{i}, 'Vertices');
                    faces = get(boxes{i}, 'Faces');
                    transformedVertices = [vertices,ones(size(vertices,1),1)] * transl(x,y,z)';     % use vertices to transform them to the desired position
                    set(boxes{i}, 'Vertices', transformedVertices(:,1:3));
                    verticesArray(:,:,i) = vertices;                                                % Store vertices in 3D array to access later
                    boxPos(i,1) = x;
                    boxPos(i,2) = y;
                    boxPos(i,3) = z;
                    i = i+1;
                    self.environmentVertices = cat(1,self.environmentVertices,vertices);            % add vertices and faces of boxes for collision detection
                    self.environmentFaces = cat(1, self.environmentFaces,faces);
                    for faceIndex = 1:size(faces,1)
                        v1 = self.environmentVertices(self.environmentFaces(faceIndex,1)',:);
                        v2 = self.environmentVertices(self.environmentFaces(faceIndex,2)',:);
                        v3 = self.environmentVertices(self.environmentFaces(faceIndex,3)',:);
                        self.environmentFaceNormals(end+1,:) = unit(cross(v2-v1,v3-v1));
                    end
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
                    self.environmentVertices = cat(1,self.environmentVertices,vertices);
                    self.environmentFaces = cat(1, self.environmentFaces,faces);
                    for faceIndex = 1:size(faces,1)
                        v1 = self.environmentVertices(self.environmentFaces(faceIndex,1)',:);
                        v2 = self.environmentVertices(self.environmentFaces(faceIndex,2)',:);
                        v3 = self.environmentVertices(self.environmentFaces(faceIndex,3)',:);
                        self.environmentFaceNormals(end+1,:) = unit(cross(v2-v1,v3-v1));
                    end
                end
            end
            %--------------------------------------------------------------------------------------------%
            elbowUp1 = deg2rad([0,90,45,120,45,-90,0]);                                             % Initial guess positions for UR10 (ikcon)
            elbowUp2 = deg2rad([0,-100,100,-180,-90,0]);                                            % Initial guess position for Robo (ikcon)
            q = [pi/2,0,-pi/4];
            % Setup for moving robots with either RMRC or IK
            steps = 100;
            numSteps = 300;
            boxNum = 6;
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
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0);

            % % - box position
            currentPos = self.robot1.model.getpos();
            endTrans = transl(x, y, z) * troty(-pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0);
            q = self.gripperOpenClose(0);

            % - back out in front of position
            currentPos = self.robot1.model.getpos();
            endTrans = transl(xF, y, z) * troty(-pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 1);

            % - lift up
            currentPos = self.robot1.model.getpos();
            endTrans = transl(-0.5, 1, 1) * troty(pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 1);

            % - counter near Robo
            currentPos = self.robot1.model.getpos();
            endTrans = transl(-0.2, 1.5, 0.8) * troty(pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 1);
            q = self.gripperOpenClose(1);

            % - default pos
            currentPos = self.robot1.model.getpos();
            endTrans = transl(-0.5, 1, 1) * troty(pi/2);
            endPose = self.robot1.model.ikcon(endTrans, elbowUp1);
            self.UR10Move(self.robot1.model, currentPos, endPose, steps, q, vertices, boxes{boxNum}, 0);

            % Robo Movements - RMRC
            currentPos2 = self.robot2.model.fkine(self.robot2.model.getpos()).T;
            endTrans2 = transl(0, 1.5, 1);
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
            q = self.gripper2OpenClose(0);

            currentPos2 = self.robot2.model.fkine(self.robot2.model.getpos()).T;
            endTrans2 = transl(0.2, 1, 1);
            qMatrix2 = self.RMRC(self.robot2.model, currentPos2, endTrans2, elbowUp2);
            for r = 1:numSteps
                pickUpPose = self.robot2.model.fkine(self.robot2.model.getpos).T * transl(0,0,0.02);
                transformedVertices = [vertices, ones(size(vertices,1),1)] * pickUpPose';
                set(boxes{boxNum}, 'Vertices', transformedVertices(:,1:3));
                drawnow();

                self.robot2.model.animate(qMatrix2(r,:))
                drawnow();
                endEffPos = self.robot2.model.getpos();
                self.Claw2_1.model.base = self.robot2.model.fkine(endEffPos).T;
                self.Claw2_1.model.animate(q);
                drawnow();
                self.Claw2_2.model.base = self.robot2.model.fkine(endEffPos).T * trotz(pi);
                self.Claw2_2.model.animate(q);
                drawnow();
            end
            q = self.gripper2OpenClose(1);

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
        end

        function UR10Move(self, robot, currentPos, endPose, numSteps, q, vertices, boxId, box)
            eVerts = self.environmentVertices;
            eFaces = self.environmentFaces;
            eFaceNorms = self.environmentFaceNormals;
            robot1Trajectory = jtraj(currentPos, endPose, numSteps);
            verts = vertices;
            for x = 1:numSteps
                if box == 1
                    pickUpPose = robot.fkine(robot.getpos).T * transl(0,0,0.2) * troty(pi/2);
                    transformedVertices = [verts, ones(size(verts,1),1)] * pickUpPose';
                    set(boxId, 'Vertices', transformedVertices(:,1:3));
                    drawnow();
                end
                % collision = IsCollision(robot,robot1Trajectory(x,:),eFaces,eVerts,eFaceNorms,false)
                robot.animate(robot1Trajectory(x,:));
                drawnow();
                % Move gripper with end effector at each step
                endEffPos = robot.getpos();
                self.Claw1_1.model.base = robot.fkine(endEffPos).T * transl(0,0,0.09);
                self.Claw1_1.model.animate(q);
                drawnow();
                self.Claw1_2.model.base = robot.fkine(endEffPos).T * trotz(pi) * transl(0,0,0.09);
                self.Claw1_2.model.animate(q);
                drawnow();
            end
        end
        function qMatrix = RMRC(self, robot, currentPos, endPose, elbowUp)
            t = 10;             
            deltaT = 0.02;      % Control frequency
            steps = 300;
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
                theta(1,i) = pi;                                                        % Roll angle
                theta(2,i) = 0;                                                         % Pitch angle
                theta(3,i) = 0;                                                         % Yaw angle
            end

            T = currentPos;                                                             % Create transformation of first point and angle
            qMatrix(1,:) = robot.ikcon(T,elbowUp);                                      % Solve joint angles to achieve first waypoint

            for i = 1:steps-1
                T = robot.fkine(qMatrix(i,:)).T;                                        % Get forward transformation at current joint state
                deltaX = x(:,i+1) - T(1:3,4);
                Rd = rpy2r(theta(1,i+1),theta(2,i+1),theta(3,i+1));                     % Get next RPY angles, convert to rotation matrix
                Ra = T(1:3,1:3);                                                        % Current end-effector rotation matrix
                Rdot = (1/deltaT)*(Rd - Ra);                                            % Calculate rotation matrix error
                S = Rdot*Ra';                                                           
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
                qdot(i,:) = (invJ*xdot)';                                               % Solve the RMRC equation, Jacobian relates joint velocities to end effector velocities
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
        function q = gripper2OpenClose(self, state) % 0 = close, 1 = open
            if state % open
                q = [pi/2,0,-pi/4];
            else % close
                q = [pi/2,0,-pi/2];
            end
            gripperPos = self.Claw2_1.model.getpos();
            endGripperPos = q;
            gripperTraj = jtraj(gripperPos, endGripperPos, 30);
            for i = 1:30
                self.Claw2_1.model.animate(gripperTraj(i,:));
                drawnow();
                self.Claw2_2.model.animate(gripperTraj(i,:));
                drawnow();
            end
        end

        %% IsIntersectionPointInsideTriangle
        % Given a point which is known to be on the same plane as the triangle
        % determine if the point is
        % inside (result == 1) or
        % outside a triangle (result ==0 )
        function result = IsIntersectionPointInsideTriangle(intersectP,triangleVerts)

            u = triangleVerts(2,:) - triangleVerts(1,:);
            v = triangleVerts(3,:) - triangleVerts(1,:);

            uu = dot(u,u);
            uv = dot(u,v);
            vv = dot(v,v);

            w = intersectP - triangleVerts(1,:);
            wu = dot(w,u);
            wv = dot(w,v);

            D = uv * uv - uu * vv;

            % Get and test parametric coords (s and t)
            s = (uv * wv - vv * wu) / D;
            if (s < 0.0 || s > 1.0)        % intersectP is outside Triangle
                result = 0;
                return;
            end

            t = (uv * wu - uu * wv) / D;
            if (t < 0.0 || (s + t) > 1.0)  % intersectP is outside Triangle
                result = 0;
                return;
            end

            result = 1;                      % intersectP is in Triangle
        end

        %% IsCollision
        function result = IsCollision(robot,qMatrix,faces,vertex,faceNormals,returnOnceFound)
            if nargin < 6
                returnOnceFound = true;
            end
            result = false;

            for qIndex = 1:size(qMatrix,1)
                % Get the transform of every joint (i.e. start and end of every link)
                tr = zeros(4,4,7); % 6 links + 1
                tr(:,:,1) = robot.base;
                L = robot.links;
                for i = 1 : 6
                    tr(:,:,i+1) = tr(:,:,i) * trotz(qMatrix(i)+L(i).offset) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
                end

                % Go through each link and also each triangle face
                for i = 1 : size(tr,3)-1
                    for faceIndex = 1:size(faces,1)
                        vertOnPlane = vertex(faces(faceIndex,1)',:);
                        [intersectP,check] = LinePlaneIntersection(faceNormals(faceIndex,:),vertOnPlane,tr(1:3,4,i)',tr(1:3,4,i+1)');
                        if check == 1 && IsIntersectionPointInsideTriangle(intersectP,vertex(faces(faceIndex,:)',:))
                            disp('Intersection');
                            result = true;
                            if returnOnceFound
                                return
                            end
                        end
                    end
                end
            end
        end
    end
end
