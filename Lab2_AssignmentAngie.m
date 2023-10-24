classdef Lab2_AssignmentAngie < handle
    
    properties (Access = private)
        lab2GuiApp  % Instance of Lab2_GUI
    end
    
    properties (Constant)
        estopStatus = 'Run'; % Initialize to 'Run'
    end
    
    properties
        robot1 % Instance of the first robot
        robot2 % Instance of the second robot
    end
    
    %% Constructor
    methods
        function self = Lab2_AssignmentAngie 
            clc
            clf
            % Create an instance of Lab2_GUI
            self.lab2GuiApp = Lab2_GUI();
            Environment_Lab2();
            self.initialiseRobots();
            input('Press enter to start');
            self.objectLocater();
        end
    end

    %% Public methods
    methods
        function startApp(self)
            % Start the Lab2_GUI app
            self.lab2GuiApp.UIFigure.Visible = 'on';
        end
    end
    
    %% E-stop
    methods (Static)
        % Method to toggle E-stop status
        function toggleEstop(estopStatus)
            % Toggle the E-stop status between 'Run' and 'Stop'
            if strcmp(estopStatus, 'Run')
                estopStatus = 'Run';
            else
                estopStatus = 'Stop';
            end
        end
    end
    
    %% Defined positions of Groceries
    methods
        function initialiseRobots(self)
            % Initialise robot instances
            self.robot1 = LinearUR10;
            self.robot2 = Robo;
        end

        function objectLocater(self)
            % This function defines positions and initiates robot movement

            % Pick item from cupboard
            pickUps1 = [-1.4, -0.2, 1.29; 
                        -1.4, -0.2, 1.0;
                        -1.4, 1.0, 1.0;
                        -1.35, -0.2, 0.47;
                        -1.35, 1.0, 0.47;
                        -1.35, 0.45, 0.2];
      
            % Give to the robot 2
            endPos1 = [-0.5, 1.5, 1.2;
                       -0.5, 1.5, 1.2;
                       -0.5, 1.5, 1.2;
                       -0.5, 1.5, 1.2;
                       -0.5, 1.5, 1.2;
                       -0.5, 1.5, 1.2];

            % Pick up from robot 1
            pickUps2 = [-0.5, 1.5, 0.8;
                        -0.5, 1.5, 0.8
                        -0.5, 1.5, 0.8
                        -0.5, 1.5, 0.8
                        -0.5, 1.5, 0.8
                        -0.5, 1.5, 0.8];
               
            % Give to user 
            endPos2 = [0.5, 1.0, 0.9;
                       0.5, 1.0, 0.9;
                       0.5, 1.0, 0.9;
                       0.5, 1.0, 0.9;
                       0.5, 1.0, 0.9;
                       0.5, 1.0, 0.9];
          
            
            parfor i = 1:size(endPos1, 1)
                endPosition = endPos1(i, :);
                hold = pickUps1(i, :);
                self.robot1Rotato(endPosition, hold, self.robot1);
            end
            
            parfor i = 1:size(endPos2, 1)
                endPosition2 = endPos2(i,:);
                hold2 = pickUps2(i,:);
                self.robot2Rotato(endPosition2, hold2, self.robot2);
            end

            input('Press enter to end operation');
        end

        %% Movement of robot 1
        function robot1Rotato(self, endPosition, hold, robot1)
            
            %% Check E-stop status before allowing movement
            if strcmp(self.estopStatus, 'Stop')
                disp('E-stop is activated. Robot will not move.');
                return; % Exit the function without moving
            end
            
           
            numSteps = 100;
            
            endTransform = transl(hold) * trotx(pi);
            elbowAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
            qEnd = self.robot1.model.ikcon(endTransform, elbowAngles);

            for loop = 1:1
                qCurrent = self.robot1.model.getpos();
                plotTraj = jtraj(qCurrent, qEnd, numSteps);

                for i = 1:min(size(plotTraj, 1), numSteps)
                    self.robot1.model.animate(plotTraj(i, :));

                end

                qCurrent = self.robot1.model.getpos();
                endTransform = transl(endPosition) * trotx(pi);
                elbowEndAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
                BrickEnd = self.robot1.model.ikcon(endTransform, elbowEndAngles);
                plotTrajEnd = jtraj(qCurrent, BrickEnd, numSteps);

                for i = 1:numSteps
                    currentTransform = self.robot1.model.fkine(plotTrajEnd(i, :)).T;
                    self.robot1.model.animate(plotTrajEnd(i, :));
                    pause(0.01);
                end
            end
        end

        function robot2Rotato(self, endPosition2, hold2, robot2)
           
            % This method moves the robot and picks up items
            % Check E-stop status before allowing movement
            %{
            if strcmp(self.estopStatus, 'Stop')
                disp('E-stop is activated. Robot will not move.');
                return; % Exit the function without moving
            end
            %}
            numSteps = 100;
            
            endTransform = transl(hold2) * trotx(pi);
            elbowAngles = deg2rad([0, -40, 50, -60, -90, 0]);
            qEnd = self.robot2.model.ikcon(endTransform, elbowAngles);

            for numBricks = 1:1
                qCurrent = self.robot2.model.getpos();
                plotTraj = jtraj(qCurrent, qEnd, numSteps);

                for i = 1:min(size(plotTraj, 1), numSteps)
                    self.robot2.model.animate(plotTraj(i, :));

                end

                qCurrent = self.robot2.model.getpos();
                endTransform = transl(endPosition2) * trotx(pi);
                elbowEndAngles = deg2rad([0, -40, 50, -60, -90, 0]);
                BrickEnd = self.robot2.model.ikcon(endTransform, elbowEndAngles);
                plotTrajEnd = jtraj(qCurrent, BrickEnd, numSteps);

                for i = 1:numSteps
                    currentTransform = self.robot2.model.fkine(plotTrajEnd(i, :)).T;
                    self.robot2.model.animate(plotTrajEnd(i, :));
                    pause(0.01);
                end
            end
        end 
    end
end


          