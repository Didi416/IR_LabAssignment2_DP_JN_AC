classdef Lab2_Assignment < handle

   methods
       function self = Lab2_Assignment()
           clf
           clc
           Environment_Lab2();
           self.objectLocater();
       end
   end

   %% Defined positions of Groceries
   methods (Static)
       function objectLocater()
          robot1 = LinearUR10;
          q = [0, 0, 0, 0, 0, 0, 0];

          endPos = [-0.9, 0.4, 1.8;
                    -0.9, 0.4, 1.8
                     0.9, 0.4, 1.8;
                    -0.6, 0.4, 1.8;
                    -0.6, 0.4, 2];

           pickUps = [-0.9, -0.4, 0.7;
                      -0.6, -0.4, 0.7
                      -0.3, -0.4, 0.7;
                      -0.1, -0.4, 0.7;
                      -1.2, -0.2, 0.7];

           for i = 1:size(endPos, 1)
               endPosition = endPos(i, :);
               hold = pickUps(i, :);
               Lab2_Assignment.robotRotato(endPosition, hold, robot1);
           end
           input('Press enter to end operation');
       end

       function robotRotato(endPosition, hold, robot1)
          numSteps = 100;

          endTransform = transl(hold) * trotx(pi);
          elbowAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
          qEnd = robot1.model.ikcon(endTransform, elbowAngles);

          for numBricks = 1:1
              qCurrent = robot1.model.getpos();
              plotTraj = jtraj(qCurrent, qEnd, numSteps);

              for i = 1:min(size(plotTraj, 1), numSteps)
                  robot1.model.animate(plotTraj(i, :));

                  %% Claw 
                  %{
                  Claw1 = TheClaww;  
                  endEffectorTransform = robot1.model.fkine(plotTraj(i, :)).T;
                  Claw1.model.base = endEffectorTransform;
                  %Claw1.model.animate([0, 0]);
                  position = plotTraj(i,:);
                  endEffector = robot1.model.fkine(position).T;
                  Claw1.model.base = endEffector * trotx(pi/2);
                  %Claw1.model.animate([0, 0]);
                  pause(0.01);
                  %}
              end

              qCurrent = robot1.model.getpos();
              endTransform = transl(endPosition) * trotx(pi);
              elbowEndAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
              BrickEnd = robot1.model.ikcon(endTransform, elbowEndAngles);
              plotTrajEnd = jtraj(qCurrent, BrickEnd, numSteps);

              for i = 1:numSteps
                  currentTransform = robot1.model.fkine(plotTrajEnd(i, :)).T;
                  robot1.model.animate(plotTrajEnd(i, :));
                  pause(0.01);
              end
          end
       end
       
        %% Collision detection
        function environmentObjects = loadEnvironmentObjects()
            % Define or load information about environment objects in this function
            environmentObjects = load(Environment_Lab2);
            % This can include defining object shapes and positions
            % For example, you can create a cell array or a struct to represent the objects
            % Example: environmentObjects = {object1, object2, object3, ...};
        end

        function collisionDetected = IsCollisionWithEnvironment(currentTransform, environmentObjects)
               % Check for collisions between robot and environment objects
               % Implement your collision detection logic here
               collisionDetected = false; % Update this based on collision checking
               if collisionDetected == 1
               display('Collision Detected')
                if returnOnceFound
                    return
                end
               end
        end
   
   end 
        
end
   


          