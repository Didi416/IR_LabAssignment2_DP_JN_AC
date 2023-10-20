classdef Lab2_Assignment < handle
    properties
        r
    end
   methods
       function self = Lab2_Assignment()
           clf
           clc
           Environment_Lab2();
           self.clawAttach(-1,0,0);
       end
   end

   %% Defined positions of Groceries
   methods
       function self = clawAttach(self, X, Y, Z)
           baseTr = transl(X, Y, Z);
           self.r = LinearUR10(baseTr);
           hold on;
           Claw1 = TheClaww();
           Claw2 = TheClaww();

           q = [0,0,0,0,0,0,0];
           Claw1.model.base = self.r.model.fkine(q).T * transl(0,0,0.09);
           Claw1.model.animate(q);
           drawnow;
           Claw2.model.base = self.r.model.fkine(q).T * trotz(pi) * transl(0,0,0.09);
           Claw2.model.animate(q);
           drawnow;
           input("Press ENTER to finish.")
       end
   end
   methods (Static)
       function objectLocater()
          self.r = LinearUR10;
          q = [0, 0, 0, 0, 0, 0, 0];

          endPos = [-0.9, 0.4, 0.50;
                    -0.9, 0.4, 0.55
                     0.9, 0.4, 0.60;
                    -0.6, 0.4, 0.50;
                    -0.6, 0.4, 0.55];

           pickUps = [-0.9, -0.4, 0.7;
                      -0.6, -0.4, 0.7
                      -0.3, -0.4, 0.7;
                      -0.1, -0.4, 0.7;
                      -1.2, -0.2, 0.7];

           for i = 1:size(endPos, 1)
               endPosition = endPos(i, :);
               hold = pickUps(i, :);
               Lab2_Assignment.robotRotato(endPosition, hold, self.r);
           end
           input('Press enter to end operation');
       end

       function robotRotato(endPosition, hold, r)
          numSteps = 100;

          endTransform = transl(hold) * trotx(pi);
          elbowAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
          qEnd = self.r.model.ikcon(endTransform, elbowAngles);

          for numBricks = 1:1
              qCurrent = self.r.model.getpos();
              plotTraj = jtraj(qCurrent, qEnd, numSteps);

              for i = 1:min(size(plotTraj, 1), numSteps)
                  self.r.model.animate(plotTraj(i, :));

                  %% Claw
                    
                  endEffectorTransform = self.r.model.fkine(plotTraj(i, :)).T;
                  Claw1.model.base = endEffectorTransform;
                  %Claw1.model.animate([0, 0]);
                  position = plotTraj(i,:);
                  endEffector = self.r.model.fkine(position).T;
                  Claw1.model.base = endEffector * trotx(pi/2);
                  %Claw1.model.animate([0, 0]);
                  pause(0.01);
              end

              qCurrent = self.r.model.getpos();
              endTransform = transl(endPosition) * trotx(pi);
              elbowEndAngles = deg2rad([0, 0, 45, 70, -35, 259, 0]);
              BrickEnd = self.r.model.ikcon(endTransform, elbowEndAngles);
              plotTrajEnd = jtraj(qCurrent, BrickEnd, numSteps);

              for i = 1:numSteps
                  currentTransform = self.r.model.fkine(plotTrajEnd(i, :)).T;
                  self.r.model.animate(plotTrajEnd(i, :));
                  pause(0.01);
              end
          end
       end
   end

       %{
       function collisionDetected = IsCollisionWithEnvironment(currentTransform, environmentObjects)
           % Check for collisions between robot and environment objects
           % Implement your collision detection logic here
           collisionDetected = false; % Update this based on collision checking
       end

       function environmentObjects = loadEnvironmentObjects()
        % Define or load information about environment objects in this function
        % This can include defining object shapes and positions
        % For example, you can create a cell array or a struct to represent the objects
        % Example: environmentObjects = {object1, object2, object3, ...};
        end
        
       %}
   
end
