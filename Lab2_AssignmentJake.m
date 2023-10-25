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

          endPos = [-1.5, -0.5, 1.8;

                    -1.5, 0.5, 1.8

                    -1.5, 1.0, 1.8;

                    -1.5, -0.5, 1.0;

                    -1.5, 0.5, 1.0

                    -1.5, 1.0, 1.0

                    -1.5, -0.5, 0.5;

                    -1.5, 0.5, 0.3

                    -1.5, 1.0, 0.5];

 

           pickUps = [-0.5, 1.5, 0.7;

                      -0.5, 1.5, 0.7;

                      -0.5, 1.5, 0.7;

                      -0.5, 1.5, 0.7;

                      -0.5, 1.5, 0.7;

                      -0.5, 1.5, 0.7

                      -0.5, 1.5, 0.7

                      -0.5, 1.5, 0.7

                      -0.5, 1.5, 0.7

                      ];

           for i = 1:size(endPos, 1)
               endPosition = endPos(i, :);
               hold = pickUps(i, :);
               Lab2_Assignment.beansCollider(endPosition, hold, robot1);
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
                  %{
                  %Claw
                  Claw1 = TheClaww;6    
                  endEffectorTransform = robot1.model.fkine(plotTraj(i, :)).T;
                  Claw1.model.base = endEffectorTransform;
                  %Claw1.model.animate([0, 0]);
                  position = plotTraj(i,:);
                  endEffector = robot1.model.fkine(position).T;
                  Claw1.model.base = endEffector * trotx(pi/2);
                  %Claw1.model.animate([0, 0]);
                  %}
                  pause(0.01);
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

      function beansCollider(endPosition, hold, robot1)
          q1 = robot1.model.ikine(transl(hold)*trotx(pi))
          %q1 = endPosition
          q2 = robot1.model.ikine(transl(endPosition)*trotx(pi))
       robot1.model.animate(q1);
       qWaypoints = [q1;q2];
       isCollision = true;
       checkedTillWaypoint = 1;
       qMatrix = [];
       while (isCollision)
           startWaypoint = checkedTillWaypoint;
           for i = startWaypoint:size(qWaypoints,1)-1
               qMatrixJoin = InterpolateWaypointRadians(qWaypoints(i:i+1,:),deg2rad(10));
               if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                   qMatrix = [qMatrix; qMatrixJoin]; %#ok<AGROW>
                   robot.animate(qMatrixJoin);
                   size(qMatrix)
                   isCollision = false;
                   checkedTillWaypoint = i+1;
                   % Now try and join to the final goal (q2)
                   qMatrixJoin = InterpolateWaypointRadians([qMatrix(end,:); q2],deg2rad(10));
               if ~IsCollision(robot,qMatrixJoin,faces,vertex,faceNormals)
                   qMatrix = [qMatrix;qMatrixJoin];
                   % Reached goal without collision, so break out
                   break;
               end
               else
                   % Randomly pick a pose that is not in collision
                   qRand = (2 * rand(1,3) - 1) * pi;
                   while IsCollision(robot,qRand,faces,vertex,faceNormals)
                       qRand = (2 * rand(1,3) - 1) * pi;
                   end
                   qWaypoints =[ qWaypoints(1:i,:); qRand; qWaypoints(i+1:end,:)];
                   isCollision = true;
                   break;
               end
          end
      end
      robot.animate(qMatrix)
      keyboard
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


          