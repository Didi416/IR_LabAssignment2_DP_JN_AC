%% setting surface for robot
function Environment_Lab2()

    surf([-2,-2; 2,2], [-2,2;-2,2], [0.0,0.0;0.0,0.0], ...
        'CData', imread('floor.jpg'), 'FaceColor', 'texturemap');
    hold on;
    %camlight
    
    surf([-2,-2;-2,-2], [-2,-2;2,2], [0.01,2.2;0.01,2.2], ...
        'CData', imrotate(imread('window1.jpg'), -90),'FaceColor','texturemap');
    hold on;
    %camlight
    
    surf([-2,-2;2,2],[2,2;2,2],[0.01,2.2;0.01,2.2], ...
        'CData', imrotate(imread('window2.jpg'), -90),'FaceColor','texturemap');
    hold on;
    %camlight
    
    axis ([-2 2 -2 2 0 2.2])
    grid on
    xlabel ('X');
    ylabel ('Y');
    zlabel ('Z');
    axis equal;
    
    % setting
    PlaceObject('kitchen2.ply', [-2.8, 5, 0.01]);
 
    % safety precautions
    %PlaceObject('barrier.ply', [-2.8, 1.65, 0.15]);
  
  
    PlaceObject('fireExtinguisher.ply', [1.1, 1.7, 0]);
    PlaceObject('emergencyStopButton.ply', [0.3, 1.65, 0.87]);
    hold on;

    %% 3D mesh of environemnt 
%     %Load the PLY file
%     ptCloud = pcread('kitchen.ply');
% 
%     figure
%     pcshow(ptCloud)
%     xlabel('X')
%     ylabel('Y')
%     zlabel('Z')
% 
%     % Extract the 'Location' property
%     xyzPoints = ptCloud.Location;
%     
%     % Calculate normals
%     normals = pcnormals(ptCloud);
% 
%     % Create a point cloud with normals
%     ptCloudWithNormals = pointCloud([xyzPoints, normals]);
%     
%     % Visualize the mesh
%     pcshow(ptCloudWithNormals);
% % 
% % 
% %     ptCloud = pcread('teapot.ply');
% % 
% %     %% 
% %     % 1. Load the PLY file
% %     filename = 'kitchen.ply'; % Replace with your file name
% %     ptCloud = pcread(filename);
% %     
% %     % Extract the XYZ locations of the point cloud
% %     points = ptCloud.Location;
% %     
% %     % 2. Reconstruct the surface using alphaShape
% %     alphaValue = 1.5; % This is a parameter that you might need to tweak. Adjust this value based on your specific dataset for best results.
% %     shp = alphaShape(points(:,1), points(:,2), points(:,3), alphaValue);
% %     
% %     % 3. Display the reconstructed shape
% %     figure;
% %     plot(shp);
% %     title('Reconstructed Mesh from PLY Points');
% %     axis equal;
% %     xlabel('X-axis');
% %     ylabel('Y-axis');
% %     zlabel('Z-axis');
% 


end

