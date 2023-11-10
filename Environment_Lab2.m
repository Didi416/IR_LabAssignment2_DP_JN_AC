%% setting surface for robot
function [vertices, faces, faceNormals] = Environment_Lab2()

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
    kitchenMesh = PlaceObject('kitchen2.ply', [-2.8, 5, 0.01]);
 
    % safety precautions
    PlaceObject('fireExtinguisher.ply', [1.1, 1.7, 0]);
    PlaceObject('emergencyStopButton.ply', [0.3, -0.3, 0.7]);
    hold on;

    vertices = get(kitchenMesh, 'Vertices');
    faces = get(kitchenMesh, 'Faces');
    faceNormals = zeros(size(faces,1),3);
    for faceIndex = 1:size(faces,1)
        v1 = vertices(faces(faceIndex,1)',:);
        v2 = vertices(faces(faceIndex,2)',:);
        v3 = vertices(faces(faceIndex,3)',:);
        faceNormals(faceIndex,:) = unit(cross(v2-v1,v3-v1));
    end
end
