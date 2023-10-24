%% setting surface for robot
function Environment_Lab2Angie()

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
    PlaceObject('fireExtinguisher.ply', [1.1, 1.7, 0]);
    PlaceObject('emergencyStopButton.ply', [0.3, -0.3, 0.7]);
    hold on;

    % inital placements of boxes
    brick1 = [-1.4, -0.2, 1.29]; % top shelf
    brick2 = [-1.4, 0.45, 1.29];
    brick3 = [ -1.4, 1.0, 1.29];
    
    brick4 = [-1.4, -0.2, 1.0];
    brick5 = [-1.4, 0.45, 1.0];
    brick6 = [-1.4, 1.0, 1.0];

    brick7 = [-1.35, -0.2, 0.47];
    brick8 = [-1.35, 0.45, 0.47];
    brick9 = [-1.35, 1.0, 0.47];

    brick10 = [-1.35, -0.2, 0.2];
    brick11 = [-1.35, 0.45, 0.2];
    brick12 = [-1.35, 1.0, 0.2];
    
    
    Brick_1 = PlaceObject('box.ply', brick1);
    Brick_2 = PlaceObject('box.ply', brick2);
    Brick_3 = PlaceObject('box.ply', brick3);
    Brick_4 = PlaceObject('box.ply', brick4);
    Brick_5 = PlaceObject('box.ply', brick5);
    Brick_6 = PlaceObject('box.ply', brick6); 
    Brick_7 = PlaceObject('box.ply', brick7); 
    Brick_8 = PlaceObject('box.ply', brick8);
    Brick_9 = PlaceObject('box.ply', brick9);
    Brick_10 = PlaceObject('box.ply', brick10);
    Brick_11 = PlaceObject('box.ply', brick11);
    Brick_12 = PlaceObject('box.ply', brick12);

end

