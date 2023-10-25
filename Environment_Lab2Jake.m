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

    %% Mesh 1 
    Y = ones(8,10)
    Y = Y*-0.45
    z = 0:(1/12):0.75
    x = -2:0.1:-1.3
    [Z,X] = meshgrid(z,x)
    oneSideOfCube_h = surf(X,Y,Z);
    cubePoints = [X(:),Y(:),Z(:)];

    cubeAtOigin_h = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3));

    %% Mesh 2
    Y2 = ones(8,10)
    Y2 = Y2*0.1
    z2 = 0:(1/12):0.75
    x2 = -2:0.1:-1.3
    [Z2,X2] = meshgrid(z2,x2)
    twoSideOfCube_h = surf(X2,Y2,Z2);
    cubePoints2 = [X2(:),Y2(:),Z2(:)];

    cubeAtOigin_h2 = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3));

    %% Mesh 3
    Z3 = ones(10,10)
    Z3 = Z3*0.45
    y3 = -0.45:(11/180):0.1
    x3 = -2:(7/90):-1.3
    [Y3,X3] = meshgrid(y3,x3)
    threeSideOfCube_h = surf(X3,Y3,Z3);
    cubePoints3 = [X3(:),Y3(:),Z3(:)];

    cubeAtOigin_h3 = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3));

    %% Mesh 4
    Z4 = ones(10,10)
    Z4 = Z4*0.75
    y4 = -0.45:(11/180):0.1
    x4 = -2:(7/90):-1.3
    [Y4,X4] = meshgrid(y4,x4)
    fourSideOfCube_h = surf(X4,Y4,Z4);
    cubePoints4 = [X4(:),Y4(:),Z4(:)];

    cubeAtOigin_h4 = plot3(cubePoints(:,1),cubePoints(:,2),cubePoints(:,3));


end

