close all; clear all; clc;

% Settings
FRICTION_CONE_APPROX = 32;

%DO NOT CHANGE: Defines Input/Output Folders
%  ./<SUBFOLDER>/filename.csv
FILEPATH_INPUT="./InputScript/InputContactNormals.csv";
addpath(genpath('InputScript')); %   <INPUT_Subfolder>
addpath(genpath('OutputPlots')); %   <OUTPUT_Subfolder>

% Reading input CSV file for testings
[C1,C2,N1,N2,mu]=readCSVInput(FILEPATH_INPUT);

% Get number of testcases
testcaseNumber = height(C1);
fprintf("Testcase number: %d\n", testcaseNumber);

% Basic setup
angle_bt_cone_boundaries = (2*pi)/FRICTION_CONE_APPROX;
fprintf("FRICTION_CONE_APPROX: %d --> Degrees between 2 neighboring friction cone boundaries: %d\n",FRICTION_CONE_APPROX, rad2deg(angle_bt_cone_boundaries));


% Process every testcases:
for i = 1:testcaseNumber
%for i = 1:1
    fprintf("[TESTCASE %d]\n", i);
    fprintf("contact point 1 (C1): (%.4f,%.4f,%.4f)\n", C1(i,1), C1(i,2), C1(i,3));
    fprintf("contact point 2 (C2): (%.4f,%.4f,%.4f)\n", C2(i,1), C2(i,2), C2(i,3));
    fprintf("contact point 1 (N1): (%.4f,%.4f,%.4f)\n", N1(i,1), N1(i,2), N1(i,3));
    fprintf("contact point 2 (N2): (%.4f,%.4f,%.4f)\n", N2(i,1), N2(i,2), N2(i,3));
    fprintf("Friction coefficient: %f\n", mu(i,1));
    
    % Build friction cones boundary vectors
    %   1. use surface normal to get 1st surface tangent
    %   2. use friction coefficient to get the 1st friction cone vertex.
    %   3. for N-vertices friction cone approximation, find other N-1
    %   cone vertices (by rotating 1st cone vertex around seuface normal)
    friction = mu(i, 1);
    normal_1 = normalize(N1(i, :));
    random_unit_vec = normalize(getRandomVector());
    tangent_1 = normalize(cross(normal_1, random_unit_vec));
    fcone_1_first_boundary = normalize(1 * normal_1 + friction * tangent_1);
    fcone_1 = zeros(FRICTION_CONE_APPROX,3);
    fcone_1(1,:) = fcone_1_first_boundary;
    for j = 2:FRICTION_CONE_APPROX
        boundary = rotateAroundAxis(fcone_1_first_boundary, normal_1, angle_bt_cone_boundaries*(j-1));
        boundary = normalize(boundary);
        fcone_1(j,:) = boundary;
    end

    normal_2 = N2(i, :);
    random_unit_vec = normalize(getRandomVector());
    tangent_2 = normalize(cross(normal_2, random_unit_vec));
    fcone_2_first_boundary = normalize(1 * normal_2 + friction * tangent_2);
    fcone_2 = zeros(FRICTION_CONE_APPROX,3);
    fcone_2(1,:) = fcone_2_first_boundary;
    for j = 2:FRICTION_CONE_APPROX
        boundary = rotateAroundAxis(fcone_2_first_boundary, normal_2, angle_bt_cone_boundaries*(j-1));
        boundary = normalize(boundary);
        fcone_2(j,:) = boundary;
    end

    % Visualize friction cones and surface normal
    % Plots will also be exported: OutputPlots/FrictionCone{testcase no.}.png
    grid on
    hold on
    for j = 1:length(fcone_1)
        xc = [C1(i, 1), C1(i,1)+fcone_1(j,1)];
        yc = [C1(i, 2), C1(i,2)+fcone_1(j,2)];
        zc = [C1(i, 3), C1(i,3)+fcone_1(j,3)];
        plot3(xc, yc, zc, "-o", "Color","#0072BD"); % for cone
    end
    xn = [C1(i, 1), C1(i, 1)+normal_1(1)];
    yn = [C1(i, 2), C1(i, 2)+normal_1(2)];
    zn = [C1(i, 3), C1(i, 3)+normal_1(3)];
    plot3(xn, yn, zn, "-o", "Color","#EDB120"); % for normal
    for j = 1:length(fcone_2)
        xc = [C2(i, 1), C2(i,1)+fcone_2(j,1)];
        yc = [C2(i, 2), C2(i,2)+fcone_2(j,2)];
        zc = [C2(i, 3), C2(i,3)+fcone_2(j,3)];
        plot3(xc, yc, zc, "-o", "Color","#A2142F"); % for cone
    end
    xn = [C2(i, 1), C2(i, 1)+normal_2(1)];
    yn = [C2(i, 2), C2(i, 2)+normal_2(2)];
    zn = [C2(i, 3), C2(i, 3)+normal_2(3)];
    plot3(xn, yn, zn, "-o", "Color","#EDB120"); % for normal
    view(45,60);
    print(gcf,'-dpng',[['./OutputPlots/FrictionCone' num2str(i)] '.png']);
    hold off

    % Construct grasp wrenches from boundary vectors of each friction cones
    % wrench = [F, F x D], where D is position w.r.t object center of mess.
    % In this project, center of mess is assigned to origin of world frame.
    forces = zeros(FRICTION_CONE_APPROX*2, 3);
    torques = zeros(FRICTION_CONE_APPROX*2, 3);
    for j = 1:FRICTION_CONE_APPROX
        F = fcone_1(j,:); % force 
        D = C1(i,:);
        T = cross(F, D); % torques
        forces(j,:) = F;
        torques(j,:) = T;
    end
    for j = 1:FRICTION_CONE_APPROX
        F = fcone_2(j,:); % force 
        D = C2(i,:);
        T = cross(F, D); % torques
        forces(j+FRICTION_CONE_APPROX,:) = F;
        torques(j+FRICTION_CONE_APPROX,:) = T;
    end


    % To avoid de-generate condition, we add a small epsilon to 0 col/row
    for row = 1:FRICTION_CONE_APPROX
        if forces(row,:)*0 == forces(row,:)
            forces(row,:) = -0.01 + (0.01+0.01)*rand(1,3);
        end
        if torques(row,:)*0 == torques(row,:)
            torques(row,:) = -0.01 + (0.01+0.01)*rand(1,3);
        end
    end
    for col = 1:3
        if forces(:,col)*0 == forces(:,col)
            forces(:,col) = -0.01 + (0.01+0.01)*rand(FRICTION_CONE_APPROX*2,1);
        end
        if torques(:,col)*0 == torques(:,col)
            torques(:,col) = -0.01 + (0.01+0.01)*rand(FRICTION_CONE_APPROX*2,1);
        end
    end
    grasp_wrenches = [forces torques]; % grasp wrench space: N x 6 matrix

    % Now we have 2 contact * N vertex-per-friction-cone = 2*N wrenches. 
    % We can construct a convex hull using these 2*N points.
    % To determine if force-closure is satisfied,
    % we can simply see if the origin is located inside the hull.
    origin = [0 0 0 0 0 0;];
    tess = convhulln(grasp_wrenches, {'QJ'});
    res = inhull(origin, grasp_wrenches, tess);
    closure = res(1);
    if closure
        fprintf("Testcase %d force-closure: YES.\n", i);
    else
        fprintf("Testcase %d force-closure: NO.\n", i);
    end
    %f_tess = convhulln(forces, {'QJ'});
    %f_res = inhull(origin, forces, f_tess);
    %F_closure = f_res(1);
    %t_tess = convhulln(torques, {'QJ'});
    %t_res = inhull(origin, torques, t_tess);
    %T_closure = t_res(1);
    %force_closure = F_closure * T_closure;
    %if force_closure
    %    fprintf("Testcase %d force-closure: YES.\n", i);
    %else
    %    fprintf("Testcase %d force-closure: NO.\n", i);
    %end

    grid on
    hold off
    % Visualize the convex hull for forces and torques
    % force:XY
    [k,av] = convhull(forces(:,[1,2]));
    plot(forces(:,1),forces(:,2),'*');
    hold on
    grid on
    plot(forces(k,1),forces(k,2));
    print(gcf,'-dpng',[['./OutputPlots/Force_XY_' num2str(i)] '.png']);
    hold off
    % force:XZ
    [k,av] = convhull(forces(:,[1,3]));
    plot(forces(:,1),forces(:,3),'*');
    hold on
    grid on
    plot(forces(k,1),forces(k,3));
    print(gcf,'-dpng',[['./OutputPlots/Force_XZ_' num2str(i)] '.png']);
    hold off
    % force:YZ
    [k,av] = convhull(forces(:,[2,3]));
    plot(forces(:,2),forces(:,3),'*');
    hold on
    grid on
    plot(forces(k,2),forces(k,3));
    print(gcf,'-dpng',[['./OutputPlots/Force_YZ_' num2str(i)] '.png']);
    hold off
    % torque:XY
    [k,av] = convhull(torques(:,[1,2]));
    plot(torques(:,1),torques(:,2),'*');
    hold on
    grid on
    plot(torques(k,1),torques(k,2));
    print(gcf,'-dpng',[['./OutputPlots/Torque_XY_' num2str(i)] '.png']);
    hold off
    % force:XZ
    [k,av] = convhull(torques(:,[1,3]));
    plot(torques(:,1),torques(:,3),'*');
    hold on
    grid on
    plot(torques(k,1),torques(k,3));
    print(gcf,'-dpng',[['./OutputPlots/Torque_XZ_' num2str(i)] '.png']);
    hold off
    % force:YZ
    [k,av] = convhull(torques(:,[2,3]));
    plot(torques(:,2),torques(:,3),'*');
    hold on
    grid on
    plot(torques(k,2),torques(k,3));
    print(gcf,'-dpng',[['./OutputPlots/Torque_YZ_' num2str(i)] '.png']);
    hold off
    % fx = transpose(forces(:,1));
    % fy = transpose(forces(:,2));
    % fz = transpose(forces(:,3));
    % scatter3(fx,fy,fz,'filled');
    
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % view(0,90); % XY
    % print(gcf,'-dpng',[['./OutputPlots/Force_XY_' num2str(i)] '.png']);
    % view(0,0); % XZ
    % print(gcf,'-dpng',[['./OutputPlots/Force_XZ_' num2str(i)] '.png']);
    % view(90,0); % YZ
    % print(gcf,'-dpng',[['./OutputPlots/Force_YZ_' num2str(i)] '.png']);
    % tx = transpose(forces(:,1));
    % ty = transpose(forces(:,2));
    % tz = transpose(forces(:,3));
    % scatter3(tx,ty,tz,'filled');
    % grid on
    % xlabel('X');
    % ylabel('Y');
    % zlabel('Z');
    % view(0,90); % XY
    % print(gcf,'-dpng',[['./OutputPlots/Torque_XY_' num2str(i)] '.png']);
    % view(0,0); % XZ
    % print(gcf,'-dpng',[['./OutputPlots/Torque_XZ_' num2str(i)] '.png']);
    % view(90,0); % YZ
    % print(gcf,'-dpng',[['./OutputPlots/Torque_YZ_' num2str(i)] '.png']);
    % hold off
end



% HELPER FUNCTIONS ==============================================

% readCSVInput()
%   Purpose: Reads from CSV file, parses into contacts, normals, friction
%       Input: filepath
%       Outputs     C1/C2: contact point 1&2 object
%                   N1/N2: contact normal 1&2 object respectively
%                   mu: coeff. of friction for contacts/normals
% CSV input file Format
%   Rows = Experiment 1, 2, 3, etc..
%   COLS = C1[1:3],C2[4:6],N1[7:9],N2[10:12],MU[13]
function [C1_,C2_,N1_,N2_,mu_] = readCSVInput(FILEPATH_)
    CSV_IN=readmatrix(FILEPATH_);
    dim=3;
    C1_=CSV_IN(:,1:3);
    C2_=CSV_IN(:,1+dim:3+dim);
    N1_=CSV_IN(:,1+2*dim:3+2*dim);
    N2_=CSV_IN(:,1+3*dim:3+3*dim);
    mu_=CSV_IN(:,13);
end

% Normalize a 3d vector
function n = normalize(v)
    n = v/norm(v);
end

% Generate a random vector
function v = getRandomVector()
    v = [randn, randn, randn];
end

% Rotate a 3d vector around an axis about an angle
function v = rotateAroundAxis(vec, axis, angle)
    c = cos(angle);
    s = sin(angle);
    x = axis(1); y = axis(2); z = axis(3);
    R = [
        c+x*x*(1-c)     x*y*(1-c)-z*s       x*z*(1-c)+y*s;
        y*x*(1-c)+z*s   c+y*y*(1-c)         y*z*(1-c)-x*s;
        z*x*(1-c)-y*s   z*y*(1-c)+x*s       c+z*z*(1-c);
    ];
    v = R * transpose(vec);
    v = transpose(v);
end