%
% Nicholas Kapsanis, z5254990
%

% note tolerance is input in degrees. and the normal vector has no assumptions about directionality.

function ThisOne = InferWhichWall(NormalVectorOfFlatPatch, GuessOfCurrentAttitude, Tolerance)
    %basically, we get a vector, and we want to check if its within tolerance of any of the normal vectors to the planes of the global coordinate frame
    % we have the current attitude of the platform in GCF, and the Normal vector of the flat patch in the platform's coordinate frame
    [x, y, z] = RotatePoints(NormalVectorOfFlatPatch(1), NormalVectorOfFlatPatch(2), NormalVectorOfFlatPatch(3), GuessOfCurrentAttitude(1), GuessOfCurrentAttitude(2), GuessOfCurrentAttitude(3));
    RotatedNormalVector = [x; y; z]./norm([x; y; z]); % Normalize the rotated normal vector

    % Define the known normal vectors in the global coordinate frame
    FloorNormal = [0; 0; 1];       % Floor (XY plane)
    Wall1Normal = [1; 0; 0];      % Wall1 (YZ plane)
    Wall2Normal = [0; 1; 0];       % Wall2 (XZ plane)
    Wall3Normal = [0; -1; 0];      % Wall3 (XZ plane)

    % Convert tolerance from degrees to radians
    ToleranceRad = deg2rad(Tolerance);

    % Check the angle between the rotated normal vector and each known normal vector
    if IsWithinTolerance(RotatedNormalVector, FloorNormal, ToleranceRad)
        ThisOne = 1; % Floor
    elseif IsWithinTolerance(RotatedNormalVector, Wall1Normal, ToleranceRad) || IsWithinTolerance(RotatedNormalVector, -Wall1Normal, ToleranceRad)
        ThisOne = 2; % Wall1
    elseif IsWithinTolerance(RotatedNormalVector, Wall2Normal, ToleranceRad)
        ThisOne = 3; % Wall2
    elseif IsWithinTolerance(RotatedNormalVector, Wall3Normal, ToleranceRad)
        ThisOne = 4; % Wall3
    else
        ThisOne = 0; % No successful association
    end
end

function isWithin = IsWithinTolerance(Vector1, Vector2, ToleranceRad)
    % Normalize the vectors
    Vector1 = Vector1 / norm(Vector1);
    Vector2 = Vector2 / norm(Vector2);

    % Calculate the angle between the vectors using the dot product
    CosTheta = dot(Vector1, Vector2);
    Angle = acos(CosTheta);

    % Check if the angle is within the tolerance
    isWithin = Angle <= ToleranceRad;
end

function [xx, yy, zz] = RotatePoints(xx, yy, zz, roll, pitch, yaw)
    % Rotation matrices
    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)]; % Roll
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)]; % Pitch
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1]; % Yaw

    % Combined rotation matrix
    R = Rz * Ry * Rx;

    % Apply rotation
    rotatedPoints = R * [xx(:)'; yy(:)'; zz(:)'];
    xx = rotatedPoints(1, :)';
    yy = rotatedPoints(2, :)';
    zz = rotatedPoints(3, :)';
end