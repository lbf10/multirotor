function [Mt] = torqueMatrix(multirotor,rotationDirection)
%TORQUEMATRIX Returns matrix of torque direction and norm per rotor
%   [Mt] = torqueMatrix(multirotor,rotationDirection) returns a matrix composed
%   of the torques-per-squared-speed orientations for each rotor in multirotor
%   multicopter. Mt is a matrix composed of column vectors.
%   rotationDirection is a vector specifying the direction of rotation for
%   each rotor. +1 is counter-clockwise and -1 is clockwise, in relation to
%   the rotor orientation specified in multirotor. rotationDirection must have
%   length equal to the number of rotors in multirotor.

Mt = [];
for i=1:multirotor.numberOfRotors()
    Mt = [Mt (multirotor.rotorLiftCoeff(i)*cross(multirotor.rotorPosition(i),multirotor.rotorOrientation(i))-multirotor.rotorDragCoeff(i)*rotationDirection(i)*multirotor.rotorOrientation(i))];
end
end

