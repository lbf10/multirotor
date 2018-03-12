function [Mf] = forceMatrix(multirotor)
%FORCEMATRIX Returns matrix of rotor orientations multiplied by lift
%coeffs.
%   [Mf] = forceMatrix(multirotor) returns a matrix composed of all the rotor
%   orientations of multirotor multicopter multiplied by its respective lift
%   coefficients. Mf is a matrix composed of column vectors.

Mf = [];
for i=1:multirotor.numberOfRotors()
    Mf = [Mf multirotor.rotorLiftCoeff(i)*multirotor.rotorOrientation(i)];
end
end

