function [Q] = matrixQ(q)
    %MATRIXQ Transformation matrix from body frame represented by q to inertial
    %frame
    %   Q = MATRIXQ(q) returns the transformation matrix from body reference
    %   frame, which attitude is represented by q q, to inertial
    %   reference frame. q must be a vector of length 4 and norm = 1.
    %   The transformation from inertial frame to body frame can be achieved
    %   with inv(Q) = Q'.

    Q(1,1) = q(1)^2+q(2)^2-q(3)^2-q(4)^2;
    Q(1,2) = 2*(q(2)*q(3)-q(1)*q(4));
    Q(1,3) = 2*(q(1)*q(3)+q(2)*q(4));
    Q(2,1) = 2*(q(2)*q(3)+q(1)*q(4));
    Q(2,2) = q(1)^2-q(2)^2+q(3)^2-q(4)^2;
    Q(2,3) = 2*(q(3)*q(4)-q(1)*q(2));
    Q(3,1) = 2*(q(2)*q(4)-q(1)*q(3));
    Q(3,2) = 2*(q(3)*q(4)+q(1)*q(2));
    Q(3,3) = q(1)^2-q(2)^2-q(3)^2+q(4)^2;
end

