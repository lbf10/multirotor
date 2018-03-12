function [q] = matrixToQuaternion(M)
%MATRIXTOQUATERNION Calculates the quaternion associated with a rotation
%matrix
%   [q] = matrixToQuaternion(M) returns the quaternion q represented by the
%   rotation matrix M. M is the transformation matrix from inertial
%   reference frame to the body fixed frame represented by the attitude q.

    if M(2,2)>-M(3,3) && M(1,1)>-M(2,2) && M(1,1)>-M(3,3)
        q1 = sqrt(1+M(1,1)+M(2,2)+M(3,3));
        q = 0.5*[q1;
                 (M(2,3)-M(3,2))/q1;
                 (M(3,1)-M(1,3))/q1;
                 (M(1,2)-M(2,1))/q1];
    elseif M(2,2)<-M(3,3) && M(1,1)>M(2,2) && M(1,1)>M(3,3)
        q2 = sqrt(1+M(1,1)-M(2,2)-M(3,3));
        q = 0.5*[(M(2,3)-M(3,2))/q2;
                  q2;
                 (M(1,2)+M(2,1))/q2;
                 (M(3,1)+M(1,3))/q2];
    elseif M(2,2)>M(3,3) && M(1,1)<M(2,2) && M(1,1)<-M(3,3)
        q3 = sqrt(1-M(1,1)+M(2,2)-M(3,3));
        q = 0.5*[(M(3,1)-M(1,3))/q3;
                 (M(1,2)+M(2,1))/q3;
                  q3;
                 (M(2,3)+M(3,2))/q3];
    elseif M(2,2)<M(3,3) && M(1,1)<-M(2,2) && M(1,1)<M(3,3)
        q4 = sqrt(1-M(1,1)-M(2,2)+M(3,3));
        q = 0.5*[(M(1,2)-M(2,1))/q4;
                 (M(3,1)+M(1,3))/q4;
                 (M(2,3)+M(3,2))/q4;
                  q4];
    else
        error('No conversion from rotation matrix to quaternion existent.')
    end

end

