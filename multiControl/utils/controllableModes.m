function faultMatrix = controllableModes(positions,orientations,directions)
liftCoeff = 6.97e-5;
dragCoeff = 1.033e-6;
Mt = [];
Mf = [];
for it=1:length(directions)
    Mt = [Mt liftCoeff*cross(positions(:,it),orientations(:,it))-dragCoeff*directions(it)*orientations(:,it)];
    Mf = [Mf liftCoeff*orientations(:,it)];
end

controllability = [Mt;Mf];

vect_len = 8;
max_val = 2^vect_len - 1 ;
values  = 0 : max_val;  values  = values';
faultCases = de2bi(values, vect_len, 'left-msb');

ranks = zeros(2^vect_len,1);
faultMatrix = [];
for it=1:length(faultCases)
    if rank(controllability*diag(faultCases(it,:)))>=4
        faultMatrix = [faultMatrix; faultCases(it,:)];
    end
end
end

