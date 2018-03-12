% Calculate A and B reference for adaptive controller
torqueAux = zeros(3,1);
for it=1:multirotor.numberOfRotors()
    torqueAux = torqueAux + multirotor.rotorOrientation(it)*(900*multirotor.rotorInertia(it));
end
auxA = -torqueAux;
auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
D = multirotor.inertia()\auxA;

A = [-eye(3)*0.001, zeros(3,3);zeros(3,3),-eye(3)]
B = [eye(3)/multirotor.mass();zeros(3,3)]