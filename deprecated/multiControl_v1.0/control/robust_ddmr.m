function [tau, P, Aerror] = robust_ddmr(T, qe, desiredAngularVelocity, desiredAngularAcceleration, multirotor, controller)
rotorIDs = 1:multirotor.numberOfRotors();
torqueAux = multirotor.rotorOrientation(rotorIDs)*(multirotor.previousRotorSpeed(rotorIDs).*multirotor.rotorInertia(rotorIDs))';
auxA = multirotor.inertia()*multirotor.previousAngularVelocity()-torqueAux;
auxA = [0 -auxA(3) auxA(2) ; auxA(3) 0 -auxA(1) ; -auxA(2) auxA(1) 0 ];
auxA = multirotor.inertia()\auxA;

auxA2 = 1.1*multirotor.inertia()*multirotor.previousAngularVelocity()-torqueAux;
auxA2 = [0 -auxA2(3) auxA2(2) ; auxA2(3) 0 -auxA2(1) ; -auxA2(2) auxA2(1) 0 ];
auxA2 = (1.1*multirotor.inertia())\auxA2;

Aerror = auxA2-auxA;

Sq = [qe(1),-qe(4),qe(3);
      qe(4),qe(1),-qe(2);
      -qe(3),qe(2),qe(1)];
  
x_e = [desiredAngularVelocity-multirotor.previousAngularVelocity();qe(2:4)'];

ssA = [auxA, zeros(3); 0.5*Sq, zeros(3)];
ssB = [eye(3);zeros(3)];
sys = ss(ssA,ssB,eye(6),0);
sysD = c2d(sys,T);

ssdA = sysD.A;
ssdB = sysD.B;
P = controller.RLQR.P;
R = controller.RLQR.R;
Q = controller.RLQR.Q;
Ef = controller.RLQR.Ef;
Eg = controller.RLQR.Eg;
niter = controller.RLQR.numberOfIteractions;

% normPbefore = 0;
% error = 100;
% cont_robust = 1;
% while (cont_robust < niter) && (error>0.01)
    [L,K,P] = robust_control(P,R,Q,ssdA,ssdB,Ef,Eg);
%     teste(cont_robust) = norm(P);
%     cont_robust = cont_robust + 1;
%     normP = norm(P);
%     error = abs((normP-normPbefore)/normP);
%     normPbefore = normP;
% end
% figure
% plot(teste)
% pause
% eig(ssdA +ssdB*K)
% eig(L)
% K = lqr(ssA,ssB,Q,R);

u = K*x_e;
tau = -multirotor.inertia()*(u-desiredAngularAcceleration+auxA*desiredAngularVelocity);
end