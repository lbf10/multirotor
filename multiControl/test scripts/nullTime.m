% tic 
% z = null(Mt)
% toc
% 
% tic
% z = null(Mt,'r')
% toc
n = 8;
b = (750^2)*ones(n,1)/2;
dr = 100*[0.5;0;1];
R = eye(n);
Q = eye(3);
N = null(Mt)
% d1 = (z'*z)\z'*b
v = (N'*(R*N+Mf'*Q*Mf*N))\(N'*(R*b+Mf'*Q*dr));
brealizado = N*v
d = Mf*N*v;
% d = d/norm(d)
figure
plot3([0,d(1)],[0,d(2)],[0,d(3)],'b')
hold on
plot3([0,dr(1)],[0,dr(2)],[0,dr(3)],'r')
% daspect([1 1 1])

custo1 = norm(N*v-b)
custo2 = norm(Mf*N*v-dr)

% b_fail = b;
% % b_fail(1) = 0
% Mf_fail = Mf;
% Mf_fail(:,1) = 0
% Mt_fail = Mt;
% Mt_fail(:,1) = 0
% z_fail = null(Mt_fail)
% d1_fail = (z_fail'*z_fail)\z_fail'*b_fail
% d_fail = Mf_fail*z_fail*d1_fail
% d_fail = d_fail/norm(d_fail)
% 
% plot3([0,d_fail(1)],[0,d_fail(2)],[0,d_fail(3)],'r')
% 
% limits = max(abs(d),abs(d_fail));
% xlim(limits(1)*[-1 1])
% ylim(limits(2)*[-1 1])
% zlim(limits(3)*[0 1])
% grid on
% 
% 
