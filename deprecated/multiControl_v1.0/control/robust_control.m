function [L,K,P] = robust_control(P,R,Q,F,G,Ef,Eg)


[nx,nu]  = size(G); 
 nr      = size(Eg,1);

I_r = [eye(nx,nx); zeros(nr,nx)];

G_r = [G; Eg];
F_r = [F; Ef];

X  = [ P^(-1)          zeros(nx,nu)    zeros(nx,nx)    zeros(nx,nr+nx)    eye(nx,nx)   zeros(nx,nu)
       zeros(nu,nx)    R^(-1)          zeros(nu,nx)    zeros(nu,nr+nx)    zeros(nu,nx) eye(nu,nu)
       zeros(nx,nx)    zeros(nx,nu)    Q^(-1)          zeros(nx,nr+nx)    zeros(nx,nx) zeros(nx,nu)
       zeros(nr+nx,nx) zeros(nr+nx,nu) zeros(nr+nx,nx) zeros(nr+nx,nr+nx) I_r          -G_r
       eye(nx,nx)      zeros(nx,nu)    zeros(nx,nx)    I_r'               zeros(nx,nx) zeros(nx,nu)
       zeros(nu,nx)    eye(nu,nu)      zeros(nu,nx)   -G_r'               zeros(nu,nx) zeros(nu,nu)];

   
Zeta = [zeros(nx,nx)    zeros(nx,nu)     zeros(nx,nx);
        zeros(nu,nx)    zeros(nu,nu)     zeros(nu,nx);
        zeros(nx,nx)    zeros(nx,nu)    -eye(nx,nx);
        zeros(nx+nr,nx) zeros(nx+nr,nu)  F_r;
        eye(nx,nx)      zeros(nx,nu)     zeros(nx,nx);
        zeros(nu,nx)    eye(nu,nu)       zeros(nu,nx)];

Xi = [zeros(nx,nx); zeros(nu,nx); -eye(nx,nx); F_r; zeros(nx,nx); zeros(nu,nx)];
       
       
LKP = Zeta'*inv(X)*Xi;

L = LKP(1:nx,:);
K = LKP(nx+1:nx+nu,:);
P = LKP(nx+nu+1:end,:);




