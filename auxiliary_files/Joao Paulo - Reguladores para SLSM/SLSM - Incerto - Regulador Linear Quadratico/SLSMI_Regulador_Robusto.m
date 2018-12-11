%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%           Regulador Linear Quadrático Robusto para SLSM INCERTO         %
%                            Dependente do Modo                           %
%                                                                         %
% Algoritmo do Artigo "Recursive Robust Regulator for Discrete-time MJLS  %
% via Penalty Game Approach" de J.P. Cerri, M.H. Terra and J.Y. Ishihara  %
% (2010).                                                                 %
%                                                                         %
% Dados de entrada -> arquivo:  Dados do sistema                          %
%                     escalar: N      - horizonte                         %
%                              parms  - parâmetro que caracteriza o valor %
%                                       de mi (exato/aproximado)          %
%                              mi     - parâmetro de penalidade           %
%                              alfa   - parâmetro de minimização          %
%                                                                         %
% Dados de saida -> matriz P: soluções das Equações de Riccati            %
%                   matriz K: ganhos de realimentação                     %
%                   matriz L: malhas fechadas                             %
%                                                                         %
%%=======================================================================%%
% Daiane C. Bortolin    -    daiane.bortolin@usp.br   -    Outubro / 2016 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [P,K,L] = SLSMI_Regulador_Robusto(N,parms,mi,alfa)

disp('*********************************************')
disp('        RLQ Robusto para SLSM Incerto        ')
disp('*********************************************')

%%%---------------------------------------------------

% Verifica os valores das variáveis de entrada

  if mi <= 0
     error('Valor de mi deve ser positivo')
 end

  if (parms ~= 0) && (parms ~= 1)
     error('parms = {0,1}')
 end 
 
%%%---------------------------------------------------  

% Variáveis globais
  global m n q s Prob F G Q R H Ef Eg P
  
%%%---------------------------------------------------

% Variaveis auxiliares
  In = eye(n);
  Im = eye(m);
  Iq = eye(q);

%%%=====================================================================%%%
%%%                         Loop do Algoritmo                           %%%
%%%=====================================================================%%%

for k = 1:N
        
    for i = 1:s
                       
        % Calculo do operador Psi
          OpPsi = zeros(n);          
          for j = 1:s  
              OpPsi = OpPsi + Prob(i,j) * P(:,:,k,j);
          end
         
        %%%---------------------------------------------------
        
        % Calculo do lambda
          lambda = (1 + alfa) * norm( mi * H(:,:,i)' * H(:,:,i) );
           
        %%%---------------------------------------------------
        
        P_cal = blkdiag(  OpPsi,R(:,:,i),Q(:,:,i) );
         
        A_cal = [ eye(n)      zeros(n,m);...
                  zeros(m,n)  Im        ; ...
                  zeros(n)    zeros(n,m); ...
                  In         -G(:,:,i)  ; ...
                  zeros(q,n) -Eg(:,:,i) ];
          
        %%%=============================================================%%%
        %%%           CONDIÇÃO PARA A EXISTÊNCIA DA SOLUÇÃO             %%%
        %%%=============================================================%%%
        
        %%%---------------------------------------------------
        %%% Condições sobre o posto   
        %%%---------------------------------------------------
        
        % % posto coluna pleno
        %  postoA = [ size(A_cal,2) rank(A_cal) ]
        %         
        % % posto linha pleno
        %   EA = [ In -G(:,:,i) ; zeros(q,n) -Eg(:,:,i) ];
        %   postoEA = [ size(EA,1) rank(EA) ]   
        %     
        % pause
        %%%----------------------------------------------------
        
        Aux1 = [ In -G(:,:,i) ; zeros(q,n) -Eg(:,:,i) ];
         if ( rank( Aux1 ) < size( Aux1,1 ) ) && parms ==0
            error('Problema sem solução para parms = 0')           
         end
        
        %%%---------------------------------------------------
        %%% Condições sobre Phi(mi,lambda)  
        %%%---------------------------------------------------
        
        phi = parms * ( mi^(-1) * In - lambda^(-1) * H(:,:,i) * H(:,:,i)' );
        %---------------------------
        % Matriz phi(mi,lambda) >= 0 
          if min( eig( phi ) ) < 0
             error('ErrorTests:convertTest', ...
              'A matriz Phi não é (semi)definida positiva \n Modifique as variáveis parms e/ou mi.')
          end
   
        %%%=============================================================%%%
        
        % Calculo da matriz Sigma
          MSigma =  blkdiag( phi,parms*lambda^(-1)*Iq );
                     
        %%%---------------------------------------------------
        
        Aux_W = blkdiag( inv(P_cal),MSigma );
        
        W = [ Aux_W  A_cal; A_cal' zeros(n+m) ];

        %%% Avalia o número de condição da matriz W   
        %%%---------------------------------------------------- 
        if rcond(W) <= eps
            error('ErrorTests:convertTest', ...
             'Matriz W é mal condicionada \n Modifique as variáveis parms e/ou mi.')
        end 
        
        %%%---------------------------------------------------- 
        
        Z = [ zeros(3*n+m+q,n); In; zeros(m,n) ];
        
        V = [ zeros(4*n+m+q,m); Im ];
        
        U = [ zeros(n+m,n); -In; F(:,:,i); Ef(:,:,i); zeros(n+m,n) ];
                      
        %%%=============================================================%%%
        %%%            [L; K; P] = [Z V U ]^T * inv(W) * U              %%%
        %%%=============================================================%%%
            
        Sol = [ Z V U ]' * inv(W) * U;
          
        Aux = mat2cell( Sol,[n m n],[n] );
        
        L(:,:,k,i) = Aux{1};
        
        K(:,:,k,i) = Aux{2};
                      
        P(:,:,k+1,i) = Aux{3};
        
        %%---------------------------------------------------
        
        % Verifica se a Riccati não é definida positiva 
           if min( eig( P(:,:,k+1,i ) ) ) <= 0
              error('A Riccati não é definida positiva.') 
           end
          
        %%%---------------------------------------------------

    end % for i

end % for k

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%