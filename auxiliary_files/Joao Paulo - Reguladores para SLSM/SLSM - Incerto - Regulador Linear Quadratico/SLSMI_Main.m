%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%               RLQ Robusto para SLSM INCERTO Dependente do Modo          %
%                                  MAIN                                   %
%                                                                         %
% Dados de entrada ->  arquivo: Dados do sistema                          %
%                     function: SLSMI_Regulador_Robusto                   %
%                                                                         %
% Dados de saida -> arquivos: Resultados.mat (N parms mi alfa K L P)      %
%                             graficos das simula��es                     %
%                                                                         %
%%=======================================================================%%
% Daiane C. Bortolin    -    daiane.bortolin@usp.br   -    Outubro / 2016 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%%%=====================================================================%%%
%%%                      Atribui��o dos Par�metros                      %%%
%%%=====================================================================%%%
          
     N = 21;               % horizonte
 parms = 0;                % par�metro que caracteriza o valor de mi
                           % parms = 0 -> mu = 0 / parms = 1 -> mu ~= 0
    mi = 1e+16;            % par�metro de penalidade ( > 0)
  alfa = 0.5;              % pertence ao par�metro de minimiza��o ( > 0)
                           % lambda = 1 + alfa
                            
%%%=====================================================================%%%
%%%                        Dados do Sistema                             %%%
%%%=====================================================================%%%
  
Joao2010_cdc;

% Vari�veis globais
  warning off
   global m n q s Prob F G Q R H Ef Eg P
  warning on
  
%%%=====================================================================%%% 
%%%         Regulador Robusto Recursivo para SLSM INCERTO com MTN       %%%
%%%=====================================================================%%%

[P,K,L] = SLSMI_Regulador_Robusto(N,parms,mi,alfa);
size(P)

%%%--------------------------------------%%%
%%%               Resultados             %%%
%%%--------------------------------------%%%

% Solu��o da Equa��o de Riccati
%   Ric = P(:,:,1,:)
  
%   disp('----------------------')
  
% Ganho de Realimenta��o
  Kr = K(:,:,end,:)
  
% disp('----------------------')

% Malha Fechada
  Lr = L(:,:,end,:);

%%%=====================================================================%%%
%%%                       Simula��o - Sistema Real                      %%%
%%%=====================================================================%%%
 
% Condi��es Iniciais 
   x(:,1) = x0;
  xr(:,1) = x0;
   x_norm(1) = norm( x0 );
  xr_norm(1) = norm( x0 );

%%%---------------------------------------------------

% Vetor de distribui��o inicial
  p0 = (1/s) * ones(1,s);                   
  
% Simula��o da cadeia de Markov
  cd ('Gera_theta')
     theta = samplefrommarkov(p0,Prob,N);
  cd ..

% Simula��o do par�metro de incertezas
  delta = -1 + 2 * rand(1,N);

%%%---------------------------------------------------

for k = 1:N-1
     
       u(:,k) = K(:,:,k,theta(k)) * x(:,k);
     x(:,k+1) = L(:,:,k,theta(k)) * x(:,k);
     
     x_norm(k+1) = norm( x(:,k+1) );
     
     %----------------------------------------
     
     % Simula��o do Sistema Real
       dF = H(:,:,theta(k)) * delta(k) * Ef(:,:,theta(k));
       dG = H(:,:,theta(k)) * delta(k) * Eg(:,:,theta(k));
         ur(:,k) = K(:,:,k,theta(k)) * xr(:,k);
       xr(:,k+1) = ( F(:,:,theta(k)) + dF ) * xr(:,k) + ...
                  ( G(:,:,theta(k)) + dG ) * ur(:,k);
              
       xr_norm(k+1) = norm( xr(:,k+1) );
                        
end
    
%%%---------------------------------------------------

%%% Verifica��o do resultado (qdo parms = 0)
%   for i = 1:s
%       MI(:,:,i) = Ef(:,:,i) + Eg(:,:,i) * K(:,:,end,i);
%       MF(:,:,i) = F(:,:,i) + G(:,:,i) * K(:,:,end,i) - L(:,:,end,i);
%   end
 
%%%------------------------------------%%%
%%% Graficos  
%%%------------------------------------%%%

%%% Compara��o das solu��es
    figure(1)
     plot(0:N-1,x_norm,'-',0:N-1,xr_norm,'-r')
     legend('Regulador','Real')
     title('Norma dos estados regulados')
     grid on
    
%%% Malha Fechada
    figure(2)
    plot(0:N-1,x,'-*')
    xlabel('Horizonte N')
    ylabel('x(k)')
    legend('x_1(k)','x_2(k)','x_3(k)')
    title('Malha Fechada')
    grid on
    
%%% Acao do controle robusto
    figure(3)
    plot(0:N-2,ur,'-*')
    xlabel('Horizonte N')
    ylabel('u(k)')
    legend('u_1(k)','u_2(k)','u_3(k)')
    title('A��o do Controle Robusto')
    grid on
 
%%%=====================================================================%%%
%%%                              Saida                                  %%%
%%%=====================================================================%%%

% save SLSMinc_Resultados.mat parms mi alfa theta N K L P x u
% saveas( figure(1),'Regulador_si_state.fig' )
% saveas( figure(2),'Regulador_si_control.fig' )

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%