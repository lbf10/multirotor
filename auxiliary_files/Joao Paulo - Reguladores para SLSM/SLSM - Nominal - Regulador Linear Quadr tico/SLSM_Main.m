%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%         Regulador Linear Quadrático para SLSM Dependente do Modo        %
%                                  MAIN                                   %
%                                                                         %
% Dados de entrada ->  arquivo: Dados do sistema                          %
%                     function: SLSM_Regulador                            %
%                                                                         %
% Dados de saida -> arquivos: Resultados.mat (N parms mi alfa K L P)      %
%                             graficos das simulações                     %
%                                                                         %
%%=======================================================================%%
% Daiane C. Bortolin    -    daiane.bortolin@usp.br   -    Outubro / 2016 %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

close all
clear all
clc

%%%=====================================================================%%%
%%%                      Atribuição dos Parâmetros                      %%%
%%%=====================================================================%%%
         
     N = 51;               % horizonte
 parms = 0;                % parâmetro que caracteriza o valor de mi
                           % parms = 0 -> mu = 0 / parms = 1 -> mu ~= 0
    mi = 1e+10;            % parâmetro de penalidade ( > 0)
                            
%%%=====================================================================%%%
%%%                         Dados do Sistema                            %%%
%%%=====================================================================%%%
  
Joao2013_Ex23;

% Variaveis globais
  warning off
  global m n s Prob F G Q R P
  warning  on
  
%%%=====================================================================%%% 
%%%               Regulador Recursivo para SLSM com MTN                 %%%
%%%=====================================================================%%%

[P,K,L] = SLSM_Regulador(N,parms,mi);

%%%--------------------------------------%%%
%%%               Resultados             %%%
%%%--------------------------------------%%%

% Solução da Equação de Riccati
%   Ric = P(:,:,end,:)
  
%   disp('----------------------')
  
% Ganho de Realimentação
  Kr = K(:,:,end,:)
  
% disp('----------------------')

% Malha Fechada
  % Lr = L(:,:,N,:)

%%%=====================================================================%%%
%%%                       Simulação - Sistema Real                      %%%
%%%=====================================================================%%%
 
% Condições Iniciais 
   x(:,1) = x0;
  xr(:,1) = x0;
   x_norm(1) = norm( x0 );
  xr_norm(1) = norm( x0 );

%%%---------------------------------------------------

% Vetor de distribuição inicial
  p0 = (1/s) * ones(1,s);                   
  
% Simulação da cadeia de Markov
  cd ('Gera_theta')
     theta = samplefrommarkov(p0,Prob,N);
  cd ..

%%%---------------------------------------------------

for k = 1:N-1
     
    % Simulação do Regulador
      u(:,k)   = K(:,:,k,theta(k)) * x(:,k);
      x(:,k+1) = L(:,:,k,theta(k)) * x(:,k);
      
      x_norm(k+1) = norm( x(:,k+1) );
     
    %----------------------------------------
     
    % Simulação do Sistema Real
      ur(:,k) = K(:,:,k,theta(k)) * xr(:,k);
      xr(:,k+1) = F(:,:,theta(k)) * xr(:,k) + G(:,:,theta(k))  * ur(:,k);
      
      xr_norm(k+1) = norm( xr(:,k+1) );

                    
end

%%%---------------------------------------------------

%%% Verificação do resultado (qdo parms = 0)
%   for i = 1:s
%       MF(:,:,i) = F(:,:,i) + G(:,:,i) * K(:,:,end,i) - L(:,:,end,i);
%   end

%%%------------------------------------%%%
%%% Graficos  
%%%------------------------------------%%%

%%% Comparação das soluções
    figure(1)
     plot(0:N-1,x_norm,'-',0:N-1,xr_norm,'-r')
     legend('Regulador','Real')
     title('Norma dos estados regulados')
     grid on
     
%%% Malha Fechada
    figure(2)
    plot(x(1,:),x(2,:),'-*')
    xlabel('x_1(k)')
    ylabel('x_2(k)')
    title('Malha Fechada')
    grid on

%%% Ação do controle robusto
    figure(3)
    plot(0:N-2,u,'-*')
    xlabel('Horizonte N')
    ylabel('u(k)')
    title('Controle')
    grid on
    
%%%=====================================================================%%%
%%%                              Saida                                  %%%
%%%=====================================================================%%%

% save SLSM_Resultados.mat parms mi alfa N theta K L P x u
% saveas( figure(1),'Regulador_state.fig' )
% saveas( figure(2),'Regulador_control.fig' )
 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%