%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                 Simulating a discrete time Markov chain                 %
%                                                                         %
% Algorithm from Lecture "Notes for Math 450 - Lecture Notes 1" by        %
% R. Feres (2007, pg 19).                                                 %
% Fonte: < ttp://www.math.wustl.edu/~feres/Math450Lect01.pdf >.           %
%                                                                         %
% Inputs - p probability distribution of initial state                    %
%        - P transition probability matrix                                %
%        - n n number of iterates.                                        %
%                                                                         %
% Output - X sample chain of length n.                                    %
%                                                                         %
% Note: need function < samplefromp.m >.                                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function X = samplefrommarkov(p,Prob,n)

%----------------------------------------------------------

% Make sure the entries in the probability vector add up to 1
  %  if ( sum(p) ~= 1) 
  %      error('p: Given probability vector is not valid!')
  % end
   if ( 1 - sum(p) >= eps) 
       error('p: Given probability vector is not valid!')
  end
 
% Make sure the entries in the transition matrix add up to 1
 if ( sum( sum(Prob,2) ) ~= length(Prob) )
     error('p: Given transition matrix is not valid!')
 end

%----------------------------------------------------------

q = p;
i = samplefromp(q,1);
X = [ i ];

for j = 1:n-1
    q = Prob(i,:);
    sum(q);
    i = samplefromp(q,1);
    X = [X i];
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%