%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                             Sample from p                               %
% Drawing (sampling) numbers from {1, 2, ... , k} with probabilities      %
% p1, p2, . . . , pk.                                                     %
%                                                                         %
% Algorithm from Lecture "Notes for Math 450 - Lecture Notes 1" by        %
% R. Feres (2007, pg 14)                                                  %
% Fonte: < ttp://www.math.wustl.edu/~feres/Math450Lect01.pdf >.           %
%                                                                         %
% Inputs - p is the probability vector of length k                        %
%        - n is the number of random                                      %
%            integers from 1,2, ...,k returned                            %
%                                                                         %
% Output - a row vector of length n with entries                          %
%          from the set {1, 2, ..., k} with                               %
%          probabilities specified by p.                                  %
%                                                                         %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function x = samplefromp(p,n)

%----------------------------------------------------------

% Make sure the entries in the probability vector add up to 1
  %  if ( sum(p) ~= 1) 
  %      error('p: Given probability vector is not valid!')
  % end
   if ( 1 - sum(p) >= eps) 
       error('p: Given probability vector is not valid!')
   end

%----------------------------------------------------------

k = size(p,2);
u = rand(1,n);
x = zeros(1,n);

for i = 1:k
    x = x + i*( sum(p(1:i))-p(i) <= u & u < sum(p(1:i)) );    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%