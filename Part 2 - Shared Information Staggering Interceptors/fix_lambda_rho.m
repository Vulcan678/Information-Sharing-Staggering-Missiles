function [lambda,rho] = fix_lambda_rho(lambda,rho)
% Function fixing the variables rho and lambda to enforce rho>0
% Input: 
% lambda (rad) - the LOS angle
% rho (m) - the distance between the target and the pursuer
% Output: 
% lambda (rad) - the fixed LOS angle
% rho (m) - the fixed distance between the target and the pursuer

% force that rho>0
if rho<0
    rho=-rho;
    lambda=lambda+pi;
end

% keep lambda in the range of (-pi,pi]
while (lambda>pi) || (lambda<=-pi)
    lambda=lambda-2*pi*sign(lambda);
end

end

