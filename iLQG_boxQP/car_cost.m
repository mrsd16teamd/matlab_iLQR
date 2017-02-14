function c = car_cost(x, u)
% cost function for car-parking problem
% sum of 3 terms:
% lu: quadratic cost on controls
% lf: final cost on distance from target parking configuration
% lx: running cost on distance from origin to encourage tight turns
global x_des;
global du;
final = isnan(u(1,:));
u(:,final)  = 0;

cu  = 1e-2*[.1 .1];         % control cost coefficients
cdu = 1e-1*[.01 1];         % change in control cost coefficients

cf  = [ 10 10 1 .1 .1 .1];    % final cost coefficients
pf  = [ .01 .01 .1 .1 .1 .1]';    % smoothness scales for final cost

cx  = 1e-2*[1  1 0.8];          % running cost coefficients
px  = [.01 .01 .1]';             % smoothness scales for running cost

% control cost
lu    = cu*u.^2;
ldu   = 50*cdu*x(9:10,:).^2; %cdu*x(9:10,:).^2; Smoother curve?

% final cost
if any(final)
   dist = bsxfun(@minus,x(1:6,final),x_des(1:6));
   llf      = cf*(sabs(dist,pf)+sabs(dist,pf).^2);
%    llf      = cf*sabs(x(:,final),pf);
   lf       = double(final);
   lf(final)= llf;
else
   lf    = 0;
end

% running cost
dist = bsxfun(@minus,x(1:3,:),x_des(1:3));
lx = cx*sabs(dist,px);
% lx = cx*sabs(x(1:2,:),px);

% drift prize
ld = -0.001*(sabs(x(5,:),1)-0.2);

% obstacle cost
lobs = 15*getmapCost(x(1,:),x(2,:));

% total cost
c     = lu + lf + lx + ldu + ld + lobs;
end

function lobs = getmapCost(x,y)
    global x0;
    global costmap;
    if isempty(costmap)
        lobs = 0;
        return
    end
    x = (round(x,1)-(x0(1)-2))*10+1;
    y = (round(y,1)-(x0(2)-2))*10+1;
    lobs = zeros(1,length(x));
    for i = 1:length(x)
        try
            lobs(i) = costmap(y(i),x(i));
        catch ME
            lobs(i) = 0;
        end
    end
end

% smooth absolute-value function (a.k.a pseudo-Huber)
function y = sabs(x,p)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end