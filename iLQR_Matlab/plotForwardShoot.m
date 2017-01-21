function plotForwardShoot(X)
% plots trajectory of center of mass, given forward-shoot X
% X size is (X_DIM, NUM_CTRL+1)

global PENDULUM;

plot(0,0,'o','Color','r','MarkerSize',12);
hold on

% Car model
if ~PENDULUM
    x = X(1,:);
    y = X(2,:);
    plot(x,y,'*','Color','b');
end

% Pendulum model
if PENDULUM
    theta = X(1,:);
    theta = theta - pi/2;
    axis([-1 1 -1 1]); 
    plot(cos(theta(1)),sin(theta(1)),'o','MarkerSize',12,'Color', 'g');
    plot(cos(theta(end)),sin(theta(end)),'o','MarkerSize',12,'Color', 'b');
    plot([0,cos(theta(end))],[0,sin(theta(end))]);

    plot(cos(theta),sin(theta),'*','Color', 'k');
end

hold off
pause(1)
end