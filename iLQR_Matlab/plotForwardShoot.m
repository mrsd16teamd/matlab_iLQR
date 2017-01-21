function plotForwardShoot(X)
% plots trajectory of center of mass, given forward-shoot X
% X size is (X_DIM, NUM_CTRL+1)

global PENDULUM;

plot(0,0,'o','Color','r','MarkerSize',12);
hold on

% Car model
if ~PENDULUM
    % footprint
    P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
    
    x = X(1,:);
    y = X(2,:);
    phi = wrapToPi(X(3,:));
    % affine transform
    A = [cos(phi(1)) -sin(phi(1)) x(1); sin(phi(1)) cos(phi(1)) y(1); 0 0 1];
    pos = A*P;
    plot(pos(1,:),pos(2,:),'Color','r');
    axis equal
    for i = 2:NUM_CTRL+1
        A = [cos(phi(i)) -sin(phi(i)) x(i); sin(phi(i)) cos(phi(i)) y(i); 0 0 1];
        pos = A*P;
        plot(pos(1,:),pos(2,:),'Color','b');
    end
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