function plotForwardShoot(X)
% plots trajectory of center of mass, given forward-shoot X
% X size is (X_DIM, NUM_CTRL+1)
global NUM_CTRL;
global PENDULUM;
global ANIM;

plot(0,0,'o','Color','r','MarkerSize',12);
hold on

% Car model
if ~PENDULUM
    if ANIM
        hold off
    end
    % footprint
    P = [-0.15  -0.15  0.15  0.15  -0.15; -0.08  0.08  0.08  -0.08  -0.08; 1 1 1 1 1];
    
    x = X(1,:);
    y = X(2,:);
    phi = wrapToPi(X(3,:));
    % affine transform
    A = [cos(phi(1)) -sin(phi(1)) x(1); sin(phi(1)) cos(phi(1)) y(1); 0 0 1];
    pos = A*P;
    plot(pos(1,:),pos(2,:),'Color','r');
    axis([-3 3 -3 3])
    axis equal
    
    if ANIM
        step = 1;
    else
        step = 3;
    end
    
    for i = 2:step:NUM_CTRL
        A = [cos(phi(i)) -sin(phi(i)) x(i); sin(phi(i)) cos(phi(i)) y(i); 0 0 1];
        pos = A*P;
        plot(pos(1,:),pos(2,:),'Color','b');

        if ANIM
            axis([-3 3 -3 3])
            axis equal
            drawnow
        end
    end
    
    i = NUM_CTRL+1;
    A = [cos(phi(i)) -sin(phi(i)) x(i); sin(phi(i)) cos(phi(i)) y(i); 0 0 1];
    pos = A*P;
    plot(pos(1,:),pos(2,:),'Color','g');
    plot(x(i),y(i),'o','Color','g','MarkerSize',12);
    axis([-3 3 -3 3])
    axis equal
end

% Pendulum model
if PENDULUM
    theta = X(1,:);
    theta = theta - pi/2;
    axis([-1 1 -1 1]); 
    plot(cos(theta(1)),sin(theta(1)),'o','MarkerSize',12,'Color', 'g');
    for i = 2:5:NUM_CTRL+1
        plot(cos(theta(i)),sin(theta(i)),'*','Color', 'k');
        plot([0,cos(theta(i))],[0,sin(theta(i))],'--','Color','k');
        pause(0.01)
    end
    plot(cos(theta(end)),sin(theta(end)),'o','MarkerSize',12,'Color', 'b');
    plot([0,cos(theta(end))],[0,sin(theta(end))]);
end

hold off
pause(0.5)
end