function X = ForwardShoot(U, x_start, dt)
global NUM_CTRL;
global X_DIM;

DEBUG = 0; % for debugging

X = zeros(X_DIM, NUM_CTRL+1);

if DEBUG
    plot(x_start(1),x_start(2),'o','Color','r','MarkerSize',12);
    hold on
end
    
X(:, 1) = x_start;
for ii = 1 : NUM_CTRL
    X(:, ii+1) = dynamics_finite(X(:, ii)', U(:, ii), dt);
    
    if DEBUG
        plot(X(1,ii+1),X(2,ii+1),'*','Color','b');
        pause(0.1);
        disp(['x: ', num2str(X(1,ii+1))]);
        disp(['y: ', num2str(X(2,ii+1))]);
        disp(['p: ', num2str(X(3,ii+1))]);
        disp(['vx ', num2str(X(4,ii+1))]);
        disp(['vy ', num2str(X(5,ii+1))]);
        disp(['r: ', num2str(X(6,ii+1))]);
        disp(' ');
    end
end