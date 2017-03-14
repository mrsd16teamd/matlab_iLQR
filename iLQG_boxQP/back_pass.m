function [diverge, Vx, Vxx, k, K, dV] = back_pass(cx,cu,cxx,cxu,cuu,fx,fu,fxx,fxu,fuu,lambda,regType,lims,u)
% Perform the Ricatti-Mayne backward pass

% tensor multiplication for DDP terms
vectens = @(a,b) permute(sum(bsxfun(@times,a,b),1), [3 2 1]);

N  = size(cx,2);
n  = numel(cx)/N;
m  = numel(cu)/N;

cx    = reshape(cx,  [n N]);
cu    = reshape(cu,  [m N]);
cxx   = reshape(cxx, [n n N]);
cxu   = reshape(cxu, [n m N]);
cuu   = reshape(cuu, [m m N]);

k     = zeros(m,N-1);
K     = zeros(m,n,N-1);
Vx    = zeros(n,N);
Vxx   = zeros(n,n,N);
dV    = [0 0];

Vx(:,N)     = cx(:,N);
Vxx(:,:,N)  = cxx(:,:,N);

diverge  = 0;
for i = N-1:-1:1
    
    Qu  = cu(:,i)      + fu(:,:,i)'*Vx(:,i+1);
    Qx  = cx(:,i)      + fx(:,:,i)'*Vx(:,i+1);
    Qux = cxu(:,:,i)'  + fu(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    if ~isempty(fxu)
        fxuVx = vectens(Vx(:,i+1),fxu(:,:,:,i));
        Qux   = Qux + fxuVx;
    end
    
    Quu = cuu(:,:,i)   + fu(:,:,i)'*Vxx(:,:,i+1)*fu(:,:,i);
    if ~isempty(fuu)
        fuuVx = vectens(Vx(:,i+1),fuu(:,:,:,i));
        Quu   = Quu + fuuVx;
    end
    
    Qxx = cxx(:,:,i)   + fx(:,:,i)'*Vxx(:,:,i+1)*fx(:,:,i);
    if ~isempty(fxx)
        Qxx = Qxx + vectens(Vx(:,i+1),fxx(:,:,:,i));
    end
    
    Vxx_reg = (Vxx(:,:,i+1) + lambda*eye(n)*(regType == 2));
    
    Qux_reg = cxu(:,:,i)'   + fu(:,:,i)'*Vxx_reg*fx(:,:,i);
    if ~isempty(fxu)
        Qux_reg = Qux_reg + fxuVx;
    end
    
    QuuF = cuu(:,:,i)  + fu(:,:,i)'*Vxx_reg*fu(:,:,i) + lambda*eye(m)*(regType == 1);
    
    if ~isempty(fuu)
        QuuF = QuuF + fuuVx;
    end
    
    if nargin < 13 || isempty(lims) || lims(1,1) > lims(1,2)
        % no control limits: Cholesky decomposition, check for non-PD
        [R,d] = chol(QuuF);
        if d ~= 0
            diverge  = i;
            return;
        end
        
        % find control law
        kK = -R\(R'\[Qu Qux_reg]);
        k_i = kK(:,1);
        K_i = kK(:,2:n+1);
        
    else        % solve Quadratic Program
        lower = lims(:,1)-u(:,i);
        upper = lims(:,2)-u(:,i);
        
        [k_i,result,R,free] = boxQP(QuuF,Qu,lower,upper,k(:,min(i+1,N-1)));
        if result < 1
            diverge  = i;
            return;
        end
        
        K_i    = zeros(m,n);
        if any(free)
            Lfree        = -R\(R'\Qux_reg(free,:));
            K_i(free,:)   = Lfree;
        end
        
    end
    
    % update cost-to-go approximation
    dV          = dV + [k_i'*Qu  .5*k_i'*Quu*k_i];
    Vx(:,i)     = Qx  + K_i'*Quu*k_i + K_i'*Qu  + Qux'*k_i;
    Vxx(:,:,i)  = Qxx + K_i'*Quu*K_i + K_i'*Qux + Qux'*K_i;
    Vxx(:,:,i)  = .5*(Vxx(:,:,i) + Vxx(:,:,i)');
    
    % save controls/gains
    k(:,i)      = k_i;
    K(:,:,i)    = K_i;
end
end %back_pass
