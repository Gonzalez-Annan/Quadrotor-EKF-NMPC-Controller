function [J,g,dJ,dg]=mpc_problem(z,x0,lastU,ref,c)
% Direct shooting, move blocking, RK4, hard constraints at substeps.
% Batched complex-step derivatives avoid subtractive finite-difference noise.
n=numel(z);
if nargout>2
    h=1e-20; Z=[z,repmat(z,1,n)+1i*h*eye(n)];
else
    Z=z;
end
ns=size(Z,2); X=repmat(x0,1,ns); last=repmat(lastU,1,ns);
map=repelem(1:numel(c.mpc.blocks),c.mpc.blocks);
cost=zeros(1,ns,'like',Z); constraints=[];
for k=1:c.mpc.horizon
    U=c.hover+c.mpc.inputMap*Z(4*map(k)-3:4*map(k),:);
    for j=1:c.mpc.integrationSubsteps
        X=quad.step(X,U,c.controlDt/c.mpc.integrationSubsteps,c);
        constraints=[constraints;quad.state_constraints(X,c)]; %#ok<AGROW>
    end
    ep=X(1:3,:)-ref.p(:,k); ev=X(4:6,:)-ref.v(:,k);
    ew=X(11:13,:)-ref.w(:,k);
    % Sign-invariant orientation cost: q and -q represent the same attitude.
    eq=1-sum(X(7:10,:).*ref.q(:,k),1).^2;
    stage=sum(c.mpc.positionWeight.*ep.^2,1)+sum(c.mpc.velocityWeight.*ev.^2,1) ...
        +c.mpc.attitudeWeight*eq+sum(c.mpc.rateWeight.*ew.^2,1);
    if k==c.mpc.horizon, stage=c.mpc.terminalMultiplier*stage; end
    cost=cost+c.controlDt*(stage+c.mpc.inputWeight*sum((U-c.hover).^2,1) ...
        +c.mpc.slewWeight*sum((U-last).^2,1));
    last=U;
end
J=real(cost(1)); g=real(constraints(:,1));
if nargout>2
    dJ=imag(cost(2:end)).'/h;
    dg=imag(constraints(:,2:end)).'/h; % fmincon convention nVar x nConstr.
end
end
