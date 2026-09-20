function [u,ctrl,info]=mpc_step(x,t,ctrl,c,ref)
if nargin<5, ref=quad.reference(t+(1:c.mpc.horizon)*c.controlDt,c); end
cachedZ=[]; cachedJ=[]; cachedC=[]; cachedDJ=[]; cachedDC=[];
nb=numel(c.mpc.blocks); A=kron(eye(nb),c.mpc.inputMap);
bhi=repmat(c.thrustMax-c.hover,nb,1); blo=repmat(c.hover-c.thrustMin,nb,1);
ticID=tic;
[z,f,exitflag,out]=fmincon(@objective,ctrl.guess,[A;-A],[bhi;blo],[],[], ...
    [],[],@constraint,ctrl.options);
elapsed=toc(ticID);
[~,g]=quad.mpc_problem(z,x,ctrl.lastU,ref,c);
violation=max([0;g;A*z-bhi;-A*z-blo]);
usable=all(isfinite(z)) && isfinite(f) && violation<=c.mpc.acceptTolerance && exitflag>=0;
info.solveTime=elapsed; info.exitflag=exitflag; info.cost=f;
info.violation=violation; info.iterations=out.iterations; info.fallback=~usable;
if usable
    u=c.hover+c.mpc.inputMap*z(1:4); ctrl.failures=0;
    map=repelem(1:numel(c.mpc.blocks),c.mpc.blocks);
    Z=reshape(z,4,[]); sequence=Z(:,map); sequence=[sequence(:,2:end),sequence(:,end)];
    first=[1,1+cumsum(c.mpc.blocks(1:end-1))];
    ctrl.guess=reshape(sequence(:,first),[],1);
else
    % Holding the last bounded command is an emergency continuity measure,
    % NOT a guarantee of state safety. Abort after repeated failures.
    u=ctrl.lastU; ctrl.failures=ctrl.failures+1;
    ctrl.guess=repmat(c.mpc.inputMap\(u-c.hover),numel(c.mpc.blocks),1);
    if ctrl.failures>=c.mpc.maxConsecutiveFailures
        error('quad:MPCFailure','NMPC failed %d consecutive times (exit %d, violation %.3g).', ...
            ctrl.failures,exitflag,violation);
    end
end
ctrl.lastU=u;
    function refresh(v)
        if isempty(cachedZ)||~isequal(v,cachedZ)
            [cachedJ,cachedC,cachedDJ,cachedDC]=quad.mpc_problem(v,x,ctrl.lastU,ref,c);
            cachedZ=v;
        end
    end
    function [f,df]=objective(v)
        refresh(v); f=cachedJ; df=cachedDJ;
    end
    function [a,b,da,db]=constraint(v)
        refresh(v); a=cachedC; b=[]; da=cachedDC; db=[];
    end
end
