function status=check_environment()
% Run before simulation; no installation or persistent path changes.
fprintf('MATLAB: %s\n',version);
products=ver; fprintf('Installed products: %d\n',numel(products));
status.optimization=exist('fmincon','file')==2 && license('test','Optimization_Toolbox');
status.mpc=exist('nlmpc','file')==2;
fprintf('Optimization: %d | MPC Toolbox: %d\n', ...
    status.optimization,status.mpc);
if status.optimization
    try
        fmincon(@(x)x^2,1,[],[],[],[],-2,2,[],optimoptions('fmincon','Display','off'));
    catch ME
        status.optimization=false; warning('%s',ME.message);
    end
end
fprintf('Simulation needs Optimization Toolbox.\n');
fprintf('This implementation uses fmincon directly; nlmpc is not a runtime dependency.\n');
end
