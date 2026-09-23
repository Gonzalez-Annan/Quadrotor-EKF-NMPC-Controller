function [u,ctrl,info]=nmpc(x,t,ctrl,c,varargin)
%NMPC Compatibility entry point for the original empty placeholder.
% c=project_config(); ctrl=quad.mpc_init(c);
% [u,ctrl,info]=nmpc(x,t,ctrl,c);
[u,ctrl,info]=quad.mpc_step(x,t,ctrl,c,varargin{:});
end
