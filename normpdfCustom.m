function y = normpdfCustom(x, mu, sigma)
%NORMPDFCUSTOM Gaussian probability density function.
%   Standalone (no Statistics Toolbox dependency, works identically in
%   plain MATLAB and Octave) -- same reasoning as quat2rotmCustom.m.
%
%   y = normpdfCustom(x, mu, sigma)

    y = exp(-0.5*((x-mu)/sigma).^2) / (sigma*sqrt(2*pi));
end