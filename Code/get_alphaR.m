function [ alpha r] = get_alphaR( x, y)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

w = ones(length(x), 1);

x_bar_w = 1/sum(w)*sum(w.*x);
y_bar_w = 1/sum(w)*sum(w.*y);

N = -2*sum(w.*(y_bar_w-y).*(x_bar_w-x));
D = sum(w.*((y_bar_w-y).^2 - (x_bar_w-x).^2));

alpha = 0.5*atan2(N, D);

r = x_bar_w*cos(alpha)+y_bar_w*sin(alpha);
end

