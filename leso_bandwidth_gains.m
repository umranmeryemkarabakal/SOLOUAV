function [beta1, beta2] = leso_bandwidth_gains(wo)
%LESO_BANDWIDTH_GAINS  Gozlemci bant genisligi (rad/s) -> [beta1,beta2].
%   Karakteristik denklem (s+wo)^2 = s^2 + 2*wo*s + wo^2
beta1 = 2*wo;
beta2 = wo^2;
end
