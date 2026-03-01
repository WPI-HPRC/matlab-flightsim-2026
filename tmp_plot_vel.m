function [outputArg1,outputArg2] = untitled(priori_states)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
all_dcm = quat2dcm(priori_states(1:4, :)');
idx = size(all_dcm, 3);
disp(idx)
zeros_arr = zeros(idx, 1);

for k = 1:idx
    % Process each direction cosine matrix (DCM)
    %current_dcm = all_dcm(:, :, k);
    zeros_arr(k, :) = priori_states(5:7, idx(k));
end

plot(zeros_arr(:, 1))