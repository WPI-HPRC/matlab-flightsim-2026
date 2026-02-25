function y_tilde = applySensorModel(y, params, rand)
% random_num gaussian N(0, 1)
num_elems = size(y, 1);

y_tilde = zeros(size(y));

for i = 1:num_elems
    yi = y(i);

    bias = params.bias(i);
    %sf   = params.sf(i);
    %k2   = params.k2(i);
    %k3   = params.k3(i);
    noise = params.bias(i) + params.noise(i) * rand;

    %% DISABLE HIGHER ORDER TERMS TERMPORARILIY
    % y_tilde(i) = yi + bias + sf * yi + k2 * yi^2 + k3 * yi^3 + noise;
    y_tilde(i) = yi + bias + noise;
    % y_tilde(i) = yi;
end

end