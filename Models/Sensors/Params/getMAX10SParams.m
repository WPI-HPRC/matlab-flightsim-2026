
function params = getMAX10SParams()


rng(42);  % Reproducibility

params.max_range = 800;
params.sens = 0.0625;

params.bias = [0; 0; 0];

params.noise = 2;

% TODO
params.sf = 0.01 * randn(3,1);
params.k2 = 0.001 * randn(3,1);
params.k3 = 0.0001 * randn(3,1);

end
