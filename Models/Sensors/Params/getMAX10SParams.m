
function params = getMAX10SParams()


rng(42);  % Reproducibility

params.max_range = 800;
params.sens = 0.0625;

params.pos.bias = [0; 0; 0];
params.pos.noise = [2; 2; 2];

params.vel.bias = [0; 0; 0];
params.vel.noise = [0.05; 0.05; 0.05];


% TODO
%params.sf = 0.01 * randn(3,1);
%params.k2 = 0.001 * randn(3,1);
%params.k3 = 0.0001 * randn(3,1);

end
