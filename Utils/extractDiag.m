function diag = extractDiag(matrix)
%extractDiag Summary of this function goes here
%   Detailed explanation goes here
arguments (Input)
    matrix
end

arguments (Output)
    diag
end

diag = zeros(size(matrix, 1), 1);
for k = 1 : size(matrix, 1)
    diag(k) = matrix(k, k);

end
