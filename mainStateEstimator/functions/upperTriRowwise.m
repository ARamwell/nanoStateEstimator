function v = upperTriRowwise(A)
% Includes diagonal, keeps zeros

    mask = triu(true(size(A)), 0);
    v = A(mask).';
end