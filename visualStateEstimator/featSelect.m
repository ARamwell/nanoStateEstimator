function [x_train, XW_train, x_test_flat, XW_test_flat] = featSelect(x_i, X_W, id)

numTest = int8(size(id,1)-1);
id_train = 1;

x_train = createArray(2,4);
XW_train = createArray(3,4);
% x_test = 
% X_test

x_train(1:2,1:4) = x_i(1:2,1:4,1);
XW_train(1:3,1:4) = X_W(1:3,1:4,1);
x_test_flat = NaN(2,4);
XW_test_flat = NaN(3,4);

if numTest>=1
    %choose ID to use for P3P (could replace this with RANSAC)
    
    %find training ID - all others used for test
    idx_train = find(id, id_train, "first");
    x_train = x_i(1:2,1:4,idx_train(1));
    XW_train = X_W(1:3,1:4, idx_train(1));
    
    %remove training ID from test set
    x_test = x_i;
    x_test(:,:,idx_train) = [];
    XW_test = X_W;
    XW_test(:,:,idx_train) = [];
    
    %flatten test set
    x_test_flat = reshape(x_test, 2, int8(numTest*4));
    XW_test_flat = reshape(XW_test, 3, int8(numTest*4));
    % x_test_flat = createArray(2, numTest*4);
    % X_test_flat = createArray(3, numTest*4);
    % for i = 1:numTest
    %     x_test_flat
end
