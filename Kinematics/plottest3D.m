% Vectors
A = [0 0 0];
B = [1 0 0];
C = [0.5 0.5 0];
D = [0.5 0.5 1];

plotVec(A, B);
hold on;
plotVec(A, C);
plotVec(A, D);
% quiver3(0, 0, 0, B(1), B(2), B(3));