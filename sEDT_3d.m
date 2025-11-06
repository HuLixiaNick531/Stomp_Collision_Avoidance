% Compute 3D signed EDT, where A is the binary map of voxels
% Note that no 0-cost boundary voxels are used for simplicity

function Q = sEDT_3d(A)
% Reverse of the binary map
Ainv = 1 - A;
% Signed EDT
Q = bwdist(A, 'Euclidean') - bwdist(Ainv, 'Euclidean');

end