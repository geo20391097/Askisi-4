function rectify_image()
% Rectify a grayscale image using a given homography H and pixel size d.
% Reads 'lavrio.jpg' (grayscale), applies inverse homography, and saves
% 'lavrio_rectified.jpg'.
%
% The code follows the logic from the problem statement but improves:
% - supports non-integer image dimensions robustly
% - uses vectorized coordinate grid for speed
% - bilinear interpolation for smoother result

% Read image and ensure grayscale double
im_orig = imread('lavrio.jpg');
if ndims(im_orig)==3
    im = rgb2gray(im_orig);
else
    im = im_orig;
end
im = double(im);
[rows, cols] = size(im);

% Homography
H = [0.014308   0.000214   -1.973254;
    -0.000397  -0.011915    9.135886;
     0.000172   0.000072    1];

% pixel size
d = 0.01;

% Map the four image corners through H to find bounding box
corners = [1, 1, 1;
           cols, 1, 1;
           cols, rows, 1;
           1, rows, 1]'; % 3x4
mapped = H * corners; % 3x4
mapped = mapped ./ mapped(3,:); % normalize
X = mapped(1,:);
Y = mapped(2,:);

minX = min(X); maxX = max(X);
minY = min(Y); maxY = max(Y);

width_rec = maxX - minX;
height_rec = maxY - minY;

cols_rec = max(1, round(width_rec / d));
rows_rec = max(1, round(height_rec / d));

% Generate grid of target coordinates (world coordinates)
% X increases to right. For Y, follow original code: top row uses maxY decreasing.
xv = minX + (0:cols_rec-1) * d;
yv = maxY - (0:rows_rec-1) * d;
[Xg, Yg] = meshgrid(xv, yv); % rows_rec x cols_rec

% Inverse homography
h = inv(H);
% Apply inverse homography to all target points
Xs = Xg(:); Ys = Yg(:);
den = h(3,1)*Xs + h(3,2)*Ys + 1;
j_im = (h(1,1)*Xs + h(1,2)*Ys + h(1,3)) ./ den;
i_im = (h(2,1)*Xs + h(2,2)*Ys + h(2,3)) ./ den;

% Bilinear interpolation
% Prepare output and valid mask
im_rec = zeros(rows_rec, cols_rec);
valid = j_im >= 1 & j_im <= cols-1 & i_im >= 1 & i_im <= rows-1;

% Only interpolate valid points
jj = j_im(valid);
ii = i_im(valid);

% Coordinates for bilinear
i1 = floor(ii); j1 = floor(jj);
di = ii - i1; dj = jj - j1;

idx11 = sub2ind([rows, cols], i1, j1);
idx12 = sub2ind([rows, cols], i1, j1+1);
idx21 = sub2ind([rows, cols], i1+1, j1);
idx22 = sub2ind([rows, cols], i1+1, j1+1);

v11 = im(idx11);
v12 = im(idx12);
v21 = im(idx21);
v22 = im(idx22);

vals = (1-di).*(1-dj).*v11 + (1-di).*dj.*v12 + di.*(1-dj).*v21 + di.*dj.*v22;

temp = zeros(numel(Xs),1);
temp(valid) = vals;

im_rec(:) = temp;

% Display and save
figure; imshow(uint8(im)); title('Original Image');
figure; imshow(uint8(im_rec)); title('Rectified Image');

imwrite(uint8(im_rec),'lavrio_rectified.jpg');
end