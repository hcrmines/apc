load('K.mat');

I = imread('scene.jpg');
I = rgb2gray(I);
figure, imshow(I, []), impixelinfo;

rect = uint8(zeros(size(I)));
for x = 1 : size(I, 2)
    for y = 1 : size(I, 1)
        p = Kdi * Kc * [x; y; 1];
        rect(y, x) = I(round(p(2)), round(p(1)));
    end
end


figure, imshow(rect, []), impixelinfo;