

myImage = imread('lane_1.png');
imshow(myImage);

croppedImage = imcrop(myImage,[1 265 640 480]);
resized_image = imresize(croppedImage,[480 640]);
subplot(1,2,1)
imshow(myImage)
title('Original Image')
subplot(1,2,2)
imshow(resized_image)
title('Cropped Image')
imwrite(resized_image,'Cropped_.png');

