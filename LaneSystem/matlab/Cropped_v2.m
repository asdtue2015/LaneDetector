
for i = 1:611
    filename = ['lane_' num2str(i,'%u') '.png'];
    myImage = imread(filename);
    imshow(myImage);
    
croppedImage = imcrop(myImage,[1 265 640 480]);
resized_image = imresize(croppedImage,[480 640]);
subplot(1,2,1)
imshow(myImage)
title('Original Image')
subplot(1,2,2)
imshow(resized_image)
title('Cropped Image')
 filename2 = ['cropped_' num2str(i,'%u') '.png'];
imwrite(resized_image, filename2);
end



