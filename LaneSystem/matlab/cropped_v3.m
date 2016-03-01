function cropped_v3(number_of_images)

 if nargin == 0
   
     % Check the number of input arguments (nargin) 
    prompt={'Enter the number of images: ', 'Enter the directory:'};
    % Create all your text fields with the questions specified by the variable prompt.
    title1='Image Editor'; 
    % The main title of your input dialog interface.
    answer = inputdlg(prompt,title1);
     
 end


number_of_images = str2num(answer{1});



for i = 1:number_of_images
    filename = ['lane_' num2str(i,'%04u') '.png'];
    myImage = imread(filename);
    imshow(myImage);

resized_image_1 = imresize(myImage,[480 640]);
croppedImage = imcrop(resized_image_1,[1 245 640 480]);
resized_image = imresize(croppedImage,[480 640]);

subplot(1,2,1);
imshow(myImage);%show the image
title('Original Image');
subplot(1,2,2);
imshow(resized_image);%show the image
title('Cropped Image');

filename2 = ['cropped_' num2str(i,'%u') '.png'];
imwrite(resized_image, filename2);
end

end
