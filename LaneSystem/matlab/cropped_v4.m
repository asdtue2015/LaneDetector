function cropped_v4(directory, pixels_number)

 if nargin == 0
   
     % Check the number of input arguments (nargin) 
    prompt={'Enter the folder_name:', 'Enter pixels_number:'};
    % Create all your text fields with the questions specified by the variable prompt.
    title1='Image Editor'; 
    % The main title of your input dialog interface.
    answer = inputdlg(prompt,title1);
     
 end


directory = answer{1};
pixels_number = str2num(answer{2});

cd (directory)

A = dir( fullfile(directory, '*.png') );
fileNames = { A.name };

for iFile = 1 : numel(A)
  newName = fullfile(directory, sprintf( ['lane_' num2str(iFile,'%04u') '.png'], iFile ) );
  movefile( fullfile(directory, fileNames{ iFile }), newName );    
end 



for i = 1:numel(A)
    filename = ['lane_' num2str(i,'%04u') '.png'];
    myImage = imread(filename);
    imshow(myImage);

resized_image_1 = imresize(myImage,[480 640]);
croppedImage = imcrop(resized_image_1,[1 pixels_number 640 480]);
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
