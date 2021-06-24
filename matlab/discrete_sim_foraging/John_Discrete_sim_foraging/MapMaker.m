
mapMaker("JohnMap.png","What.png");



function mapMaker(filename,newfile)
img1 = imread(filename);
img2 = img1 ~= 0;
img2 = img1(:,:,1); 
imwrite(img2,newfile);

end