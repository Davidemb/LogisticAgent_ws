clear all;
close all;
ri = imread('/home/dave/Desktop/model3.pgm');
imshow(ri);
hold on
%imrotate(ri,pi);
size(ri)
x = [575, 575, 740, 740, 740,  900, 900, 900, 1060, 1060, 1060, 1225,  1225, 1225, 1390,  1390, 1390, 1550,  1550, 1550];
yy = [175, 350, 175, 265, 350, 175, 265, 350,  175,  265,  350,  175,  265,  350,  175,    265,  350,  175,  265,  350];
y = -yy+576;
p=[x;y]'
%x=[171, 174];
%y=[442, 689];


plot(x,y,'go');




