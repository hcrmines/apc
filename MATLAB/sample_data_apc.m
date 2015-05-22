% this function downsample the number of instances of each category

clc
clear 
close all

imgext = '.jpg';
mskext = '.pnm';
imgdir= '~/VOC_Data/APC/elmers_washable_no_run_school_glue/resize';
masksdir = fullfile(imgdir, 'masks');

datadir = '~/VOC_Data/APC/Data';
imdatadir = fullfile(datadir, 'img');
datamskdir = fullfile(datadir, 'masks');

% object label
catlabel='Obj06';
% instance downsample rate
samplerate = 20;

cd( imgdir );
imname  = getFilenames(imgdir,imgext);
mskname = getFilenames(masksdir,mskext);

ilength =length(imname);

for i=1:samplerate:ilength
    copyfile(fullfile(imgdir, [imname{i} imgext] ),...
             fullfile(imdatadir, [catlabel imname{i} imgext] ));
    
    copyfile(fullfile(masksdir, [imname{i} '_mask' mskext] ),...
             fullfile(datamskdir, [catlabel imname{i} '_mask' mskext] ));
end


