% convert image and mask data from APC to GTcls and GTinst

clc
clear 
close all

mskext = '.pnm';
imgext = '.jpg';
datadir='/home/hao/Documents/Toolbox/Data';
imdir = fullfile(datadir, 'img');
maskdir = fullfile(datadir, 'masks');
clsdir = fullfile(datadir, 'cls');
instdir = fullfile(datadir, 'inst');


msknames = getFilenames(maskdir, mskext);
imnames = getFilenames(imdir, imgext);

for ifile = 1:length (msknames)
    [GTcls, GTinst]  = img2GTcls( maskdir, msknames{ifile}, mskext );
    [~,name,~] = fileparts(imnames{ifile});
    clsfilename = fullfile(clsdir, [name '.mat']);
    instfilename = fullfile(instdir, [name '.mat']);
    save(clsfilename,'GTcls');
    save(instfilename,'GTinst');
end





