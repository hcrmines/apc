test_case = 11;

imlist = {...
    'Obj15N3_0', ... % 1
    'apc_obj15_test1', ... % 2
    'apc_obj15_test2', ... % 3
    'apc_obj06_test1', ... % 4
    'apc_obj06_test2', ... % 5
    'apc_obj06_test3', ... % 6
    'apc_test1', ... % 7
    'apc_test2', ... % 8
    'apc_test3', ... % 9
    'image_to_recognize',... #10
    'two_objects'
    };

imname=imlist{test_case};

imgdir=pwd;

cachedir=fullfile(pwd, 'cachedir');

imgfile=fullfile(imgdir, [imname '.jpg']);
img = imread(imgfile);

% resize HD image to height x 640
sz_img = size(img);
if sz_img(2) > 640
    img = imresize(img, [NaN, 640]);
end
imwrite(img, imgfile);

if(~exist(cachedir, 'file')) mkdir(cachedir); end
mcgdir=fullfile(cachedir, 'mcg');
ovoutdir=fullfile(cachedir, 'overlaps');
sptextdir=fullfile(cachedir, 'sptextdir');
regspimgdir=fullfile(cachedir, 'reg2spimgdir');
featdir=fullfile(cachedir, 'featdir');
refinedoutdir=fullfile(cachedir, 'refinement_out');
scorefile=fullfile(cachedir, 'scores.mat');

k_top_candidate = 100;
imagelist_to_apc({imname}, ...
                 imgdir,...
                 mcgdir,...
                 ovoutdir,...
                 sptextdir,...
                 regspimgdir,...
                 featdir,...
                 refinedoutdir,...
                 scorefile,...
                 k_top_candidate);
             
tmp=load(scorefile);
chosen=tmp.topchosen;
chosenscores=tmp.topscores;

%visualize the top detections
obj_id = 6;
max_candidate_to_visualize = 5;
visualize_apc({imname}, imgdir, sptextdir, regspimgdir, chosen, chosenscores, obj_id, max_candidate_to_visualize);
visualize_apc({imname}, imgdir, sptextdir, regspimgdir, chosen, chosenscores, 15 , max_candidate_to_visualize);

categories = [6 15];
%%%%%%
% % ncategories is the number of categories we have trained on
% % this function is located in ros_ws/src/apc
[mask, det2cat, det2scores] = concat_sds_data({imname}, sptextdir, ...
    regspimgdir, chosen, chosenscores, categories, refinedoutdir);
%%%%%%

close all;

%mask = uint8(zeros(480,640));
%det2cat = [6];
%det2scores = [-1];
save('recognition_results.mat','mask','det2cat','det2scores')

% exit
