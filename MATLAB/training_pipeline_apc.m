
% clc
% clear 
% close all


% assign the directory of the training data
% it must contain three subdirectories:
% 1. img: shows orignial image
% 2. cls: ground truth of categories
% 3. inst: ground truth of instances
datadir='/home/hcrws1/Documents/Toolbox/Data';


imgdir = fullfile(datadir,'img');

% sbddir (gtdir) stores ground truth. It is required to contain two
% subdirectories: cls and inst, saving ground truth of catigories and
% instances respectively. 
% In our data strorage structure, sbddir is the datadir, since all ground
% truth is stored under the datadir.
sbddir = datadir;


% for training, all cache data of objects are stored under cachedir of the training data directory
cachedir=fullfile(datadir, 'cachedir');
if(~exist(cachedir, 'file')) mkdir(cachedir); end

mcgdir=fullfile(cachedir, 'mcg'); % filled by MCG algorithm
ovoutdir=fullfile(cachedir, 'overlaps');
sptextdir=fullfile(cachedir, 'sptextdir');
regspimgdir=fullfile(cachedir, 'reg2spimgdir');
featdir=fullfile(cachedir, 'featdir');


imname=getFilenames(imgdir,'.jpg');
ilength = length (imname);
for i=1:ilength
    [~,name,~] = fileparts(imname{i});
        imname{i} = name;
end


% ---------------------------------------------------
% COMPUTE FEATURES

% Given: imnames, imgdir and sbddir
% setup_svm_training_apc fills the directories: mcgdir, ovoutdir, sptextdir, regspimagdir, and featdir 
% Note: sbddir is the directory storing ground truth. It must contains two
% subdirectories: cls (storing category gt) and inst (storing instance gt).
% The function load_gt (setup_svm_training_apc -> preprocess_mcg_candidates -> load_gt)
% is used to load both ground truth.
% -----------------------------------------------------

region_meta_info = setup_svm_training_apc(imname, imgdir, mcgdir, ovoutdir, sptextdir, regspimgdir, featdir, sbddir);

save('ws11objfst200_20150522');



% ---------------------------------------------------
% TRAIN SVM
% ---------------------------------------------------

% load('ws2obj20150518');

run_name = 'test11objfast200';
models=train_svms_apc(imname, featdir, region_meta_info, cachedir, run_name);
save('apc_pretrained_models/svms/APC.mat', 'models');

fprintf('\nSVM models are trained. Saving to apc_pretrained_models/svms/APC_F200.mat. Done!\n');

save('ws11objfst200_20150522_svm_trained');


% % --------------------------------------------------
% % TO DO (low priority): check how refinement can improve performance
% %
% % TRAIN REFINED SVM MODELS
% % Each time only one refined model is trained
% % NOT A REQUIRED STEP
% % --------------------------------------------------
% 
% load('ws2obj20150518_svm_trained');
% obj_to_refine = OBJ_OF_INTEREST_APC;
% 
% % TO DO: parameter to be tuned:
% Wsz = [10,10];
% categid = obj_to_refine(1);
% refinement_model = train_refiner_apc(imname, region_meta_info, featdir, sptextdir, regspimgdir, sbddir, Wsz, categid);


