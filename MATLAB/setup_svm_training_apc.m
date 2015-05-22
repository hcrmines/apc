function region_meta_info = setup_svm_training_apc(imnames, imgdir, mcgdir, ovoutdir, sptextdir, regspimgdir, featdir, sbddir)
% The only modification is saving region_meta_info to disk

%%%%%%
%MCG
%%%%%%

% given imnames,
% this function fills the directory: mcgdir

fprintf('Computing MCG. Note that this can be parallelized\n');

if(~exist(mcgdir, 'file')) mkdir(mcgdir); end

for i=1:numel(imnames)
    fprintf('MCG : %d/%d\n', i, numel(imnames));
    imname=imnames{i};
    imgfile=fullfile(imgdir, [imname '.jpg']);
    img=imread(imgfile);
    candidates_file=fullfile(mcgdir,[imname '.mat']);
    if(~exist(candidates_file))
    
        candidates=im2mcg(img, 'fast');
%         candidates=im2mcg(img, 'accurate');
        save(candidates_file, 'candidates');
    end
end

%%%%%%%%%%%
%Preprocess
%%%%%%%%%%%

% given imnames, mcgdir, and sbddir (gtdir)
% this function fills: ovoutdir (overlaps), sptextdir (sptextdir), and regspimgdir (reg2spimgdir)
fprintf('Preprocessing candidates.\n');
[region_meta_info]=preprocess_mcg_candidates(imnames, mcgdir, sbddir, ovoutdir, sptextdir, regspimgdir, 200);


% Hao edited: region_meta_info is needed by training, better save it.
% it is saved to the featdir
save(fullfile(featdir,'region_meta_info'), 'region_meta_info');

%%%%%%%%%%%%%
%Load network
%%%%%%%%%%%%%
model_def_file='prototxts/pinetwork_extract_fc7.prototxt';
model_file='sds_pretrained_models/nets/C';
assert(exist(model_def_file, 'file')>0);
assert(exist(model_file, 'file')>0);
rcnn_model=rcnn_create_model(model_def_file,model_file);
rcnn_model=rcnn_load_model(rcnn_model);

%%%%%%%%%%%%%%%%%
%Extract features
%%%%%%%%%%%%%%%%%
fprintf('Saving features..\n');
save_features(imnames, imgdir, sptextdir, regspimgdir, featdir, rcnn_model);

fprintf('Saving features done!\n');


