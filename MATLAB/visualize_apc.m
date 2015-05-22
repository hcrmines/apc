function visualize_apc(imnames, imgdir, sptextdir, regspimgdir, topchosen, topscores, categid, max_candidate_to_visualize)
% original version: function visualize_apc(imnames, imgdir, sptextdir, regspimgdir, topchosen, topscores, categid, refineddir)

obj_of_interest = OBJ_OF_INTEREST_APC;
cateid_local = find(obj_of_interest == categid);

%only keep this category
topchosen=topchosen{cateid_local};
topscores=topscores{cateid_local};


%sort detections in order of decreasing scores
imids=[];
scrs=[];
detids=[];
for i=1:numel(topchosen)
    imids=[imids; i*ones(numel(topchosen{i}),1)];
    detids=[detids; [1:numel(topchosen{i})]'];
    scrs=[scrs; topscores{i}(:)];
end
[s1, i1]=sort(scrs, 'descend');

max_candidate_to_visualize = min(max_candidate_to_visualize, numel(i1));

%go down the detections and display them
for i=1:max_candidate_to_visualize
    k=i1(i);
    imid=imids(k);
    detid=detids(k);
    scr=scrs(k);

    %read sprep and maybe refined regions
    [sp, reg2sp]=read_sprep(fullfile(sptextdir, [imnames{imid} '.txt']), fullfile(regspimgdir, [imnames{imid} '.png']));
    reg2sp=reg2sp(:,topchosen{imid}(detid));
    
%     % do not do refinement
%     if(exist('refineddir', 'var'))
%         tmp=load(fullfile(refineddir, int2str(categid), [imnames{imid} '.mat']));
%         reg2sp=tmp.newreg2sp(:,detid);
%     end
    
    %the mask
    m1=reg2sp(sp);

    img=imread(fullfile(imgdir, [imnames{imid} '.jpg']));

    %show
    imagesc(color_seg(double(m1), img)); axis equal;
    title(sprintf('Image %d, detid %d, score %f', imid, detid, scr));
    pause;
end

