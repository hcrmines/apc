function [mask, det2cat, det2scores] = concat_sds_data(imnames, sptextdir, regspimgdir, chosen, chosenscores, ncategories, refinnedir)

% Mask image, sized like Kinect
% Each pixel is marked with the integer of the detection it belongs to
mask = zeros(480, 640);
% Each index is a detection, element is category number
det2cat = [];
% Each index is a detection, element is the score
det2scores = [];
ndet = 0;

% Iterate through categories
for i_c = 1 : ncategories
    % Pull results for this category
	c = chosen{i_c};
    cscr = chosenscores{i_c};

    % Sort detections for this category by score
    imids = [];
    scrs = [];
    detids = [];
    for i = 1 : numel(c)
        imids = [imids; i*ones(numel(c{i}), 1)];
        detids = [detids; [1:numel(c{i})]'];
        scrs = [scrs; cscr{i}(:)];
    end
    [s1, i1] = sort(scrs, 'descend');

    % Iterate through detections
    for i_d = 1 : numel(i1)
        k = i1(i_d);
        imid = imids(k);
        detid = detids(k);
        scr = scrs(k);
    
        % Pull region info
        [sp, reg2sp] = read_sprep(fullfile(sptextdir, [imnames{imid} '.txt']), fullfile(regspimdir, [imnames{imid} '.png']));
        reg2sp = reg2sp(:, c{imid}(detid));
        if(exist('refineddir', 'var'))
            tmp = load(fullfile(refineddir, int2str(categid), [imnames{imid} '.mat']));
            reg2sp = tmp.newreg2sp(:, detid);
        end

        % Create mask
        m = reg2sp(sp);
        
        % Add detection to data structures
        ndet = ndet + 1;
        det2cat(ndet) = i_c;        % category #
        det2score(ndet) = scr;      % score

        % Assuming detected region is 1, all else 0, set mask
        mask(m == 1) = ndet;
    end
end

% Our first detection # is 1, so fill in the zeroth index with null
det2cat = [-1 det2cat];
det2scores = [-1 det2scores];

end
