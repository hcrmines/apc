function [cen_x, cen_y, ang_ma, ang_mi] = analyze_mask(mask, ndet)

cen_x = zeros(1, ndet);
cen_y = zeros(1, ndet);
ang_ma = zeros(1, ndet);
ang_mi = zeros(1, ndet);

for i = 1 : ndet
    m = mask == i;
    [L, n] = bwlabel(m);
    if n == 0
        cen_x(i) = -1;
        cen_y(i) = -1;
        ang_ma(i) = -1;
        ang_mi(i) = -1;
        continue;
    end
    
    objs = regionprops(L, 'area', 'centroid', 'orientation');
    
    areas = cat(1, objs(:).Area);
    [~, indices] = sort(areas);
    R = objs(indices(n));
    
    cen_x = R.Centroid(1);
    cen_y = R.Centorid(2);
    
    if R.Orientation > 0
        a_mi = R.Orientation - 90;
    else
        a_mi = R.Orientation + 90;
    end
    ang_ma(i) = deg2rad(R.Orientation);
    ang_mi(i) = deg2rad(a_mi); 
end

cen_x = [-1 cen_x];
cen_y = [-1 cen_y];
ang_ma = [-1 ang_ma];
ang_mi = [-1 ang_mi];

end