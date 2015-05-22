function imname = getFilenames(imgdir, imgext)

    list = dir(imgdir);
    imname = {list.name};
    ilength = length(imname);
    
    j=0;
    for i=1:ilength
        [~,name,ext]=fileparts(imname{i});
        if(strcmp(imgext,ext))
            j=j+1;
            imname{j}=name;
        end
    end

    imname = imname(1:j);
end