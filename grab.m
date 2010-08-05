function grab(folder,username)
    if nargin > 1
        system(sprintf('kinit %s@CSAIL.MIT.EDU',username));
    else
        system('kinit');
    end
    system('aklog');
    system(sprintf('cp -r /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s .',folder));
end