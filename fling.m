function fling(folder)
    system('kinit');
    if system(sprintf('mkdir /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s',folder))
        return;
    else
        system(sprintf('cp *.dat /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s/',folder));
    end
end