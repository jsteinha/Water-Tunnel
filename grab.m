function grab(folder)
    system('kinit');
    system('aklog');
    system(sprintf('cp -r /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s .',folder));
end