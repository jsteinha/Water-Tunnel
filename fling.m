function fling(folder,option)
    system('kinit');
    system('aklog');
    if system(sprintf('mkdir /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s',folder))
        if nargin < 2 || option ~= 'f'
            return;
        else
            display('forcing copy to remote directory...')
        end
    end
    display('writing to remote directory...')
    system(sprintf('cp *.dat /afs/csail.mit.edu/group/locomotion/data/water_tunnel/%s/',folder));
    display('done!')
end