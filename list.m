function list(username)
    if nargin > 0
        system(sprintf('kinit %s@CSAIL.MIT.EDU',username));
    else
        system('kinit');
    end
    system('aklog');
    system('ls /afs/csail.mit.edu/group/locomotion/data/water_tunnel');
end