function job=schedule(fnc,name)
    cmd=sprintf('mkdir -p %s',name);
    system(cmd);
    cmd=sprintf('cp *.m *.mex* %s/',name);
    system(cmd);
    cd(name);

    jm = findResource('scheduler','configuration','generic');
    set(jm,'configuration','generic');
    job = createJob(jm);

    for i=1:10
        createTask(job, fnc, 4, {i}); end;
    submit(job);

    cd ..
end
