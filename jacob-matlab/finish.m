function results=finish(job)
    waitForState(job); results = getAllOutputArguments(job);
end
