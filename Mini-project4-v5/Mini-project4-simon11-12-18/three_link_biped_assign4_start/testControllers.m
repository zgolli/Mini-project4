function testControllers(Solutions, indices)
    for i=1:length(indices)
        animate(Solutions(i));
    end
end