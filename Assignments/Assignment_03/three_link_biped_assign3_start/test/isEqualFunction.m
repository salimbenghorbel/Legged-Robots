function equal = isEqualFunction(struct1, struct2, threshold)
    equal = true;
    for i = 1:length(struct1.Y)
        for j = 1:length(struct1.T{i})
        %%if (norm(struct1.T{i} - struct2.T{i}) > threshold)
            if (abs(struct1.T{i}(j) - struct2.T{i}(j)) > threshold)
                equal = false;
            end
        end
        
        if (abs(struct1.TE{i} - struct2.TE{i}) > threshold)
            equal = false;
        end
        
        size_struct = size(struct1.Y{i});
        for k = 1:size_struct(1)
            for l = 1:size_struct(2)
                if (abs(struct1.Y{i}(k,l) - struct2.Y{i}(k,l)) > threshold)
                    equal = false;
                end
            end
        end
        
        for j = 1:length(struct1.YE{i})
            if (abs(struct1.YE{i}(j) - struct2.YE{i}(j)) > threshold)
                equal = false;
            end
        end
    end
end