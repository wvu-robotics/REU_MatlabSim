function testResult = TestFunction()
    A.x = 1;
    A.y = 2;
    parfor i=1:4
        B = struct();
        for j=1:5
            letter = char(64+j);
            B.(letter)=j;
        end
        fprintf("Fieldnames B: ");
        f = fieldnames(B);
        fprintf("%s, ",f);
    end
end