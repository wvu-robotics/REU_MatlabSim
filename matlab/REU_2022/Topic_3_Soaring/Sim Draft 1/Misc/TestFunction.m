function testResult = TestFunction(testResult)
    if(mod(testResult,2) == 1)
        testResult = testResult * 2;
    end
end