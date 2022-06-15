function TestFunction(message)
    fprintf("Hi! The message is %s\n",message);
    assignin('base','baseVar',1);
    assignin('caller','callerVar',2);
    baseVar
    callerVar
end