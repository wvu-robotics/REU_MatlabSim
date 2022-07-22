% Utility class: stores helper functions
classdef Utility
    methods (Static)
        %% Calculates Random number In Range (randIR) between low and high
        function value = randIR(low,high) 
            value = rand() * (high-low) + low;
        end
        
        %% Calculates distance if within threshold
        function [Verdict] = isNear(A1, A2, Threshold)
            % A NaN Threshold will tell isNear to ignore the threshold.
            if ~isnan(Threshold)
                Verdict = NaN;
                if abs(A1.position(1) - A2.position(1)) > Threshold 
                    return
                elseif abs(A1.position(2) - A2.position(2)) > Threshold
                    return
                elseif abs(A1.position(3) - A2.position(3)) > Threshold
                    return
                elseif A1.position(1) == A2.position(1) && A1.position(2) == A2.position(2)
                    return
                end
            end
            dist = norm([(A1.position(1)-A2.position(1)), ...
                         (A1.position(2)-A2.position(2)), ...
                         (A1.position(3)-A2.position(3))]);
            if isnan(Threshold) || dist <= Threshold
                Verdict = dist;
            end
        end
        %% Calculates Weighted Centroid
        function [Centroid, distances, diffHeight] = findCentroid(currentAgent, localAgents, SL)
            %% Init
            numLocalAgents = size(localAgents,2);
            Centroid       = [0,0,0];
            distances      = zeros(1,numLocalAgents);
            diffHeight     = distances;
            numLocalAgents = length(distances);
            
            %% Loop
            for i = 1:numLocalAgents
                if ~localAgents(i).isAlive
                    continue;
                end
                distances(i) = norm(currentAgent.position - localAgents(i).savedPosition);
                diffHeight(i) = -currentAgent.position(3) + localAgents(i).savedPosition(3); % negative if above others.
                normHeight = diffHeight(i)/SL.neighborRadius;
                if normHeight < SL.heightIgnore
                    weight = 0;
                else
                    weight = SL.heightPriority * (normHeight - SL.heightIgnore);
                    % if normHeight = 1 and heightIgnore = -1, weight is 2.
                end
                Centroid = Centroid + weight*localAgents(i).savedPosition;
            end
            Centroid = Centroid / numLocalAgents;  
        end

        %% Mid of Min and Max
        function mid = midMinMax(num, min, max)
            mid = min(max(num,min),max);
        end
        %% Render From Data (WIP)
        function video = renderData(fileName)
            allData = load(fileName,"xData","yData","zData","fVelData","VelData","zVelData","bankAngleData","headingData");
            numSteps = allData.SL.totalTime / allData.SL.dt;
        end
        
        %% Convert (Row,Column) to spreadsheet position format: <Column Letter><Row Number>
        function ssPos = findSSPos(rowNumber,colNumber)
            columnLetter = char(64 + colNumber); %char(65) = 'A', char(66) = 'B' ...
            ssPos = sprintf("%s%g",columnLetter,rowNumber);
        end
        
        %% Saves a struct in the given file name (work around for par-for save conflict?)
        function parSave(fileName,structToSave)
            save(fileName,'-struct','structToSave');
        end
        
        %% Normal eval function (work around for par-for eval conflict?)
        function output = parTryEval(param)
            try
                output = eval(param);
            catch
                output = param;
            end
        end
        
        %% Generae output structs from .mat files
        function generateOutputStruct(outputStructCode, matFilesFolder,changedVariables,outputVariables)
            %{
            outputStructCode = "E:\WVU_REU\7-20-22\SimBatch_20220720110652\%s.mat";
            matFilesFolder = "E:\WVU_REU\7-20-22\SimBatch_20220720110652\MatFiles";
            changedVariables=["rngSeed","cohesion","heightFactorPower","cohesionAscensionIgnore","ascensionFactorPower","separation","alignment","cohesionAscensionMax"];
            outputVariables = ["simNumber";"rngSeed";"timeStart";"timeEnd";"surviving";"collisionDeaths";"groundDeaths";"flightTime";"heightScore";"explorationPercent";"thermalUseScore";"finalHeightMax";"finalHeightMin";"finalHeightAvg"];
            %}
            
            
            fprintf("Generating output struct... ");
            % Read mat files
            fileSearch = sprintf("%s/*.mat",matFilesFolder);
            dirData = dir(fileSearch);
            numFiles = size(dirData,1);
            fileNames = {dirData.name};
            for i=numFiles:-1:1
                try
                    fileName = sprintf('%s/%s',matFilesFolder,fileNames{i});
                    bigOutputData(i) = load(fileName);
                catch
                    fprintf("Load failed.\n");
                end
            end
            
            % Write changedVariables to .mat file
            if(~isempty(changedVariables))
                numVar = length(changedVariables);
                for varIndex = 1:numVar
                    varLabel = changedVariables(varIndex);
                    cellsChangedVariableValues = cell(1,numFiles);
                    for sim=1:numFiles
                        cellsChangedVariableValues{1,sim} = bigOutputData(sim).SL.(changedVariables(varIndex));
                    end
                    structChangedVariables.(varLabel) = [cellsChangedVariableValues{1,:}];
                end
                structChangedVariablesName = sprintf(outputStructCode,"StructChangedVariables");
                save(structChangedVariablesName,'-struct','structChangedVariables');
            end
            
            % Write outputVariables to Excel sheet
            if(~isempty(outputVariables))
                numVar = length(outputVariables);
                for varIndex = 1:numVar
                    varLabel = outputVariables(varIndex);
                    cellsOutputVariableValues = cell(1,numFiles);
                    for sim=1:numFiles
                        cellsOutputVariableValues{1,sim} = bigOutputData(sim).(outputVariables(varIndex));
                    end
                    structOutputVariables.(varLabel)=[cellsOutputVariableValues{1,:}];
                end
                structOutputVariablesName = sprintf(outputStructCode,"StructOutputVariables");
                save(structOutputVariablesName,'-struct','structOutputVariables');
            end
            fprintf("Done!\n");
        end
        
        %% Generate output excel sheet from .mat files
        function generateOutputExcelSheet(outputExcelName,matFilesFolder,inputCellsToCopy,changedVariables,outputVariables)
            fprintf("Generating output Excel sheet... ");
            % Read mat files
            fileSearch = sprintf("%s/*.mat",matFilesFolder);
            dirData = dir(fileSearch);
            numFiles = size(dirData,1);
            fileNames = {dirData.name};
            for i=numFiles:-1:1
                fileName = sprintf('%s/%s',matFilesFolder,fileNames{i});
                bigOutputData(i) = load(fileName);
            end
            
            % Setup output Excel sheet
            sheetNum = 1;
            varLabelColumn = 2;
            startingColumn = 3;
            % Copy inputCellsToCopy to Excel sheet
            writecell(inputCellsToCopy,outputExcelName,'Sheet',sheetNum,'Range','A1','AutoFitWidth',0);
            outputRow = size(inputCellsToCopy,1) + 5;
            % Write 'OUTPUT' to Excel sheet
            writecell({'OUTPUT'},outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,1),'AutoFitWidth',0);
            outputRow = outputRow + 1;
            
            % Write changedVariables to Excel sheet
            if(~isempty(changedVariables))
                writecell({'Changed Variables'},outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,1),'AutoFitWidth',0);
                numVar = length(changedVariables);
                cellsChangedVariableLabels = cell(numVar,1);
                cellsChangedVariableValues = cell(numVar,numFiles);
                for varIndex = 1:numVar
                    cellsChangedVariableLabels{varIndex,1} = changedVariables(varIndex);
                    for sim=1:numFiles
                        cellsChangedVariableValues{varIndex,sim} = bigOutputData(sim).SL.(changedVariables(varIndex));
                    end
                end
                writecell(cellsChangedVariableLabels,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,varLabelColumn),'AutoFitWidth',0);
                writecell(cellsChangedVariableValues,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,startingColumn),'AutoFitWidth',0);

                outputRow = outputRow + numVar + 1;
            end
            
            % Write outputVariables to Excel sheet
            if(~isempty(outputVariables))
                writecell({'Output Variables'},outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,1),'AutoFitWidth',0);
                numVar = length(outputVariables);
                cellsOutputVariableLabels = cell(numVar,1);
                cellsOutputVariableValues = cell(numVar,numFiles);
                for varIndex = 1:numVar
                    cellsOutputVariableLabels{varIndex,1} = outputVariables(varIndex);
                    for sim=1:numFiles
                        cellsOutputVariableValues{varIndex,sim} = bigOutputData(sim).(outputVariables(varIndex));
                    end
                end
                writecell(cellsOutputVariableLabels,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,varLabelColumn),'AutoFitWidth',0);
                writecell(cellsOutputVariableValues,outputExcelName,'Sheet',sheetNum,'Range',Utility.findSSPos(outputRow,startingColumn),'AutoFitWidth',0);
            end
            fprintf("Done!\n");
        end
        
        %% Send notification to ACP via pushover
        function notifyACP()
            url = "https://api.pushover.net/1/messages.json";
            appToken = "a5q8yhpz3t9c7pzzmrmzczpejcyozy";
            ACPToken = "uti2e2jd1jhtujgbtnaaop95rubtmq";
            message = "Simulation Complete.";
            success = false;
            for i=1:10
                if(success)
                    break
                else
                    try
                        webwrite(url,'token',appToken,'user',ACPToken,'message',message); 
                        success = true;
                    catch
                        success = false;
                    end
                end
            end
        end
        
        %% Clean .mat files of certain parameters
        function cleanMatFiles(matFilesFolder,badParameters)
            fileSearch = sprintf("%s/*.mat",matFilesFolder);
            dirData = dir(fileSearch);
            numFiles = size(dirData,1);
            fileNames = {dirData.name};
            for i=numFiles:-1:1
                if(mod(i,100)==0)
                    fprintf("i: %d\n",i);
                end
                fileName = sprintf('%s/%s',matFilesFolder,fileNames{i});
                fileIn = load(fileName);
                fields = fieldnames(fileIn);
                for fieldIndex = 1:length(fields)
                    field = fields{fieldIndex};
                    if(isempty(find(badParameters == field,1)))
                        try
                            fileOut.(field) = fileIn.(field);
                        catch
                            fprintf("wtf");
                        end
                    end
                end
                save(fileName,'-struct','fileOut');
                %fprintf("Saved %s.\n",fileName);
                %break
            end
        end
        
        %% Combine .mat files with certain parameters
        function combineMatFiles(matFilesFolder,SLparameters)
            %{
            SLparameters = ["cohesion","heightFactorPower","cohesionAscensionIgnore","cohesionAscensionMax","ascensionFactorPower","separation","alignment"];
            SLparameters = ["cohesion","cohesionAscensionIgnore","cohPower","separation","alignment","k"];
            SLparameters = ["cohesion","separation","alignment","cohPower","separationHeightWidth","alignmentHeightWidth"];
            %}
            fileSearch = sprintf("%s/*.mat",matFilesFolder);
            dirData = dir(fileSearch);
            numFiles = size(dirData,1);
            fileNames = {dirData.name};
            firstFileName = sprintf('%s/%s',matFilesFolder,fileNames{1});
            firstFileIn = load(firstFileName);
            fields = fieldnames(firstFileIn);
            fields(find(contains(fields,'SL'))) = [];

            for i=1:length(SLparameters)
                combinedData.(SLparameters(i)) = [];
            end
            for i=1:length(fields)
                combinedData.(fields{i}) = [];
            end
            
            for fileIndex=numFiles:-1:1
                if(mod(fileIndex,100)==0)
                    fprintf("fileIndex: %d\n",fileIndex);
                end
                fileName = sprintf('%s/%s',matFilesFolder,fileNames{fileIndex});
                fileIn = load(fileName);
                for i=1:length(SLparameters)
                    combinedData.(SLparameters(i)) = [combinedData.(SLparameters(i)),fileIn.SL.(SLparameters(i))];
                end
                for i=1:length(fields)
                    combinedData.(fields{i}) = [combinedData.(fields{i}),fileIn.(fields{i})];
                end
            end
            fileName = sprintf('%s/../%s',matFilesFolder,"CombinedData.mat");
            save(fileName,'-struct','combinedData');
        end
        
        function [indepValues,averages] = avgValues(data,indepName,depName)
            indep = data.(indepName);
            dep = data.(depName);
            indepValues = unique(indep);
            indepMap = containers.Map(indepValues,1:length(indepValues));
            sums = cell(1,length(indepValues));
            for i=1:length(indep)
                sumsIndex = indepMap(indep(i));
                sums{sumsIndex} = [sums{sumsIndex},dep(i)];
            end
            averages = zeros(1,length(indepValues));
            for i=1:length(indepValues)
                averages(i) = sum(sums{i})/length(sums{i});
            end
        end
    end
end