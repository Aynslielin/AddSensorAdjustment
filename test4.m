%% Add Sensors to Driving Scenario and Get Target Poses Using Sensor Input
% In this example, you will add sensors to a driving scenario using the |addSensors| 
% function and get ground-truth target poses based on the individual sensor inputs. 
% Then, you process the ground-truth target poses into detections and visualize 
% them. 
%% Set Up Driving Scenario and Bird's-Eye-Plot
% Create a driving scenario with an ego vehicle and two target vehicles. One 
% target vehicle is in the front and the other is to the left of the ego-vehicle.

[scenario, egovehicle] = helperCreateDrivingScenario;
%% 
% Configure a vision sensor to be mounted at the front of the ego vehicle.

visionSensor = visionDetectionGenerator(SensorIndex=1,SensorLocation=[4.0 0],Height=1.1,Pitch=1.0,DetectorOutput="Objects only");
%% 
% Configure an ultrasonic sensor to be mounted at the left side of the ego-vehicle.

rightUltrasonic = ultrasonicDetectionGenerator(SensorIndex=2,MountingLocation=[1 0 0.2],MountingAngles=[-90 0 0],FieldOfView=[70 35],UpdateRate=100);
%% 
% Create a bird's-eye-plot to visualize the driving scenario.

[detPlotter,lmPlotter,olPlotter,lulrdPlotter,luldetPlotter,bepAxes] = helperCreateBEP(visionSensor,rightUltrasonic);
%% Add Sensors and Simulate Driving Scenario
% Add both vision and ultrasonic sensors to the driving scenario using the |addSensors| 
% function. You can add sensors to any vehicle in the driving scenario using the 
% |addSensors| function by specifying the actor ID of the desired vehicle. In 
% this example, specify the ego-vehicle actor ID. 

addSensors(scenario,{visionSensor,rightUltrasonic},egovehicle.ActorID);
%% 
% Simulate the driving scenario. Note that you get separate target poses based 
% on individual sensors by specifying their respective sensor IDs as inputs to 
% the |targetPoses| function. This syntax returns the ground-truth poses of targets 
% only within the range of the specified sensor. You then pass the ground-truth 
% poses to their respective sensor models to generate detections and visualize 
% them.
plot(scenario, 'Centerline', 'on', 'RoadCenters', 'on');
chasePlot(egovehicle,'Centerline','on')

legend(bepAxes,'show')
lookaheadDistance = 0:0.5:60;
while advance(scenario)
    
    lb = laneBoundaries(egovehicle,'XDistance',lookaheadDistance,'LocationType','inner');
    [lmv,lmf] = laneMarkingVertices(egovehicle);

    % Get ground-truth poses of targets in the range of vision and ultrasonic sensors separately  
    tposeVision = targetPoses(scenario,visionSensor.SensorIndex);
    tposeUltrasonic = targetPoses(scenario,rightUltrasonic.SensorIndex);
    
    % Obtain detections based on targets only in range
    [obdets,obValid] = visionSensor(tposeVision,scenario.SimulationTime);
    [lobdets,lobValid] = rightUltrasonic(tposeUltrasonic,scenario.SimulationTime);

    helperPlotBEPVision(egovehicle,lmv,lmf,obdets,obValid,detPlotter,lmPlotter,olPlotter)
    helperPlotBEPUltrasonic(lobdets,lobValid,rightUltrasonic,lulrdPlotter,luldetPlotter)
end
%% Helper Functions
% |helperCreateDrivingScenario| creates a driving scenario by specifying the 
% road and vehicle properties.

function [scenario, egovehicle] = helperCreateDrivingScenario
    scenario = drivingScenario;
    roadCenters1 = [-120 30 10;-60 0 10;0 0 10; 60 0 10; 120 30 10; 180 60 10];
    roadCenters2 = [-180 -60 10; -60 -30 10; 0 0 10];
    roadCenters3 = [60 0 10; 120 0 10; 180 -30 10];
    lspc1 = lanespec(3);
    lspc2 = lanespec(2);
    road(scenario,roadCenters1,Lanes=lspc1);
    road(scenario,roadCenters2,Lanes=lspc2);
    road(scenario,roadCenters3,Lanes=lspc2);

    % Create an ego vehicle that travels in the center lane at a velocity of 30 m/s.
    egovehicle = vehicle(scenario,ClassID=1);
    egopath = [1.5 0 10; 60 0 10; 90 0 10;  120 -0.5 10; 180 -30 10];
    egospeed = 30;
    smoothTrajectory(egovehicle,egopath,egospeed);

    % Add a target vehicle that travels ahead of the ego vehicle at 35 m/s in the right lane, and changes lanes close to the ego vehicle.    
    ftargetcar = vehicle(scenario,ClassID=1);
    ftargetpath = [8 2 10; 60 -3.2 10; 120 33 10];
    ftargetspeed = 35;
    smoothTrajectory(ftargetcar,ftargetpath,ftargetspeed);

    % Add a second target vehicle that travels in the left lane at 30m/s.    
    %{
    ltargetcar = vehicle(scenario,ClassID=1);
    ltargetpath = [-5.0 3.5 10; 60 3.5 10; 111 28.5 10];
    ltargetspeed = 30;
    smoothTrajectory(ltargetcar,ltargetpath,ltargetspeed);
    %}
    % Add a third target vehicle that travels in the left lane at 40m/s.    
    rtargetcar = vehicle(scenario,ClassID=1);
    targetpath3 = [-31.6 -18.5 10; -10 -7 10; 0 -2.5 10; 30 -3 10; 90 0 10;  120 -0.5 10; 180 -30 10];
    targetspeed3 = 50;
    smoothTrajectory(rtargetcar,targetpath3,targetspeed3);
end
%% 
% |helperCreateBEP| creates a bird's-eye-plot for visualing the driving scenario 
% simulation.

function [detPlotter, lmPlotter, olPlotter, lulrdPlotter,luldetPlotter,bepAxes] = helperCreateBEP(visionSensor,leftUltrasonic)
    figureName = "BEP";
    Figure = findobj('Type','Figure',Name=figureName);
    if isempty(Figure)
        screenSize = double(get(groot,'ScreenSize'));
        Figure = figure(Name=figureName);
        Figure.Position = [screenSize(3)*0.17 screenSize(4)*0.15 screenSize(3)*0.4 screenSize(4)*0.6];
        Figure.NumberTitle = 'off';
        Figure.MenuBar = 'none';
        Figure.ToolBar = 'none';
    end
    clf(Figure);
    bepAxes = axes(Figure);
    grid(bepAxes,'on');
    legend(bepAxes,'hide');
    
    bep = birdsEyePlot(Parent=bepAxes,XLim=[-20 60],YLim=[-35 35]);
    caPlotterV = coverageAreaPlotter(bep,DisplayName="Vision Coverage area",FaceColor="b");
    caPlotterU = coverageAreaPlotter(bep,DisplayName="Ultrasonic Coverage area",FaceColor="m");
    detPlotter = detectionPlotter(bep,DisplayName="Object detections");
    lmPlotter = laneMarkingPlotter(bep,DisplayName="Lane markings");
    olPlotter = outlinePlotter(bep);
    plotCoverageArea(caPlotterV,visionSensor.SensorLocation,...
        visionSensor.MaxRange,visionSensor.Yaw, ...
        visionSensor.FieldOfView(1));
    plotCoverageArea(caPlotterU,leftUltrasonic.MountingLocation(1:2),...
        leftUltrasonic.DetectionRange(3),leftUltrasonic.MountingAngles(1), ...
        leftUltrasonic.FieldOfView(1));
    
    lulrdPlotter = rangeDetectionPlotter(bep,DisplayName="Left Ultrasonic Ranges",LineStyle="-");
    luldetPlotter = detectionPlotter(bep,DisplayName="Point-On-Target (Left Ultrasonic)",MarkerFaceColor="k");
end

%% 
% |helperPlotBEPVision| plots ego vehicle outlines, lanes and vision detections 
% on the bird's-eye-plot.

function helperPlotBEPVision(egovehicle,lmv,lmf,obdets,obValid,detPlotter, lmPlotter, olPlotter)
    plotLaneMarking(lmPlotter,lmv,lmf)
    
    [objposition,objyaw,objlength,objwidth,objoriginOffset,color] = targetOutlines(egovehicle);
    plotOutline(olPlotter,objposition,objyaw,objlength,objwidth, ...
        OriginOffset=objoriginOffset,Color=color)

    if obValid
        detPos = cellfun(@(d)d.Measurement(1:2),obdets,UniformOutput=false);
        detPos = vertcat(zeros(0,2),cell2mat(detPos')');
        plotDetection(detPlotter,detPos)
    end

end
%% 
% |helperPlotBEPUltrasonic| plots ultrasonic range measurements and points on 
% targets.

function helperPlotBEPUltrasonic(lobdets,lobValid,leftUltrasonic,lulrdPlotter,luldetPlotter)
    
    if ~isempty(lobdets) && lobValid
        lranges = lobdets{1}.Measurement;
        plotRangeDetection(lulrdPlotter,lranges,leftUltrasonic.FieldOfView(1),leftUltrasonic.MountingLocation,leftUltrasonic.MountingAngles)
        plotDetection(luldetPlotter,lobdets{1}.ObjectAttributes{1}.PointOnTarget(1:2)')
    end

end
%% 
% _Copyright 2022 The MathWorks, Inc._