clc; clear all; close all;
%% Define Vehicle Position
vehicle.x=0;vehicle.y=0;
%% Define Radius of Detection
RoD=20;%meters
%% Define Waypoint Spacing
wpSpace=0.5;%meters
%% Define Bounding Points
point1.x=10.0; point1.y=10.0;
point2.x=400.0; point2.y=10.0;
point3.x=400.0; point3.y=100.0;
point4.x=10.0; point4.y=100.0;
%% Use points to get bounding lines
%point 1 to 2
lineA=headingDist(point1,point2,wpSpace);
waypointsA=setWaypoints(lineA);
%point 2 to 3
lineB=headingDist(point2,point3,wpSpace);
waypointsB=setWaypoints(lineB);
%point 3 to 4
lineC=headingDist(point3,point4,wpSpace);
waypointsC=setWaypoints(lineC);
%point 4 to 1
lineD=headingDist(point4,point1,wpSpace);
waypointsD=setWaypoints(lineD);
%% Get Candidate Commence Search Points
CCSP.x1=point1.x;CCSP.y1=point1.y+RoD;CCSP.psi1=lineA.heading;
CCSP.x2=point2.x;CCSP.y2=point2.y+RoD;CCSP.psi2=lineA.heading+pi;
CCSP.x3=point3.x;CCSP.y3=point3.y-RoD;CCSP.psi3=lineC.heading;
CCSP.x4=point4.x;CCSP.y4=point4.y-RoD;CCSP.psi4=lineC.heading+pi;
%% Select CSP
CSP=selectCSP(CCSP,vehicle);
%% Now we use RoD to generate parallel line
thePoints=[];
anchorPoint.x=CSP.x;
anchorPoint.y=CSP.y;
%select where to start in enumerated trajectory
if(CSP.choice==1)
    lineNum=1;
elseif(CSP.choice==2)
    lineNum=3;
elseif(CSP.choice==3)
    lineNum=3;
elseif(CSP.choice==4)
    lineNum=1;
else
    print("error");
end
figure()
hold on
%vehicle point
plot(vehicle.x, vehicle.y, 'b-o', 'MarkerSize',12)
%bounding points
plot(point1.x, point1.y, 'r-x', 'MarkerSize',12)
plot([waypointsA.points.x],[waypointsA.points.y], 'b-*', 'MarkerSize',12);
plot(point2.x, point2.y, 'r-x', 'MarkerSize',12)
plot([waypointsB.points.x],[waypointsB.points.y], 'b-*', 'MarkerSize',12);
plot(point3.x, point3.y, 'r-x', 'MarkerSize',12)
plot([waypointsC.points.x],[waypointsC.points.y], 'b-*', 'MarkerSize',12);
plot(point4.x, point4.y, 'r-x', 'MarkerSize',12)
plot([waypointsD.points.x],[waypointsD.points.y], 'b-*', 'MarkerSize',12);

fullSegment=[];
%South to North Case
if(CSP.choice==1||CSP.choice==2)
    while(anchorPoint.y<max([point3.y point4.y]))
        %we
        if(lineNum==1)
            endPoint.x=lineB.start.x-RoD;
            endPoint.y=anchorPoint.y;
        %sn
        elseif(lineNum==2)
            endPoint.x=anchorPoint.x;
            endPoint.y=anchorPoint.y+RoD;
        %ew
        elseif(lineNum==3)
            endPoint.x=lineD.start.x+RoD;
            endPoint.y=anchorPoint.y;
        %sn
        elseif(lineNum==4)
            endPoint.x=anchorPoint.x;
            endPoint.y=anchorPoint.y+RoD;
        else
            print("error");
        end
        segment=genSegment(anchorPoint, endPoint, wpSpace);
        anchorPoint.x=segment.waypoints(end).x;
        anchorPoint.y=segment.waypoints(end).y;
        lineNum=lineNum+1;
        if(lineNum==5)
            lineNum=1;
        end
        fullSegment=[fullSegment,segment.waypoints];
        plot([segment.waypoints.x],[segment.waypoints.y], 'g-o', 'MarkerSize',12);
    end
%end S to N case
%begin N to S case
elseif(CSP.choice==3||CSP.choice==4)
    while(anchorPoint.y>min([point1.y point2.y]))
        %we
        if(lineNum==1)
            endPoint.x=lineB.start.x-RoD;
            endPoint.y=anchorPoint.y;
        %ns
        elseif(lineNum==2)
            endPoint.x=anchorPoint.x;
            endPoint.y=anchorPoint.y-RoD;
        %ew
        elseif(lineNum==3)
            endPoint.x=lineD.start.x+RoD;
            endPoint.y=anchorPoint.y;
        %ns
        elseif(lineNum==4)
            endPoint.x=anchorPoint.x;
            endPoint.y=anchorPoint.y-RoD;
        else
            print("error");
        end
        segment=genSegment(anchorPoint, endPoint, wpSpace);
        anchorPoint.x=segment.waypoints(end).x;
        anchorPoint.y=segment.waypoints(end).y;
        lineNum=lineNum+1;
        if(lineNum==5)
            lineNum=1;
        end
        fullSegment=[fullSegment,segment.waypoints];
        plot([segment.waypoints.x],[segment.waypoints.y], 'g-o', 'MarkerSize',12);
    end
%end N to S case
end%end traj gen loops
properFullSegment.time=linspace(1,length(fullSegment),length(fullSegment))';
properFullSegment.signals.values(:,1)=cell2mat({fullSegment(:).x}');
properFullSegment.signals.values(:,2)=cell2mat({fullSegment(:).y}');
properFullSegment.signals.values(:,3)=cell2mat({fullSegment(:).psi}');
properFullSegment.signals.dimensions=3;
hold off
%% Functions
%define heading and distance
function line = headingDist(pointA,pointB,wpSpace)
    line.heading=atan2((pointB.y-pointA.y),(pointB.x-pointA.x));
    line.distance=sqrt((pointB.y-pointA.y)^2+(pointB.x-pointA.x)^2);
    line.start=pointA;
    line.finish=pointB;
    line.numPoints=floor((line.distance)/wpSpace);
    line.wpSpace=wpSpace;
end
%define bounded line
function waypoints = setWaypoints(line)
    if(atan2((line.finish.y-line.start.y),(line.finish.x-line.start.x))==pi/2)
        for i=1:line.numPoints
            waypoints.points(i).x=line.start.x;
            waypoints.points(i).y=line.start.y+i*line.wpSpace;
        end
    elseif(atan2((line.finish.y-line.start.y),(line.finish.x-line.start.x))==-pi/2)
        for i=1:line.numPoints
            waypoints.points(i).x=line.start.x;
            waypoints.points(i).y=line.start.y-i*line.wpSpace;
        end
    elseif(line.heading>pi/2||line.heading<-pi/2)
        for i=1:line.numPoints
            waypoints.points(i).x=line.start.x-i*line.wpSpace*cos(tan(line.heading));
            waypoints.points(i).y=tan(line.heading)*(waypoints.points(i).x-line.start.x)+line.start.y;
        end
    else
        for i=1:line.numPoints
            waypoints.points(i).x=line.start.x+i*line.wpSpace*cos(tan(line.heading));
            waypoints.points(i).y=tan(line.heading)*(waypoints.points(i).x-line.start.x)+line.start.y;
        end
    end
end
%define select CSP
function selectedCSP=selectCSP(CCSP,vehicle)
    theMin=min([sqrt((CCSP.x1-vehicle.x)^2+(CCSP.y1-vehicle.y)^2) ...
                sqrt((CCSP.x2-vehicle.x)^2+(CCSP.y2-vehicle.y)^2) ...
                sqrt((CCSP.x3-vehicle.x)^2+(CCSP.y3-vehicle.y)^2) ... 
                sqrt((CCSP.x4-vehicle.x)^2+(CCSP.y4-vehicle.y)^2)]);
    if(sqrt((CCSP.x1-vehicle.x)^2+(CCSP.y1-vehicle.y)^2)==theMin)
        selectedCSP.x=CCSP.x1;
        selectedCSP.y=CCSP.y1;
        selectedCSP.psi=CCSP.psi1;
        selectedCSP.choice=1;
    elseif(sqrt((CCSP.x2-vehicle.x)^2+(CCSP.y2-vehicle.y)^2)==theMin)
        selectedCSP.x=CCSP.x2;
        selectedCSP.y=CCSP.y2;
        selectedCSP.psi=CCSP.psi2;
        selectedCSP.choice=2;
    elseif(sqrt((CCSP.x3-vehicle.x)^2+(CCSP.y3-vehicle.y)^2)==theMin)
        selectedCSP.x=CCSP.x3;
        selectedCSP.y=CCSP.y3;
        selectedCSP.psi=CCSP.psi3;
        selectedCSP.choice=3;
    elseif(sqrt((CCSP.x4-vehicle.x)^2+(CCSP.y4-vehicle.y)^2)==theMin)
        selectedCSP.x=CCSP.x4;
        selectedCSP.y=CCSP.y4;
        selectedCSP.psi=CCSP.psi4;
        selectedCSP.choice=4;
    else
        print('Error');
        selectedCSP.x=-1;
        selectedCSP.y=-1;
        selectedCSP.psi=-1;
    end
end
%generate trajectory segment by segment
function segment=genSegment(anchorPoint, endPoint, wpSpace)
    %%get num points for leg
    distance=sqrt((endPoint.y-anchorPoint.y)^2+(endPoint.x-anchorPoint.x)^2);
    numPoints=floor(distance/wpSpace);
    %%check orientation
    %we
    if(endPoint.x>anchorPoint.x)
        for i=1:numPoints
            segment.waypoints(i).x=anchorPoint.x+i*wpSpace;
            segment.waypoints(i).y=anchorPoint.y;
            segment.waypoints(i).psi=pi/2;
        end
    end
    %sn
    if(endPoint.y>anchorPoint.y)
        for i=1:numPoints
            segment.waypoints(i).x=anchorPoint.x;
            segment.waypoints(i).y=anchorPoint.y+i*wpSpace;
            segment.waypoints(i).psi=0;           
        end
    end
    %ew
    if(endPoint.x<anchorPoint.x)
        for i=1:numPoints
            segment.waypoints(i).x=anchorPoint.x-i*wpSpace;
            segment.waypoints(i).y=anchorPoint.y;
            segment.waypoints(i).psi=-pi/2;        
        end
    end
    %ns
    if(endPoint.y<anchorPoint.y)
        for i=1:numPoints
            segment.waypoints(i).x=anchorPoint.x;
            segment.waypoints(i).y=anchorPoint.y-i*wpSpace;
            segment.waypoints(i).psi=pi;            
        end
    end
end
