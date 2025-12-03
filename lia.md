<!--
author:   David Croft
email:    david.croft@warwick.ac.uk
version:  0.1.0
language: en
narrator: UK English Female

classroom: false

link:  assets/styles.css
import: module_templates/macros.md

@style
.flex-container {
    display: flex;
    flex-wrap: wrap; /* Allows the items to wrap as needed */
    align-items: stretch;
    gap: 20px; /* Adds both horizontal and vertical spacing between items */
}

.flex-child { 
    flex: 1;
    margin-right: 20px; /* Adds space between the columns */
}

@media (max-width: 600px) {
    .flex-child {
        flex: 100%; /* Makes the child divs take up the full width on slim devices */
        margin-right: 0; /* Removes the right margin */
    }
}
@end

@onload
window.codeblocks = [
`%% SECTION 1

clear;
close all;

d = load('01_city_c2s_fcw_10s_Lidar.mat');
pcloud = d.LidarPointCloud;

pc = pcloud(1).ptCloud;

% Set the region of interest.
xBound = 40; % in meters
yBound = 20; % in meters
xlimits = [-xBound, xBound];
ylimits = [-yBound, yBound];
zlimits = pc.ZLimits;

player = pcplayer(xlimits, ylimits, zlimits);

% Crop the point cloud to only contain points within the specified region.
indices = find(pc.Location(:, 2) >= -yBound ...
    & pc.Location(:, 2) <= yBound ...
    & pc.Location(:, 1) >= -xBound ...
    & pc.Location(:, 1) <= xBound);

% Select the subset of points and display them.
pc = select(pc, indices);
view(player, pc);`,

`% Uncomment this if you would like to see the LiDAR pointcloud changing while the vehicle is moving.
for k = 2:length(pcloud)
    pc = pcloud(k).ptCloud;
    
    % Crop the data to ROI.
    indices = find(pc.Location(:, 2) >= -yBound ...
        & pc.Location(:, 2) <= yBound ...
        & pc.Location(:, 1) >= -xBound ...
        & pc.Location(:, 1) <= xBound);
    pc = select(pc, indices);
    
    % Plot the results.
    view(player, pc);

    % delay
    pause(0.1)
end`,

`%% SECTION 2

maxDistance = 0.2; %in metres
referenceVector = [0, 0, 1];
[~, inPlanePointIndices, outliers] = pcfitplane(pc, maxDistance, referenceVector);`, 

`% colourize

labelSize = [pc.Count, 1];
colorLabels = zeros(labelSize, 'single');

% set the colormap for labeling the points
colors = [0 0 1; ... % Blue for unlabeled points; specified as [R G B]
         0 1 0; ... % Green for ground points
         1 0 0; ... % Red for obstacle points
          1 1 1];    % White for ego vehicle
% indices will follow the order of declaration of the rows in color
blueIdx = 0; % the entire point cloud is initially blue
greenIdx = 1;
redIdx = 2;
whiteIdx = 3;

% Label the ground points
colorLabels(inPlanePointIndices) = greenIdx;`,

`% Select the points that are not part of the ground plane.
pcWithoutGround = select(pc, outliers);

% Declare a “Danger Zone” around the LiDAR

sensorLocation = [0 0 0]; % place the Lidar sensor at the center of coordinate system
radius = 10; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

nearPointIndices = outliers(nearIndices) ;

% Label the obstacle points.
colorLabels(nearPointIndices) = redIdx;`,

`% Declare an area to label as the vehicle points
radius = 3; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle = select(pcWithoutGround, nearIndices);

delta = 0.1;
selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
            pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
            pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];

colorLabels(vehiclePointIndices) = whiteIdx;`,

`player1 = pcplayer(xlimits, ylimits, zlimits);
colormap(player1.Axes, colors)

view(player1, pc.Location, colorLabels);
title(player1.Axes, 'Segmented Point Cloud');`
];
@end


@displaycodeblock
  <script style="display: block" modify="false" run-once="true">
    let idx = parseInt(@0);
    send.lia("LIASCRIPT:\n```matlab\n"+window.codeblocks[ idx ]+"\n```");
  </script>
@end

@displaycodeblocks
  <script style="display: block" modify="false" run-once="true">
    let param = "@0";

    let merge = param.trim().split(/\s+/).filter(Boolean).map((p) => {
      let idx = parseInt(p);
      return window.codeblocks[idx];
    }).join("\n\n");
    
    send.lia("LIASCRIPT:\n```matlab\n"+merge+"\n```");
  </script>
@end

-->

# Matlab Tutorial: LiDAR Point Cloud Processing

Learning Outcomes
======================

By the end of this session you will be able to:


## Pointclouds

LiDAR (Light Detection and Ranging) data can be stored as a 3-D point cloud 
This 3-D LiDAR data can then be used and processed to detect obstacles in the path of a vehicle.
pointCloud objects in MATLAB are used to efficiently process this data
This example shows how to configure and use the point cloud player in MATLAB

## MATLAB script


- First section - starting the point cloud viewer

  - Import the point cloud dataset and visualise it via viewer with the standard colormap
  
- Second section – data segmentation:
  
  - Different types of points are identified and labelled with different colours
  
- Third section – advancement of ego vehicle:
  
  - A loop is created to visualise the advancement of the ego vehicle and the segmented data set

<section class="flex-container">
  <div class="flex-child" style="min-width: 300px; max-width: 40%"> 
    ![a](media/lidarpc_1.png "Original Point Cloud")
  </div>

  <div class="flex-child" style="min-width: 300px; max-width: 40%">
    ![a](media/lidarpc_0.png "Segmented Point Cloud")
  </div>
</section>





# Section 1: Loading

Loading the example and setting up the display

The first step is to load an example sequence of point clouds.

This can be done using the `load(...)`  function

The  __region of interest __ then needs to be defined for the display

Similarly\, the point cloud has to be cropped to contain points within the specified region


<div class = "important">
<b style="color: rgb(var(--color-highlight));">The dataset</b><br>

You will need to download the example dataset in order to run this code. 

The file is available [here](01_city_c2s_fcw_10s_Lidar.mat) and should be placed in your MATLAB working directory.

[Download the dataset file](01_city_c2s_fcw_10s_Lidar.mat)
</div>

@displaycodeblock(0)


## Point Cloud Player

When visualised in the point cloud player, the data should appear similarly to the image below. You can rotate and zoom the view using your mouse.

It is worth discussing what this view shows.

1. The colours are not part of the original data, the points in the cloud have been coloured according to their z-axis (vertical) position to help visualise the 3D nature of the scene.
2. The ground plan is very clearly visible, revealed as a series of rings around the ego vehicle. These rings are an artifact of the mechanics of the specific LiDAR sensor used to collect the data. 
  - A mechanical LiDAR sensor rotates a set of lasers around a vertical axis, firing the lasers outwards. Each of the rings seen in this view corresponds to the path of one of these lasers as it rotated.
  - The gaps in the rings occur because the lasers are positioned a discrete vertical angles. The more lasers a rotating LiDAS has, the more rings and the smaller the gaps between them.

![](media/lidarpc_3.png)


# Point Cloud Video

Append this code to the end of the previous code to have MATLAB display each point cloud in sequence.

@displaycodeblock(1)

!?["Matlab Point Cloud Video"](media/matlab.mp4)<!--
autoplay="true"
muted="true"
-->


# Section 2: Segmentation

Ground Plane & Nearby Obstacles segmentation.

Although the ground plane is one of the most prominent features in the point cloud, it is generally of limited interest for obstacle detection.

Identifying and removing the points corresponding to the ground plane allows us to focus on the more relevant obstacles in the scene.

* The first step is to identify the ground plane.
  * This is done using the [RANSAC](https://en.wikipedia.org/wiki/Random_sample_consensus) algorithm.
* We are going to consider the ground points as those points with upward direction (Z plane) < 20cm.
* The function to fit the ground plane is  __‘__  __pcfitplane__  __\(…\)’__  as seen bellow

@displaycodeblock(2)

## Add Colour Labels


@displaycodeblock(3)




## aaa

```matlab
% Select the points that are not part of the ground plane.
pcWlithoutGround = select(pc, outliers);

% Declare a “Danger Zone” around the LiDAR

sensorLocation = [0 0 0]; % place the Lidar sensor at the center of coordinate system
radius = 10; % in meters
nearIndices = findNeighborsInRadius(pclWithoutGround, sensorLocation, radius);

nearPointIndices = outliers(nearIndices) ;

% Label the obstacle points.
colorLabels(nearPointIndices) = redIdx;
```


## Segment the Ego Vehicle

```matlab
% Declare an area to label as the vehicle points
radius = 3; % in meters
nearIndices = findNeighborsInRadius(pcWithoutGround, sensorLocation, radius);

vehiclePointIndices = outliers(nearIndices);
pcVehicle = select(pcWithoutGround, nearIndices) ;
delta = 0.1;

selfCube = [pcVehicle.XLimits(1)-delta, pcVehicle.XLimits(2)+delta ...
pcVehicle.YLimits(1)-delta, pcVehicle.YLimits(2)+delta ...
pcVehicle.ZLimits(1)-delta, pcVehicle.ZLimits(2)+delta];

colorLabels(vehiclePointIndices) = whiteIdx;

playerl = pcplayer(xlimits, ylimits, zlimits);
colormap(player1.Axes, colors)

view(playerl, pc.Location, colorLabels);
title(player1.Axes, 'Segmented Point Cloud');
```


## Updated Point Cloud Player






## Full segmentation code

@displaycodeblocks(0 1 2 3)