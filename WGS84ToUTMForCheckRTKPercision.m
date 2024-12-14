% clearvars -except filepath;
clear;
close all;clc

%% Transform Wgs to UTM
path = 'D:\matlab相关\算法\数据转换代码\';
% path = filepath;
name1 = 'raw_gnss.csv';
% name2 = 'pointcloud.csv_time_vector2.csv';
Wgs84 = readtable([path name1]);
Lat = table2array(Wgs84(:,"field_latitude"));
Lon = table2array(Wgs84(:,"field_longitude"));
alt = table2array(Wgs84(:,"field_altitude"));
[x,y,utmzone] = deg2utm(Lat,Lon);
pose_true = zeros(length(x) , 4);
pose_true(2:end,1) = x(2:end , :) - x(1,:);
pose_true(2:end,2) = y(2:end , :) - y(1,:);
pose_true(2:end,3) = alt(2:end , :) - alt(1,:);
pose_true(:,4) = table2array(Wgs84(:,"x_time"));
figure,plot(pose_true(2:end,4)-pose_true(1:end-1,4));
figure
plot(pose_true(:,1) , pose_true(:,2) , 'g-');axis equal;

%% Get Timestamp(about Radar,RTK)
time_vector_rtk = table2array(Wgs84(:,"x_time"));
% RadarTime = readtable([path name1]);
time_vector_radar = time_vector_rtk;
Value_index = zeros(length(time_vector_radar) , 2);
pose_time_align = zeros(size(pose_true));
pose_time_align(:,3) = pose_true(:,3);
% p = zeros(length(time_vector_radar) , 2);

%% align pose and Timestamp
for i = 1:length(time_vector_radar)
    DiffTime = time_vector_rtk - time_vector_radar(i);
    [Value_index(i,1) , Value_index(i,2)] = min(abs(DiffTime));
    if i < 6
        p = polyfit(pose_true(1:i+5,1), pose_true(1:i+5,1), 1);

        % solve projection point1
        A1 = -1/p(1);
        B1 = pose_true(1,2) - A1*pose_true(1,1);
        x1_p = (-B1+p(2))/(-A1+p(1));
        y1_p = (B1*p(1)-A1*p(2))/(-A1+p(1));

        % solve projection point2
        A2 = -1/p(1);
        B2 = pose_true(i+5,2) - A2*pose_true(i+5,1);
        x2_p = (-B2+p(2))/(-A2+p(1));
        y2_p = (B2*p(1)-A2*p(2))/(-A2+p(1));

        % solve reference point 
        pose_time_align(i,1:2) = (time_vector_radar(i) - time_vector_rtk(1))*([x2_p , y2_p] - [x1_p , y1_p])+[x1_p , y1_p];

    elseif i+5>length(time_vector_rtk)
        p = polyfit(pose_true(i-5:length(time_vector_rtk),1), pose_true(i-5:length(time_vector_rtk)), 1);

        % solve projection point1
        A1 = -1/p(1);
        B1 = pose_true(i-5,2) - A1*pose_true(i-5,1);
        x1_p = (-B1+p(2))/(-A1+p(1));
        y1_p = (B1*p(1)-A1*p(2))/(-A1+p(1));

        % solve projection point2
        A2 = -1/p(1);
        B2 = pose_true(length(time_vector_rtk),2) - A2*pose_true(length(time_vector_rtk),1);
        x2_p = (-B2+p(2))/(-A2+p(1));
        y2_p = (B2*p(1)-A2*p(2))/(-A2+p(1));

        % solve reference point 
        pose_time_align(i,1:2) = (time_vector_radar(i) - time_vector_rtk(1))*([x2_p , y2_p] - [x1_p , y1_p])+[x1_p , y1_p];

        
    else
        p = polyfit(pose_true(i-5:i+5,1), pose_true(i-5:i+5,1), 1);

        % solve projection point1
        A1 = -1/p(1);
        B1 = pose_true(i-5,2) - A1*pose_true(i-5,1);
        x1_p = (-B1+p(2))/(-A1+p(1));
        y1_p = (B1*p(1)-A1*p(2))/(-A1+p(1));

        % solve projection point2
        A2 = -1/p(1);
        B2 = pose_true(i+5,2) - A1*pose_true(i+5,1);
        x2_p = (-B2+p(2))/(-A2+p(1));
        y2_p = (B2*p(1)-A2*p(2))/(-A2+p(1));

        % solve reference point 
        pose_time_align(i,1:2) = (time_vector_radar(i) - time_vector_rtk(1))*([x2_p , y2_p] - [x1_p , y1_p])+[x1_p , y1_p];
    end
     pose_time_align(i,4) = time_vector_radar(i);
end

writematrix(pose_true , [path 'RTK.csv'])