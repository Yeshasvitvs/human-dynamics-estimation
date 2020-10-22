close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

subjectMass = 55.2;
I_g         = [0 0 9.81]';
I_f_g       = (subjectMass) * I_g;


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];
legendString = ["x", "y", "z"];

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

comAcc = diff(data.comVelocity')/0.02;

%% Get left foot wrench measurements expressed in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

%% The feet wrench from shows are in assumed frame that has the same orientation as the inertial frame
%% So the force can be summed, but not the moments
FeetMeasuredWrenchInLinkFrame = LeftFootMeasuredWrenchInLinkFrame + RightFootMeasuredWrenchInLinkFrame;
FeetMeasuredWrenchInLinkFrameNorm = [];

for i=1:size(FeetMeasuredWrenchInLinkFrame, 1)
    FeetMeasuredWrenchInLinkFrameNorm(i) = norm(FeetMeasuredWrenchInLinkFrame(i,1:3));
end


fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
    plot(LeftFootMeasuredWrenchInLinkFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(RightFootMeasuredWrenchInLinkFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(FeetMeasuredWrenchInLinkFrame(:,s), 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot', 'Right Foot', 'Left + Right',...
           'FontSize', fontSize, 'Location', 'Best');
end

a = axes;
t = title ("Foot measured forces expressed in Inertial Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


fH = figure('units','normalized','outerposition',[0 0 1 1]);
plot(FeetMeasuredWrenchInLinkFrameNorm, 'LineWidth', lineWidth)
hold on;
plot(ones(size(FeetMeasuredWrenchInLinkFrameNorm,2))*I_f_g(3), 'LineWidth', lineWidth)
hold on;
title ("Norm of Foot measured forces Vs Weight expressed in Inertial Frame");


fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
    plot(subjectMass * data.comBiasAcceleration(s,:)', 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    legend(legendString(s),...
           'FontSize', fontSize, 'Location', 'Best');
end


a = axes;
t = title ("$m \ \ddot{x}_{com}$", 'Interpreter', 'latex');
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')

