close all;
clear all;
clc;

%% Plot parameters
fontSize  = 20;
lineWidth = 3;

autumncolors = colormap(autumn);
bonecolors   = colormap(bone);
summercolors = colormap(summer);
wintercolors = colormap(winter);


%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

%% Load data
load('/home/yeshi/software/robotology-superbuild/robotology/human-dynamics-estimation/conf/xml/testData/matLogFile.mat');

subjectMass = 53.5;
I_g         = [0 0 9.81]';
I_f_g       = (subjectMass) * I_g;

baseLinkName  = 'Pelvis';
baseLinkIndex = find(strcmp(linkNames, baseLinkName)); 

leftFootLinkName  = 'LeftFoot';
leftFootLinkIndex = find(strcmp(linkNames, leftFootLinkName));

rightFootLinkName  = 'RightFoot';
rightFootLinkIndex = find(strcmp(linkNames, rightFootLinkName));

left_foot_f_g  = [];
right_foot_f_g = [];
b_f_g = [];
b_f2_g = [];

%% Get left foot wrench measurements expressed in link frame
LeftFootMeasuredWrenchInLinkFrame  = data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
RightFootMeasuredWrenchInLinkFrame = data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

LeftFootMeasuredWrenchInLinkFrame_in_base_old = [];
RightFootMeasuredWrenchInLinkFrame_in_base_old = [];

LeftFootMeasuredWrenchInLinkFrame_new = [];
RightFootMeasuredWrenchInLinkFrame_new = [];

LeftFootMeasuredWrenchInLinkFrame_in_base_new = [];
RightFootMeasuredWrenchInLinkFrame_in_base_new = [];

left_beta = [];
right_beta = [];


for i = 1:size(linkData(baseLinkIndex).data.pose, 2) %% Assuming all the time series length is correct
    
    w_R_b_rpy = linkData(baseLinkIndex).data.pose(4:6, i);
    w_R_b = iDynTree.Rotation.RPY(w_R_b_rpy(1), w_R_b_rpy(2), w_R_b_rpy(3));
    b_R_w = w_R_b.inverse;
    
    % Gravity vector in base
    b_f_g(i,:) = b_R_w.toMatlab * I_f_g;
        
    w_R_left_foot_rpy = linkData(leftFootLinkIndex).data.pose(4:6, i);
    w_R_left_foot = iDynTree.Rotation.RPY(w_R_left_foot_rpy(1), w_R_left_foot_rpy(2), w_R_left_foot_rpy(3));
    
    w_R_right_foot_rpy = linkData(rightFootLinkIndex).data.pose(4:6, i);
    w_R_right_foot = iDynTree.Rotation.RPY(w_R_right_foot_rpy(1), w_R_right_foot_rpy(2), w_R_right_foot_rpy(3));
     
    LeftFootMeasuredWrenchInLinkFrame_in_base_old(i,:) = b_R_w.toMatlab * LeftFootMeasuredWrenchInLinkFrame(i, 1:3)';
    RightFootMeasuredWrenchInLinkFrame_in_base_old(i,:) = b_R_w.toMatlab * RightFootMeasuredWrenchInLinkFrame(i, 1:3)';
    
    %% New wrench correction
    w_R_left_foot_mat = w_R_left_foot.toMatlab();
    w_R_right_foot_mat = w_R_right_foot.toMatlab();
    
    left_cosBeta = w_R_left_foot_mat(3,3);
    right_cosBeta = w_R_right_foot_mat(3,3);
    
    left_sinBeta = sqrt(1 - ( left_cosBeta * left_cosBeta ));
    right_sinBeta = sqrt(1 - ( right_cosBeta * right_cosBeta ));
    
    left_beta(i, :) = atan(left_sinBeta/left_cosBeta);
    right_beta(i, :) = atan(right_sinBeta/right_cosBeta);
   
    left_ffx_R_s = iDynTree.Rotation.RPY(0, -left_beta(i,:), 0);
    right_ffx_R_s = iDynTree.Rotation.RPY(0, -right_beta(i,:), 0);

    LeftFootMeasuredWrenchInLinkFrame_new(i, :) = left_ffx_R_s.toMatlab * LeftFootMeasuredWrenchInLinkFrame(i, 1:3)';
    RightFootMeasuredWrenchInLinkFrame_new(i, :) = right_ffx_R_s.toMatlab * RightFootMeasuredWrenchInLinkFrame(i, 1:3)';
    
    LeftFootMeasuredWrenchInLinkFrame_in_base_new(i,:) = b_R_w.toMatlab * w_R_left_foot.toMatlab * LeftFootMeasuredWrenchInLinkFrame_new(i, 1:3)';
    RightFootMeasuredWrenchInLinkFrame_in_base_new(i,:) = b_R_w.toMatlab * w_R_right_foot.toMatlab * RightFootMeasuredWrenchInLinkFrame_new(i, 1:3)';
    
    
end

%% Plot forces in assumed link and xsens base frames
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
%     plot(LeftFootMeasuredWrenchInLinkFrame_in_base_old(:,s), 'LineWidth', lineWidth, 'Color', autumncolors(40, :));
%     hold on;
%     plot(LeftFootMeasuredWrenchInLinkFrame_in_base_new(:,s), 'LineWidth', lineWidth, 'Color', autumncolors(40, :), 'LineStyle', '--');
%     hold on;
%     
%     plot(RightFootMeasuredWrenchInLinkFrame_in_base_old(:,s), 'LineWidth', lineWidth, 'Color', summercolors(40, :));
%     hold on;
%     plot(RightFootMeasuredWrenchInLinkFrame_in_base_new(:,s), 'LineWidth', lineWidth, 'Color', summercolors(40, :), 'LineStyle', '--');
%     hold on;
%     
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base_old(:,s) + RightFootMeasuredWrenchInLinkFrame_in_base_old(:,s), 'LineWidth', lineWidth, 'Color', wintercolors(60, :));
    hold on;
    plot(LeftFootMeasuredWrenchInLinkFrame_in_base_new(:,s) + RightFootMeasuredWrenchInLinkFrame_in_base_new(:,s), 'LineWidth', lineWidth, 'Color', wintercolors(60, :), 'LineStyle', '--');
    hold on;
    plot(b_f_g(:,s), 'LineWidth', lineWidth, 'Color', bonecolors(80, :));
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
%     legend('$f_{l}$ Old', '$f_{l}$ New',  '$f_{r}$ Old', '$f_{r} New$', '$f_{l} + f_{r}$ Old', '$f_{l} + f_{r}$ New', '$mg$',...
%            'FontSize', fontSize, 'Location', 'Best', 'Interpreter', 'latex');
legend('$f_{l} + f_{r}$ Old', '$f_{l} + f_{r}$ New', '$mg$',...
           'FontSize', fontSize, 'Location', 'Best', 'Interpreter', 'latex');
end

a = axes;
t = title ("Foot measured forces expressed in Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')


fH = figure('units','normalized','outerposition',[0 0 1 1]);
plot(left_beta * (180/pi), 'LineWidth', lineWidth, 'Color', wintercolors(120, :));
hold on;
plot(right_beta * (180/pi), 'LineWidth', lineWidth, 'Color', summercolors(120, :));
hold on;
xlabel('Samples', 'FontSize', fontSize);
ylabel('$\beta$ (Degree)', 'Interpreter', 'latex', 'FontSize', fontSize);
legend('$\beta_{l}$', '$\beta_{r}$',...
       'FontSize', fontSize, 'Location', 'Best', 'Interpreter', 'latex');

a = axes;
t = title ("Time evolution of $\beta$",  'Interpreter', 'latex');
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')