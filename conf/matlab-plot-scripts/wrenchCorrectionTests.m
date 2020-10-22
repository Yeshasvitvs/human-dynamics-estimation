clear all;
close all;
clc;

previousWrenchCorrectionData = load('previousWrenchCorrection.mat');
newWrenchCorrectionData      = load('newWrenchCorrection.mat');

%% Legend or Title Index
wrenchLegendString = ["$f_x [N]$", "$f_y [N]$", "$f_z [N]$","$m_x [Nm]$", "$m_y [Nm]$", "$m_z [Nm]$"];

subjectMass = 53.5;
I_g         = [0 0 9.81]';
I_f_g       = (subjectMass) * I_g;


baseLinkName  = 'Pelvis';
baseLinkIndex = find(strcmp(linkNames, baseLinkName)); 

leftFootLinkName  = 'LeftFoot';
leftFootLinkIndex = find(strcmp(linkNames, leftFootLinkName));

rightFootLinkName  = 'RightFoot';
rightFootLinkIndex = find(strcmp(linkNames, rightFootLinkName));


%% Get left foot wrench measurements expressed in link frame
previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame  = previousWrenchCorrectionData.data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame = previousWrenchCorrectionData.data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame  = newWrenchCorrectionData.data.task1_wrenchMeasurementsInLinkFrame(1:6,:)';
newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame = newWrenchCorrectionData.data.task1_wrenchMeasurementsInLinkFrame(7:12,:)';

previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base = [];
previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base = [];

newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base = [];
newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base = [];


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
     
    previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_left_foot.toMatlab * previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame(i, 1:3)';
    previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_right_foot.toMatlab * previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame(i, 1:3)';
    
    newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_left_foot.toMatlab * newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame(i, 1:3)';
    newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(i,:) = b_R_w.toMatlab * w_R_right_foot.toMatlab * newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame(i, 1:3)';
   
end


%% Plot forces in assumed link and xsens base frames
fH = figure('units','normalized','outerposition',[0 0 1 1]);

for s = 1:3
    
    subplot(3,1,s);
    plot(previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(previousWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(:,s) + previousWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    
    plot(newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    plot(newWrenchCorrectionData_LeftFootMeasuredWrenchInLinkFrame_in_base(:,s) + newWrenchCorrectionData_RightFootMeasuredWrenchInLinkFrame_in_base(:,s), 'LineWidth', lineWidth);
    hold on;
    %         ylim([-400 800])
    xlabel('Samples', 'FontSize', fontSize);
    ylabel(wrenchLegendString(s), 'Interpreter', 'latex', 'FontSize', fontSize);
    set (gca, 'FontSize' , fontSize)
    legend('Left Foot', 'Right Foot', 'Left + Right',...
           'FontSize', fontSize, 'Location', 'Best');
end

a = axes;
t = title ("Foot measured forces expressed in Base Frame");
t.FontSize = fontSize;
a.Visible = 'off' ;
t.Visible = 'on' ;
axis(a,'fill')