function ExtractFrame

% This function extracts frame from a video and outputs image file on the
% directory of the video. 
% Feel free to modify such that it receives filename, more complex for loop
% input, etc. 

clc;
clear all;
workingDir = 'C:\Users\Yeshara Packirisamy\Documents\UCT\Fourth Year 2018\Thesis\Footage_Run_Thesis\Calibration\External\Extrinsics\ER';
mkdir(workingDir,'images')

%vid1= fullfile('C:\Users\Yeshara Packirisamy\Documents\UCT\Fourth Year 2018\Thesis\Footage_Run_Thesis\Calibration\External\Extrinsics\EL','EL_calibration.MP4');
obj = VideoReader('ER_calibration.MP4');
%numFrames = obj.NumberOfFrames;

for img = 2197:100:20158    
    %imgstr=num2str((img));
    %filename=strcat('EL_calibration_',imgstr,'.jpg');
    filename = [sprintf('%05d',(img-2196)) '.jpg'];
    fullname=fullfile(workingDir,'images',filename);
    b = read(obj,img);
    imwrite(b,fullname);
end

end

