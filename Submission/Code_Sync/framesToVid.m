%Input file of images 'VidImages' in directory
%Outputs a video named vidName in directory
%vid used to set FrameRate etc., not read from

load('CamMatrix.mat');
vidName = 'FR_Run6_flip.MP4';       %output name

directory = 'C:\Users\Yeshara Packirisamy\Documents\UCT\Fourth Year 2018\Thesis\Footage_Run_Thesis\Test Data\2-Jog';
vid1= fullfile('C:\Users\Yeshara Packirisamy\Documents\UCT\Fourth Year 2018\Thesis\Footage_Run_Thesis\Test Data\2-Jog','FR_Run6.MP4');
vid = VideoReader(vid1);

imageNames = dir(fullfile(directory,'images','*.jpg'));
imageNames = {imageNames.name}';

outputVideo = VideoWriter(fullfile(directory,vidName),'MPEG-4');
outputVideo.FrameRate = vid.FrameRate;
open(outputVideo)

numberOfFramesWritten = 0;
for ii = 1:(length(imageNames))
   img = imread(fullfile(directory,'images',imageNames{ii}));
   img=imrotate(img,180);
 % [img,newOrigin]= undistortImage(img,stereoParams.CameraParameters1);
   writeVideo(outputVideo,img)
   progressIndication = sprintf('%4d of %d video frames complete', (ii),(length(imageNames)));
   disp(progressIndication);
   numberOfFramesWritten = numberOfFramesWritten + 1;
end

close(outputVideo)

complete = sprintf('Video Complete');
 disp(complete);
