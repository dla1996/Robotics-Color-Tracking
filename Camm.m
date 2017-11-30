global HIGH LOW left right down up
HIGH = 1;
LOW = 0;
left = 'GOING LEFT';
right = 'GOING RIGHT';
up = 'Going up';
down = 'Going down';
global KEY_IS_PRESSED
KEY_IS_PRESSED = 0;


%------- This section initializes all the arduino pins
robot = arduino();

%----------------- Grip stuff --------------------
gripPot = 'A0';            % Brown wire
gripEnable = 'D43';        % Digital Pin number for grip enable % Blue
gripDirPin = 'D30';        % Pin number for grip direction pin  % Blue % HIGH = OPEN, LOW = CLOSE
%gripMax = 475;                  % Open
%gripMin = 270;                  % Closed

%----------------- Wrist stuff -------------------
 m1Pot = 'A4';        % Green wire
 wristEnable = 'D42';       % Digital Pin number for wrist enable % Green
 wristDirPin = 'D28';       % Pin number for wrist direction pin % Purple % HIGH = UP , LOW = DOWN
%int wristMax = 856;                 % Going down
%int wristMin = 0;                   % Goin up

%------------------ Elbow stuff ------------------
m2Pot = 'A1';        % Red wire
elbowEnable = 'D41';       % Digital Pin for elbow enable % Gray
elbowDirPin = 'D26';       % Pin number for elbow direction pin % Gray % 
%elbowMax = 994;                 % Going down
%elbowMin = 124;                 % Going up

%------------------ Shoulder stuff ------------------
m3Pot = 'A3';        % Yellow wire
shoulderEnable = 'D40';    % Digital Pin for shoulder enable % Purple
shoulderDirPin = 'D24';    % Pin number for shoulder direction pin % White
%int shoulderMax = 782;              % Going up
%int shoulderMin = 176;              % Going down

%------------------- Waist stuff -------------------
m4Pot = 'A2';        % Orange wire
waistEnable = 'D39';       % Digital Pin for waist enable % Orange
waistDirPin = 'D22';       % Pin number for waist direction pin % Black
%int waistMax = 825;                 % Going left
%int waistMin = 90;                  % Going right


% % Configure the pot outputs as analogIn for arduino
configurePin(robot, gripPot);
configurePin(robot, m1Pot);
configurePin(robot, m2Pot);
configurePin(robot, m3Pot);
configurePin(robot, m4Pot);

% Configure the motor enable pins on the arduino as digitalOutputs
configurePin(robot,gripEnable,'DigitalOutput');
configurePin(robot,wristEnable,'DigitalOutput');
configurePin(robot,elbowEnable,'DigitalOutput');
configurePin(robot,shoulderEnable,'DigitalOutput');
configurePin(robot,waistEnable,'DigitalOutput');

% Configure the direction pins on the arduino as digitalOutputs
configurePin(robot,gripDirPin,'DigitalOutput');
configurePin(robot,wristDirPin,'DigitalOutput');
configurePin(robot,elbowDirPin,'DigitalOutput');
configurePin(robot,shoulderDirPin,'DigitalOutput');
configurePin(robot,waistDirPin,'DigitalOutput');
%------------------------------------------------------------


a = imaqhwinfo;
[camera_name, camera_id, format] = getCameraInfo(a);


% Capture the video frames using the videoinput function
% You have to replace the resolution & your installed adaptor name.
vid = videoinput(camera_name, camera_id, format);

% Set the properties of the video object
set(vid, 'FramesPerTrigger', Inf);
set(vid, 'ReturnedColorspace', 'rgb')
vid.FrameGrabInterval = 2;

vidSize = vid.VideoResolution;

%start the video aquisition here
start(vid)
frame = getsnapshot(vid);
frameSize = size(frame);

bwPlayer = vision.VideoPlayer('Position', [100 100 [frameSize(2), frameSize(1)]+30]);
% Set a loop that stop after 100 frames of aquisition
while(vid.FramesAcquired<=600)
    
    % Get the snapshot of the current frame
    data = getsnapshot(vid);
    
    %set(currFrameText, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
    % Now to track red objects in real time
    % we have to subtract the red component 
    % from the grayscale image to extract the red components in the image.
    diff_im = imsubtract(data(:,:,1), rgb2gray(data));
    %Use a median filter to filter out noise
    diff_im = medfilt2(diff_im, [3 3]);
    % Convert the resulting grayscale image into a binary image.
    diff_im = im2bw(diff_im,0.18);
    
    % Remove all those pixels less than 300px
    diff_im = bwareaopen(diff_im,300);
    
    % Label all the connected components in the image.
    bw = bwlabel(diff_im, 8);
    
    % Here we do the image blob analysis.
    % We get a set of properties for each labeled region.
    stats = regionprops(bw, 'BoundingBox', 'Centroid');
    
    %step(bwPlayer, bw);
    
    % Display the image
    subplot(2,2,1); imshow(data)
    subplot(2,2,2); imshow(bw)
    
    
    hold on
    
    %This is a loop to bound the red objects in a rectangular box.
    for object = 1:length(stats)
        bb = stats(object).BoundingBox;     % Value of Bounding box, length and height
        bc = stats(object).Centroid;        % Value of centroid
        
        if stats(object).Centroid(1) < 640/2
            %moveLeft(robot, waistDirPin, waistEnable);
            moveRight(robot, waistDirPin, waistEnable, ((640/2)-stats(object).Centroid(1))/320);
        end
    
        if stats(object).Centroid(1) > 640/2
            moveLeft(robot, waistDirPin, waistEnable, (stats(object).Centroid(1) - (640/2))/320);
            %moveRight(robot, waistDirPin, waistEnable);
        end
        
        if stats(object).Centroid(2) > 320/2
            %moveElbowUp(robot, elbowDirPin, elbowEnable, ((320/2) - stats(object).Centroid(2))/160);
            moveElbowDown(robot, elbowDirPin, elbowEnable,(stats(object).Centroid(2) - (320/2))/160);
            pause(0.05);
        end
        
        if stats(object).Centroid(2) < 320/2
            moveElbowUp(robot, elbowDirPin, elbowEnable, ((320/2) - stats(object).Centroid(2))/160);
            %moveElbowDown(robot, elbowDirPin, elbowEnable,(stats(object).Centroid(2) - (320/2))/160);

            pause(0.05)
        end
        
        % Create a box with dimensions bb, EdgeColor red(r)
        % LineWidth 2
        rectangle('Position',bb,'EdgeColor','r','LineWidth',2)
        
        % Plot at Centroid values, bc(1) horicontal, bc(2) Vertical
        plot(bc(1),bc(2), '-m+')
        
        % create a text object at x, y, sometext
        a=text(bc(1)+15,bc(2), strcat('X: ', num2str(round(bc(1))), '    Y: ', num2str(round(bc(2)))));
        currFrameText = text(15,vidSize(2)-100,num2str(vid.FramesAcquired));
        
        set(a, 'FontName', 'Arial', 'FontWeight', 'bold', 'FontSize', 12, 'Color', 'yellow');
        
    end
    
    hold off
end
% Both the loops end here.

% Stop the video aquisition.
stop(vid);

% Flush all the image data stored in the memory buffer.
flushdata(vid);

% Clear all variables
%release(bwPlayer);
clear all
sprintf('%s','That was all about Image tracking, Guess that was pretty easy :) ')

function moveRight(robot, waistDirPin, waistEnable, error)
       
        global HIGH LOW right left
        display(left)
        writeDigitalPin(robot, waistDirPin, LOW);
        writeDigitalPin(robot, waistEnable, HIGH);
        
        pause(error * 0.2);
        writeDigitalPin(robot, waistEnable, LOW);
end

function moveLeft(robot, waistDirPin, waistEnable, error)
       global HIGH LOW left right
       display(right)
        writeDigitalPin(robot, waistDirPin, HIGH);
        writeDigitalPin(robot, waistEnable, HIGH);
       
        pause(error * 0.2);
        writeDigitalPin(robot, waistEnable, LOW);
end

function moveUp(robot, shoulderDirPin, shoulderEnable, error)
        global HIGH LOW down up
        display(down);
        writeDigitalPin(robot, shoulderDirPin,HIGH);
        writeDigitalPin(robot, shoulderEnable,HIGH);
        pause(error * 0.2);
        writeDigitalPin(robot, shoulderEnable, LOW);
        
end

function moveDown(robot, shoulderDirPin, shoulderEnable, error)
        global HIGH LOW down up
        display(up);
        writeDigitalPin(robot, shoulderDirPin,LOW);
        writeDigitalPin(robot, shoulderEnable,HIGH);
        pause(error * 0.2);
        writeDigitalPin(robot, shoulderEnable, LOW);
        
end

function moveElbowUp(robot, elbowDirPin, elbowEnable, error)
        global HIGH LOW down up
        display(up);
        writeDigitalPin(robot, elbowDirPin,HIGH);
        writeDigitalPin(robot, elbowEnable,HIGH);
        pause(error * 0.2);
        writeDigitalPin(robot, elbowEnable, LOW);
        
end
 
function moveElbowDown(robot, elbowDirPin, elbowEnable, error)
        global HIGH LOW down up
        display(down);
        writeDigitalPin(robot, elbowDirPin,LOW);
        writeDigitalPin(robot, elbowEnable,HIGH);
        pause(error * 0.2);
        writeDigitalPin(robot, elbowEnable, LOW);
        
end

