% Nicholas Kapsanis, z5254990

function main(ThisDataset)

    clc();  % clear screen.
    if ~exist('ThisDataset','var')
        disp('You need to specify the folder from where to read the dataset'); 
        return;
    end

    % Some shared variales.
    API=[];
    FFF = zeros(10,1);  FFF(1)=1;  % FFF is used as an array of flags.
    main01(ThisDataset)

return;
% ...........................................................

function main01(ThisDataset)

    % initialize API
    API = API4010_v13();
    
    % Select a dataset to playBack
    ok=API.e.SelectDataset(ThisDataset);
    if (ok<1)
        return;
    end
    
    % create figures in which we show data dynamically
    [hp,hir,hic]=InitSomePlots();
    hButtons=CreateControlsInFigure(12); 
    %---------------------------------------------------
    % API functions for creating oscilloscope figures.
    PushScopes3Channels = API.c.PushScopes3Channels;
    IniScopes3Channels = API.c.IniScopes3Channels;
    PushScopes4Channels = API.c.PushScopes4Channels;
    IniScopes4Channels = API.c.IniScopes4Channels;
    % ----------------------------------------------------------------------------------

    % Gyroscopes in figure#30
    a=20;   hhScopesGyros = IniScopes3Channels(30,400,-a,a,'3D gyros'         ,{'Wx','Wy','Wz'});     cxG=0;
    set(30,'name','gyros');
    % default range, -/+ 20 degrees/sec.

    % Accelerometer in figure#31
    a=1.2 ;   hhScopesAcc = IniScopes3Channels(31,400,-a,a,'3D accelerometers',{'ax','ay','az'}); cxA=0;
    set(31,'name','accXYZ');
    % default range, -/+ 1.2G
    
    % Attitude1 - API Attitude in figure#32
    a=50 ;   hhScopesAtti1 = IniScopes3Channels(32,800,-a,a,'Attitude1 API - Uncalibrated',{'Roll','Pitch','Yaw'}); cxAtti1=0;
    set(32,'name','Attitude - API');
    % default range, -/+ 50degrees

    % Attitude2 - API calibrated in figure#33
    a=10 ;   hhScopesAtti2 = IniScopes3Channels(33,800,-a,a,'Attitude2 API - Calibrated',{'Roll','Pitch','Yaw'}); cxAtti2=0;
    set(33,'name','Attitude - API, calibrated');

    % AttitudeP - attitude predictor non API in figure#34
    a = 50;   hhScopesAttiP1 = IniScopes3Channels(34,800,-a,a,'Attitude Predictor - Uncalibrated',{'Roll','Pitch','Yaw'}); cxAttiP1=0;
    set(34,'name','Attitude - non API');

    % AttitudeP2 - attitude predictor calibrated in figure#35
    a = 10;   hhScopesAttiP2 = IniScopes3Channels(35,800,-a,a,'Attitude Predictor - Calibrated',{'Roll','Pitch','Yaw'}); cxAttiP2=0;
    set(35,'name','Attitude - non API, calibrated');
    
    % Magnetometers in figure#36
    a=1 ;   hhScopesMag = IniScopes3Channels(36,400,-a,a,'3D magnetometers',{'Mx','My','Mz'}); cxM=0;
    set(36,'name','magnetometers');

    % unCalibrated Error checking oscilloscope in figure#37
    a=2 ;   hhScopesUncalibratedErr = IniScopes3Channels(37,400,-a,a,'Uncalibrated Error',{'Roll del','Pitch del','Yaw del'}); cxE1=0;
    set(37,'name','Error checking - Uncalibrated');

    % Calibrated Error checking oscilloscope in figure#38
    a=2 ;   hhScopesCalibratedErr = IniScopes3Channels(38,400,-a,a,'Calibrated Error',{'Roll del','Pitch del','Yaw del'}); cxE2=0;
    set(38,'name','Error checking - Calibrated');

    % auto ROI scope
    a=50;   hhScopeAutoRollPitch = IniScopes4Channels(39,400,-a,a,'Auto ROI Roll Pitch Altitude Time',{'Roll','Pitch','Altitude','Execution Time'}); cxROI=0;
    API.c.SetVerticalScalesOsci(hhScopeAutoRollPitch(1),[-50,50]);
    API.c.SetVerticalScalesOsci(hhScopeAutoRollPitch(2),[-50,50]);
    API.c.SetVerticalScalesOsci(hhScopeAutoRollPitch(3),[0,100]);
    API.c.SetVerticalScalesOsci(hhScopeAutoRollPitch(4),[0,12]);
    set(39,'name','Auto ROI Roll Pitch Altitude Time');

    % project 2 B7 scope, should be 3 channels, roll, pitch, yaw each with 3 datasets in 3 different colors
    a=50;
    L = 1400; % length of the scope
    Fig =40; % figure number for the scope
    hhScopeB7Uncalibrated = IniScopes3Channels(Fig,L,-a,a,'State Prediction Comparison',{'Roll','Pitch','Yaw'}); cxB7=0;
    set(Fig, 'name','State Prediction Comparison');
    set(hhScopeB7Uncalibrated, 'Color', [0,1,0]); % Set the color of the scope to green

    % EKF guest scope for B7
    hhScopeB7EKF = IniScopes3Channels(Fig,L,0,0,[],[],1); cxB7EKF=0;
    set(hhScopeB7EKF, 'Color', [1,0,0]); % Set the color of the scope to red

    hhScopeB7GroundTruth = IniScopes3Channels(Fig,L,0,0,[],[],2); cxB7GT=0;
    set(hhScopeB7GroundTruth, 'Color', [0,0,1]); % Set the color of the scope to blue

    figure(12); % put #12 on top.
    figure(Fig); %put the B7 scope on top.

    OtherInitializations();
    
    % ------------------------------------------------    
    % "Pointers" to other useful and frequently used API functions.    
    GetEvnt = API.GetEvnt;                              % to read, chronologically, events (usually sensors' measurements)
    ConvertDepthsTo3DPoints = API.e.Depths2pts;         % to convert Depth images into 3D points clouds.
    Rotate2D  = API.e.Rotate2D;                         % simple 2D rotation.
    
    
    %get first measurement!.
    [id,r,ti]=GetEvnt();
    % in which:
    % id: type of event (an identifier).
    % r : event's data (usually actual measurement).
    % ti: timestamp (1 count =0.1ms, class uint32).
    
    t=double(ti)/10000;   % convert to time to seconds. 
    t0=t;                 % to remember the time of the previous event.   
    t00=t;                % to remember the time of the very first event ("initial time").
    
    % Variable for storing the last measurment from IMU. 
    % (accelerometers,gyros, magnetometers)
    imuData=[0;0;0; 0;0;0; 0;0;0];   % [ax;ay;az; gx;gy;gz; mx;my;mz] ; 9x1
    gxyz = [0;0;0];                  % for storing last gyros' measurement, separately.    
    Depth=[];  %last received Depth image.
    RGB0=[];   %last received RGB image.
    
    FFF(2)=1;            %flag PAUSE=1;  We will start in paused condition.
    % In this example, we use the flag FFF(2) to indicate if the playback
    % session is paused or running.
    
    % ---------------------------------------------------------
    
    % these API functions are for using the state equation of the Attitude, 0 = uncalibrated, 5 = 5sec calibration.
    ContextAttitude1Predictor = API.i.IniAttitudePredictor(0);
    ContextAttitude5Predictor = API.i.IniAttitudePredictor(5);
    runAttitudePredictor = API.i.runAttitudePredictor;
    
    % Initialise all state variables here.
    %Attitude00 = [0;0;deg2rad(-45)];  % define initial attitude. for use in GG datasetds
    Attitude00 = [0;0;0];  % define initial attitude. for use in HH datasetds
    Bias = [0;0;0]; % define initial bias.
    BGround = [0;0;0]; % define initial bias for the ground truth model
    XeCal = [-1.5; -0.5; -2.5];
    XeCal = deg2rad(XeCal); % convert to radians

    % each attitude heae coresponds to a different attitude predictor
    Attitude1=Attitude00; % API - non calibrated attitude.
    Attitude2=Attitude00; % API - 5 second calibrated attitude
    AttitudeP1=Attitude00; % my attitude predictor - non calibrated.
    AttitudeP2=Attitude00; % my attitude predictor - 5 second calibrated.

    % EKF initialization
    EKF_active = false; % Flag to indicate if EKF is active
    Xe = [0; 0; 0]; % Initial state estimate (roll, pitch, yaw)
    XeSTD = deg2rad(30); % Initial standard deviation of the state estimate (degrees->radians)
    P = eye(3) * XeSTD^2; % Initial covariance matrix (3x3)
    R_floor = diag([0.0025, 0.0025, 0.0025]); % Measurement noise covariance for floor detection
    R_wall = diag([0.0025, 0.0025, 0.0025]); % Measurement noise covariance for wall detection
    Q = zeros(3); %decleration here, updates dynamically.
    GyroSTD = deg2rad(4); %WGN standard dev of gyro measurements (degrees->radians)
    Pu = eye(3) * GyroSTD^2; % Process noise covariance for gyros (3x3)
    % ----------------------------------------------------------------
    
    % events' LOOP!
    while (FFF(1))   % "infinite" loop, to keep reading events, chronologically, as those happened.
        % I use flag FFF(1) = END the loop
        % I use flag FFF(2) = pause/continue
        if FFF(2) % paused
            while (FFF(2))
                pause(0.5);
                if (FFF(1)==0) 
                    break;
                end
                if (FFF(6))==1 % this flag is set to 1, if user wants to select a ROI
                    ChooseROIandEstimatePlane(Depth, AttitudeP2); 
                    FFF(6)=0;
                end
                if (FFF(7))
                    if ~isempty(Depth)
                        TransformAndPlotPoints(Depth, AttitudeP2); 
                    else
                        disp('No depth image available for transformation and plotting.');
                    end
                    FFF(7)=0; % reset it.
                end
            end
        end
        % Loop is running here, not paused. FFF(1)=1
        [id,r,ti]=GetEvnt();   % get new/next event (measurement,etc).
        t=double(ti)/10000-t00;   % time since initial time, in seconds.
        dt=t-t0; % elapsed time since previous event. stricty <=5ms as the IMU is 200Hz.
        t0=t; % remember time of last event.
        
        % run all state equations here

        % API function for running the attitude predictor non calibrated and calibrated
        [Attitude1,ContextAttitude1Predictor,~]=runAttitudePredictor(Attitude1,gxyz,dt,ContextAttitude1Predictor);
        [Attitude2,ContextAttitude5Predictor,~]=runAttitudePredictor(Attitude2,gxyz,dt,ContextAttitude5Predictor);

        % my attitude predictor non calibrated and calibrated
        AttitudeP1 = AttitudeStateEquation(gxyz,dt,AttitudeP1);
        [AttitudeP2, Bias] = CalibratedAttitudeStateEquation(gxyz,t,dt,AttitudeP2,5, Bias);

        % errors relative to API
        AttitudeErrUncalibrated = Attitude1 - AttitudeP1;
        AttitudeErrCalibrated = Attitude2 - AttitudeP2;

        % run EKF here after 15 seconds.
        if (t > 15) 
            EKF_active = true; 
        end
        
        if EKF_active
            % use API jacobian for now, implement mine later
            [Jx, Ju] = GetFunctionJacobians(Xe, gxyz, dt);

            Q = Ju * Pu * Ju';
            P = Jx * P * Jx' + Q; %predicted covariance matrix.
            Xe = AttitudeStateEquation(gxyz, dt, Xe); %predict the state estimate using the Kalman state estimate.
        end

        %ground truth model is the same as the calibrated model but with a better initial guess
        [XeCal, ContextAttitude5Predictor, ~] = runAttitudePredictor(XeCal, gxyz, dt, ContextAttitude5Predictor);
        % NOW, DISPATCH actions, based on the event's type.
        %....................................................
        switch(id)
        % type=1: IMU data from IMU.
            case(1)
                %pull 9x1 data from IMU event, [axyz,gxyz,mxyz], class single, clear r also to prevent bad data
                imuData=r; 
                r=[];
                
                axyz=imuData(1:3);  % 3D accelerometers (G's (9.81m/s^2))
                gxyz=imuData(4:6);  % 3D gyroscopes (rad/sec)
                mxyz=imuData(7:9) ; % 3D magnetometers (micro-teslas)

                %"Push" the gyroscopes' measurements. (3x1)
                cxG=PushScopes3Channels(hhScopesGyros,cxG,gxyz*180/pi,400);
                % "Push" the accelerometers' measurements. (3x1)
                cxA=PushScopes3Channels(hhScopesAcc,cxA,axyz,400);

                % "Push" attituds, 800 samples in degrees. API and predictor, calibrated and uncalibrated
                cxAtti1=PushScopes3Channels(hhScopesAtti1,cxAtti1,Attitude1*180/pi,800);
                cxAtti2=PushScopes3Channels(hhScopesAtti2,cxAtti2,Attitude2*180/pi,800);
                cxAttiP1=PushScopes3Channels(hhScopesAttiP1,cxAttiP1,AttitudeP1*180/pi,800);
                cxAttiP2=PushScopes3Channels(hhScopesAttiP2,cxAttiP2,AttitudeP2*180/pi,800);

                % "Push" the magnetometers' measurements. (3x1)
                cxM=PushScopes3Channels(hhScopesMag,cxM,mxyz,400);
                
                % "Push" the error between the two attitude estimates. (3x1)
                cxE1=PushScopes3Channels(hhScopesUncalibratedErr,cxE1,AttitudeErrUncalibrated*180/pi,400);
                cxE2=PushScopes3Channels(hhScopesCalibratedErr,cxE2,AttitudeErrCalibrated*180/pi,400);

                % "Push" the 3 attitude estimates to the B7 scope.
                if EKF_active
                    cxB7EKF = PushScopes3Channels(hhScopeB7EKF, cxB7EKF, rad2deg(Xe), 1400); % EKF - red
                    cxB7GT = PushScopes3Channels(hhScopeB7GroundTruth, cxB7GT, rad2deg(XeCal), 1400); % Ground truth - blue
                    cxB7 = PushScopes3Channels(hhScopeB7Uncalibrated, cxB7, rad2deg(Attitude1), 1400); % Uncalibrated - green
                end
            continue;
            
            case(3)      % type=3: RGB data from RGB camera.    
                %ShowRGBimage(r,hic);    % we may update the figure that shows the RGB images.
                trgb = t;
                RGB0=r; 
                r=[]; % time and data of last RGB event, in case we needed to remember those.
                % we may keep it, for further use. (it is the last RGB we received.)
            continue;
            
            case(2)     % type=2: Depth data from camera.
                tdepth = t; 
                Depth=r; r=[];   % time and data of last camera Depth event.
                
                [xx,yy,zz]=ConvertDepthsTo3DPoints(Depth,1);
                [zz,xx]=Rotate2D(zz,xx, 18.5*pi/180 );

                % Plot the Depth
                ShowDepthImageAndPoints(Depth,xx,yy,zz,hp,hir);
                % refresh the RGB image
                ShowRGBimage(RGB0,hic); 
                % so that all figures do alsways show RGB,Depth and points that
                % correspond to the same scan.
                
                % We may check if the user wanted to select ROI, etc, etc,
                % even when the playback session is not paused.
                if (FFF(6))
                    if (FFF(6))==1
                        ChooseROIandEstimatePlane(Depth, AttitudeP2);
                        FFF(2)=1;
                    end
                    %  we may check for other values of the flag, for other  purposes.
                    FFF(6)=0; % reset it.
                end

                if (FFF(7))
                    if ~isempty(Depth)
                        TransformAndPlotPoints(Depth, AttitudeP2); 
                    else
                        disp('No depth image available for transformation and plotting.');
                    end
                    FFF(7)=0; % reset it.
                end
                
                % here we consider the fixed ROI to be calculated every time a depth image is recieved
                tic;
                [ok, LastRolePitchFromDepth, AltitudeFromDepth, vn, vnAPI] = autoROI(Depth);

                processingTime = toc*1000;
                % Push the auto ROI data to the oscilloscope
                cxROI = PushScopes4Channels(hhScopeAutoRollPitch,cxROI, [LastRolePitchFromDepth(1)*180/pi; LastRolePitchFromDepth(2)*180/pi; AltitudeFromDepth; processingTime], 400);
                % delay loop playback, also adjust this if you want slower or faster playback.

                % here we process the Kalman filter update step if it is active
                doUpdate = true;
                if EKF_active
                    % tolerance tweaked to detect the wall more often (as EKF starts the platforms yaw is already > 15 degrees, so the wall is not detected initially.)
                    ThisOne = InferWhichWall(vnAPI, Xe, 25);
                    if ThisOne == 1 %floor detected
                        H = FloorDetectionJacobian(Xe);
                        R = R_floor;
                    elseif ThisOne == 2
                        H = WallDetectionJacobian(Xe);
                        R = R_wall;
                        %disp("wall seen at "+t/1000+" seconds");
                    else
                        doUpdate = false; % no valid wall detected, skip update step
                    end
                end
                % do EKF update
                
                if doUpdate && EKF_active
                    y = vnAPI; %note that the measurement here is directly the vn, as the rotational bits are handles in the H matrix.
                    S = R + H * P * H';
                    iS = inv(S);

                    K = P * H' * iS;

                    Xe = Xe + K * y;
                    P = P - K * H * P;

                end
                pause(0.15);   % e.g., 150ms.
            continue;
            %--------------------------------------------------------

            % "Info event". id=100 (API is telling something to your program)
            case(100)
                % this event is just in case, you need it.
                % if not, just ignore it.
                % id=100, having data such as r(1)==1, 
                if (r(1)==1)
                    %A JUMP in time has ocurred, due to user performing JUMP in time.
                    % User jumped in time! This does not occur in real life.
                    % But it is possible in playback sessions.
                    t0=t; 
                    
                    Attitude1=Attitude00;                    % I RESET the state, to its assumed initial condition. "X(t)=X0"
                    fprintf('My program knows that there was a JUMP in time!\n');
                    % you usualy need to detect JUMPS, if you are integrating a continuous time model.
                    % Take proper actions for avoiding nonsensical time steps "dt"
                    % (negative, or other inconcistencies.)
                    % if you jump to the time 0, you may need to re-initialize certain matters.
                end
            continue;
            % ----------------------------
            % Other event types, not considered in this example.
            otherwise
                if(id<1)
                    break;
                end %End of dataset. BYE. Event's ID < 1 indicates that we reached the end of the data.
            continue;
        end  % end SWITCH
    end  % end events' LOOP
        %...........................................................
        disp('END');
        delete(hButtons);   % I delete GUI's buttons that will not be used anymore.
        % but I keep the figures, in case I wanted to inspect them.
end    % end main01 function. 
% ----------------------------------------------------------
% my implementation of the Attitude estimator with calibration option
function [NewAttitudeP, Bias] = CalibratedAttitudeStateEquation(gxyz,t,dt,CurrentAttitudeP,calSeconds, Bias)
    % calibrate if calSeconds > 0;
    % persistent retains value in between calls to the function.

    persistent gSum gCount;
    if isempty(gSum) || calSeconds == 0
        gSum = zeros(3,1); 
        gCount = 0; 
    end

    if t <= calSeconds && calSeconds > 0
        gSum = gSum + gxyz;
        gCount = gCount + 1;
        Bias = gSum / gCount; % average bias
        NewAttitudeP = CurrentAttitudeP; % don't update attitude on random fluctuations.
        return;
    end
    gxyz = gxyz - Bias; % remove bias from gyros.
    NewAttitudeP = AttitudeStateEquation(gxyz,dt,CurrentAttitudeP);
end

function NewAttitudeP = AttitudeStateEquation(gxyz,dt,CurrentAttitudeP)

    ang = CurrentAttitudeP; % Current attitude (roll,pitch,yaw) in radians at time t
    wx = gxyz(1); % Gyroscope roll (rad/s)
    wy = gxyz(2); % Gyroscope pitch (rad/s)
    wz = gxyz(3); % Gyroscope yaw (rad/s)

    % precompute trigonometric functions
    c1 = cos(ang(1));
    c2 = cos(ang(2));
    s1 = sin(ang(1));

    roll = ang(1) + dt*(wx + (wy*s1 + wz*c1)*tan(ang(2)));
    pitch = ang(2) + dt*(wy*c1 - wz*s1);
    yaw = ang(3) + dt*((wy*s1 + wz*c1)/c2);
    
    NewAttitudeP = [roll; pitch; yaw]; % New attitude (roll,pitch,yaw) in radians at time t+dt
end

function SetSomeFictitiousIMUBiases() 
 % This function is to tell the API to add some fictitious biases to the gyros'
   b = 0*pi/180; a=0;
   ok=API.d.PretendExtraIMUBiases([a;a;a;  b;b;b]);
end
% ----------------------------------------------------------
function OtherInitializations()
end

% ----------------------------------------------------------
function ShowDepthImageAndPoints(depthImage,xx,yy,zz, hp,hr)
    % in this case, just some visualization.
    set(hr,'cdata',depthImage);     % show the depth image
    set(hp,'xdata',xx(1:end),'ydata',yy(1:end),'zdata',zz(1:end));   % show current 3D points.
    % I use the function "SET" for dynamically updating properties. Ask MATLAB help for more details.
end
% ----------------------------------------------------------        
% Callback function, for servicing some buttons' events.
function MyCallback(a,b,x)
    switch(x)
        % button pause ON/OFF 
        case 1
            FFF(2)=~FFF(2); 
            fprintf('\nPaused=%d\n',FFF(2)); 
            return ;

        % button "END"
        case 2  
            FFF(1)=0; return ;  % BYE!
        
        % button "go to t=0".
        case 3
            API.e.Jmp(0) ; return ; % playback session will jump to t=0.
        case 4, FFF(6)=1 ; return ; % set 

        % button plot points in GCF
        case 5
            FFF(7) = 1; return; % Flag 7 indicates transform and plot.
    end
    return;
end

function [Jx, Ju] = GetFunctionJacobians(Xe, gxyz, dt)
    phi_x = Xe(1); % roll
    phi_y = Xe(2); % pitch
    w2 = gxyz(2); % pitch rate
    w3 = gxyz(3); % yaw rate

    %precompute trig functions
    c1 = cos(phi_x);
    c2 = cos(phi_y);
    s1 = sin(phi_x);
    s2 = sin(phi_y);
    t2 = tan(phi_y);
    Ju = [
        dt, dt*s1*t2, dt*c1*t2;
        0, dt*c1, -dt*s1;
        0, (dt*s1)/c2, (dt*c1)/c2
    ];

    Jx = [
        dt*t2*(w2*c1-w3*s1)+1, dt*(t2^2+1)*(w3*c1+w2*s1), 0;
        -dt*(w3*c1+w2*s1), 1, 0;
        (dt*(w2*c1-w3*s1))/c2, (dt*s2*(w3*c1+w2*s1))/ c2^2, 1
    ];
end

function H = FloorDetectionJacobian(Xe)
    phi_x = Xe(1); % roll
    phi_y = Xe(2); % pitch

    %precompute trig functions
    c1 = cos(phi_x);
    c2 = cos(phi_y);
    s1 = sin(phi_x);
    s2 = sin(phi_y);

    H = [
        0, -c2, 0;
        c1*c2, -s1*s2, 0;
        -c2*s1, -c1*s2, 0
        ];
end

function H = WallDetectionJacobian(Xe)
    phi_x = Xe(1); % roll
    phi_y = Xe(2); % pitch
    phi_z = Xe(3); % yaw

    %precompute trig functions
    c1 = cos(phi_x);
    c2 = cos(phi_y);
    c3 = cos(phi_z);
    s1 = sin(phi_x);
    s2 = sin(phi_y);
    s3 = sin(phi_z);

    H = -[
        0, -c3*s2, -c2*s3;
        s1*s3 + c1*c3*s2, c2*c3*s1, -c1*c3 - s1*s2*s3;
        c1*s3 - c3*s1*s2, c1*c2*c3,   c3*s1 - c1*s2*s3
    ];
end

% ==========================================================================================================================

% create some control buttons, for my GUI. You may add more, for other purposes.
function hh=CreateControlsInFigure(figu)
 figure(figu);
 currY=1;  hy = 20; px=10;  hyb=hy*1.1;  ddx=150;
 % I generate the buttons' positions using these variables.
 
 hh(1)= CreateMyButton('pause/cont.',[px, currY, ddx, hy],{@MyCallback,1}); currY=currY+hyb;
 hh(2)= CreateMyButton('END',[px, currY, ddx, hy],{@MyCallback,2}); currY=currY+hyb;
 hh(3)= CreateMyButton('Go to t=0',[px, currY, ddx, hy],{@MyCallback,3}); currY=currY+hyb ;
 hh(4)= CreateMyButton('plane fitting',[px, currY, ddx, hy],{@MyCallback,4}); currY=currY+hyb ;
 hh(5)= CreateMyButton('Plot Points in GCF',[px, currY, ddx, hy],{@MyCallback, 5}); currY=currY+hyb;
 
 
 % you may set certain properties of  GOs (in this case, of these control buttons)
 set(hh(1),'BackgroundColor',[1,1,0]); %rgb sort of yellow.
 set(hh(2),'BackgroundColor',[1,0.2,0]);  % sort of red.   
 set(hh(3),'BackgroundColor',[0,1,0]);    % ... green. 
 set(hh(4),'BackgroundColor',[1,1,0]*0.6); %rgb sort of dark yellow.
 set(hh(5),'BackgroundColor',[1,1,1]*0.6); 
 
 return;
end

% ==========================================================================================================================
% note attitude is input for testing of project 2 B2, remove when not testing B2
function ChooseROIandEstimatePlane(Depth, Attitude)
    figuRGB = 10;   % RGB is in figure #10. We want to select ROI in RGB image.
    disp('From the RGB image, choose a rectangular region that seems to be part of the floor');
    r=API.b.SelectROI(figuRGB);
    if (r.ok<1), return ;  end % Bad ROI
    % r.pp = [ [ u1;u2],[v1;v2]]; so that that region of pixels is [u1->u2] horizontal, and [v1->v2] vertical.

    % get boundaries
    p=r.pp;
    u12=p(:,1);   % horizontal interval
    v12=p(:,2);   % vertical interval
          
    % 3D points from ROI
    [ok,xx,yy,zz]=API.b.Gt3DPtsFromROI(u12,v12,Depth,1);   
    if (ok<1), return; end % no points in that ROI.
    
    % compensate camera pitch
    xxAPI = xx; %seperating API points from mine to validate.
    yyAPI = yy;
    zzAPI = zz;

    [zzAPI,xxAPI]=API.e.Rotate2D(zzAPI,xxAPI,18.5*pi/180);
    [xx,yy,zz] = RotateaboutY(xx,yy,zz,18.5*pi/180);

    % now we have the points expressed in platform's CF.
    
    % Plot the selected 3D points
    figure(77);
    clf();
    plot3(xx,yy,zz,'.');
    axis equal;     % locks axis aspect ratio.
    

    %plane fit to a tolerance
     tole = 20 ; % tolerance=20mm = 2cm   note all values are in mm here.
     [ok, vn, xyz0] = FitPlaneToPoints(xx, yy, zz, tole);
     if ~ok
        disp('no plane => no solution');
        title('uhh?, it is not a planar surface!');
        return;
     end
     % if we are here, we have a plane that fits the points, with vn normal unit vector and xyz0 center of geometry.
      [ok,RollPitch] = RollPitchFromVector(vn);
      if (ok<1) 
          disp('no roll and pitch estimation.');
          return;
      end  % NO 

    %find altitide
    altitude = abs(dot(vn,xyz0))/10; %/10 to convert from mm to cm.

    % Roll and Pich have been estimated successfully, compare results to API
     %check that error between my and API plane fitting is small enough
     [~,APIVn,~]=API.p.GtNormalV(xxAPI,yyAPI,zzAPI,tole);
     [~,RollPitchAPI] = API.e.GuessRollPitchFromVector(APIVn,1,0.2);
     ComparePlaneFitting(vn,RollPitch,APIVn,RollPitchAPI);
        if (abs(altitude-(API.e.DistanceToPlane(APIVn,xyz0,[0,0,0]))/10)>2)
            disp('Plane fitting and normal vector estimation are NOT consistent.');
            fprintf('Altitude difference: %.2f cm\n',abs(altitude-(API.e.DistanceToPlane(APIVn,xyz0,[0,0,0]))/10));
        end

      disp('if that plane were the floor, we could estimate platform ROLL, PITCH and ALTITUDE!')
      fprintf('normal=[%.3f,%.3f,%.4f];[roll,pitch]=[%.1f,%.1f]\n',vn,RollPitch*180/pi); 
      fprintf('altitude=%.1fcm\n',altitude); % altitude in cm.
      vn=vn*100;                     % increase its lenght, so it can be seen. 
      % show vector in figure of selected points.
      hold on;
      hq=quiver3(xyz0(1),xyz0(2),xyz0(3),vn(1),vn(2),vn(3));   
      set(hq,'AutoScale','off','linewidth',3);
      title('OK, 3D points define a planar patch!');  
      hold off;

    %testing project 2 B2 section here
    ThisOne = InferWhichWall(vn, Attitude, 15); %15 degrees tolerance for wall estimation.
    disp('ThisOne: ');
    disp(ThisOne);
    return ;
end 


function [ok, RollPitch, Altitude, vn, vnAPI] = autoROI(Depth)
    % Initialize outputs
    vnAPI = [NaN; NaN; NaN]; % Default value for vnAPI
    RollPitch = [NaN; NaN];  % Default value for RollPitch
    Altitude = NaN;          % Default value for Altitude
    vn = [NaN; NaN; NaN];    % Default value for vn

    % define ROI [[u1;u2],[v1;v2]]
    u12 = [80,180];
    v12 = [200,240];

    % 3D points from ROI
    [ok,xx,yy,zz]=API.b.Gt3DPtsFromROI(u12,v12,Depth,1);
    if ~ok
        return;
    end

    [xx,yy,zz] = RotateaboutY(xx,yy,zz,18.5*pi/180);

    tole = 20 ; % tolerance=20mm = 2cm   note all values are in mm here.
    [ok, vn, xyz0] = FitPlaneToPoints(xx, yy, zz, tole);
    if ~ok
        return;
    end

    [ok,RollPitch] = RollPitchFromVector(vn);
    if ~ok
        return;
    end
    Altitude = abs(dot(vn,xyz0))/10; %/10 to convert from mm to cm.

    [ok,vnAPI,~] = API.p.GtNormalV(xx,yy,zz,tole);
    if ~ok
        return;
    end
    
end

% helper function, fits a plane to a cloud of points using SVD
function [ok, vn, xyz0] = FitPlaneToPoints(xx, yy, zz, tolerance)
    % get to matrix form
    points = [xx(:), yy(:), zz(:)];
    xyz0 = mean(points, 1); % center of geometry
    centredPoints = points - xyz0; % center the points

    [~, ~, V] = svd(centredPoints, 0); % singular value decomposition
    vn = V(:, 3); % normal vector is the last column of V and is unitary by this method

    % Ensure the normal vector points upward
    if vn(3) < 0
        vn = -vn; % Flip the normal vector
    end

    %check tolerances
    distances = abs(centredPoints * vn);
    ok = all(distances < tolerance); % check if all distances are within tolerance
end

% helper function, to guess the roll and pitch angles from the normal vector of a plane assumed to be the floor.
function [ok, RollPitch] = RollPitchFromVector(vn)
    % vn is the normal unit vector of the plane, assumed to be the floor.
    % RollPitch = [roll; pitch] in radians.
    if norm(vn) == 0 || any(isnan(vn)) || any(isinf(vn))
        ok = 0;
        RollPitch = [0; 0]; % invalid input
        return;
    end

    roll = atan2(vn(2), vn(3));
    pitch = atan2(-vn(1), sqrt(vn(2)^2 + vn(3)^2));
    RollPitch = [roll; pitch]; % radians.
    ok = 1;
end

% helper function to check my implementation against the API one for plane fitting and normal vector estimation
function ComparePlaneFitting(vn, RollPitch, APIVn, APIRollPitch)
    %compare normal vectors
    angleDiff = acosd(dot(vn, APIVn));  % vectors are known unity so no need to normalise.

    %note that roll and pitch are radians, so we convert to degrees.
    rollDiff = abs(RollPitch(1) - APIRollPitch(1)) * (180 / pi);
    pitchDiff = abs(RollPitch(2) - APIRollPitch(2)) * (180 / pi);

    if angleDiff < 2 && rollDiff < 2 && pitchDiff < 2
        fprintf('Plane fitting and normal vector estimation are consistent.\n');
    else
        fprintf('Plane fitting and normal vector estimation are NOT consistent.\n');
        fprintf('Comparison Results:\n');
        fprintf('Normal Vector Difference: %.2f degrees\n', angleDiff);
        fprintf('Roll Difference: %.2f degrees\n', rollDiff);
        fprintf('Pitch Difference: %.2f degrees\n', pitchDiff);
    end
end
% helper function to rotate about Y 3D version used for readability.
function [xx,yy,zz] = RotateaboutY(xx,yy,zz,rad)

    R = [cos(rad), 0, sin(rad); 
        0, 1, 0; 
        -sin(rad), 0, cos(rad)];

    rotatedPoints = R * [xx(:)'; yy(:)'; zz(:)'];
    
    xx = rotatedPoints(1,:)'; % rotated x-coordinates
    yy = rotatedPoints(2,:)'; % rotated y-coordinates
    zz = rotatedPoints(3,:)'; % rotated z-coordinates
end

function [xx, yy, zz] = RotatePoints(xx, yy, zz, roll, pitch, yaw)
    % Rotation matrices
    Rx = [1, 0, 0; 0, cos(roll), -sin(roll); 0, sin(roll), cos(roll)]; % Roll
    Ry = [cos(pitch), 0, sin(pitch); 0, 1, 0; -sin(pitch), 0, cos(pitch)]; % Pitch
    Rz = [cos(yaw), -sin(yaw), 0; sin(yaw), cos(yaw), 0; 0, 0, 1]; % Yaw

    % Combined rotation matrix
    R = Rz * Ry * Rx;

    % Apply rotation
    rotatedPoints = R * [xx(:)'; yy(:)'; zz(:)'];
    xx = rotatedPoints(1, :)';
    yy = rotatedPoints(2, :)';
    zz = rotatedPoints(3, :)';
end

function TransformAndPlotPoints(depth, attitude)
    [xx,yy,zz] = API.e.Depths2pts(depth,1);
    % to compensate camera misalignment to express in platforms CF
    [xx,yy,zz] = RotateaboutY(xx,yy,zz,18.5*pi/180);

    % get attitude
    roll = attitude(1);
    pitch = attitude(2);
    yaw = attitude(3);

    %rotate inb 3D based on attitude
    [xx, yy, zz] = RotatePoints(xx, yy, zz, roll, pitch, yaw);

    % plot in fig 55
    figure(55); clf(); 
    plot3(xx,yy,zz,'.','markersize',1); title('Transformed 3D points');
  
    set(55,'name','3Dpts Transformed');
    
    axis([0,1400, -700,+700, -700,+700]);  % .
    xlabel('X');ylabel('Y');zlabel('Z'); grid on;

    set(gca(),'Clipping','off'); 

end



%------------------------------------------------------     
end   %end nesting function "main()".
%------------------------------------------------------     

%------------------------------------------------------     
% I create my GUI (you may prefer other style, so implement that way)
function [hp,hir,hic]=InitSomePlots()
    % create some figures for dynamic plots, and some menu to allow user actions.
    % This function returns the handles of graphic object,etc; to be used in other parts of the program, if you need.
    
    % ---------------------------
    % in figure#10 I will show RGB imagery. I create an empty image item. 
    figure(10) ; clf();  
    hic=image(0); 
    %"hic"  contains the handle of the graphic object, of class image. We
    % will use it later, for refreshing the RGB images.
    axis([1,320,1,240]);
    set(10,'name','RGB');
    title('RGB');
    
    set(gca(), 'xdir', 'reverse'); % we need to tell MATLAB this, 
    % so that the images  (RGB and depth) do not appear mirrowed
    % horizontally, to our eyes.

    % ---------------------------
   % in figure#11 I will show Depth imagery.
    figure(11) ; clf();  
    hir=imagesc(0);  
    axis([1,320,1,240]);
    title('Depth');
    customColormap = gray(128);             % colormap for showing "monochrome" imges using color. In this case, I want to use grayscale.
    colormap(customColormap);
    set(gca(), 'xdir', 'reverse');
    caxis([0 3000]);
    set(11,'name','depth');
    % all these matters are MATLAB stuff. % this code is for showing those images in the way I liked. 
    % you may change that.
    % you may use MATLAB help, or use the web, or ChatGTP, for investigating those capabilities of MATLAB.
    
    % ---------------------------
    % This one, for showing 3D points clouds, in figure #12.
    figure(12); clf(); 
    hp=plot3(0,0,0,'.','markersize',1); title('3D points');    
    % create plot of 3D points, for posterior use.
    
    set(12,'name','3Dpts');
    
    axis([0,1400, -700,+700, -700,+700]);  % .
    % in this example, I used millimeters. but you may scale points to meters, etc. 
    
    xlabel('X');ylabel('Y');zlabel('Z'); grid on;
    % ---------------------------
   
    set(gca(),'Clipping','off');  % to improve presentation, in certain sense,
    % you may investigate this, but it is not relevant.
end
%------------------------------------------------------     
function h = CreateMyButton(strBla,position,CallbackFunction)
    h=uicontrol('Style','pushbutton','String',strBla,'Position',position, 'Callback',CallbackFunction); 
    % for more details:  ask MATLAB help.
    return
end
% ----------------------------------------------------------
function ShowRGBimage(r,h)
    set(h,'cdata',r);  
end
