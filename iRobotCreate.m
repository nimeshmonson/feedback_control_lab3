classdef iRobotCreate < handle
    %IROBOTCREATE A Matlab interface to the iRobotCreate.
    %   This is a wrapper to the iRobotCreate.
    %
    %   Usage:
    %       This creates an interface for Matlab to communicate with the
    %       robot and give commands to the robot. There are two modes in
    %       this: simulation and robot. In simulation mode, it will create a
    %       graphical representation of the robot and respond to the same
    %       commands as a real robot, emulating the behaviour of a robot.
    %       The constructor will, by default, draw the roomba on a figure.
    %       In the robot mode, it will connect with the roomba and
    %       give it commands. The constructor in this mode will connect
    %       with the port defined.
    %
    %       When the constructor is called, it will create handles to the
    %       appropriate functions. Thus, you can run commands in simulation
    %       mode and use similar, if not the same, code in the real mode.
    %
    %       For more information on any method, type the following at the prompt:
    %       help iRobotCreate.methodname
    %
    %   Example constructing object:
    %       Simulation mode:
    %           iRC = iRobotCreate('Hz', 5);
    %           This will create an object iRC in simulation mode with an
    %           update rate of 5 Hz.  The robot will default to 5hz if the
    %           argument is omitted
    %
    %       Robot mode:
    %           iRC = iRobotCreate('Port', 0, 'Version', 2); This will connect
    %           Matlab to a version 2 iRobot Create through the object iRC on
    %           port number 0 (/dev/ttyUSB# in linux, and COM# in windows)
    %           with the default update rate of 5 Hz.
    %
    %   Methods:
    %       iRobotCreate
    %       anglesensor
    %       beep (robot)
    %       connect
    %       comm (robot)
    %       delete
    %       directdrive
    %       distancesensor
    %       drawroomba (simulation)
    %       backward 
    %       forward
    %       stop 
    %       getpose (simulation)
    %       isbumped (robot)
    %       moveroomba (simulation)
    %       resumecontrol (robot)
    %       rotate
    %       left 
    %       right
    %       setnoise (simulation)
    %       setupdaterate (simulation)
    %       setvel
    %       setvelocity 
    %       setworldframe (simulation)
    %       setworkspace (simulation)
    %       trailsize (simulation)
    %   Note: Certain methods are only available in certain modes.

    %The handles to the various functions
    properties
        connect_h;
        comm_h;
        drawroomba_h;
        moveroomba_h;
        resumecontrol_h;
        beep_h;
        setnoise_h;
        setvel_h;
        set_velocity_h; 
        trailsize_h;
        directdrive_h;
        stop_h;
        forward_h;
        backward_h; 
        rotate_h;
        left_h; 
        right_h; 
        isbumped_h;
        setworkspace_h;
        setworldframe_h;
        getpose_h;
        setupdaterate_h;
        distancesensor_h;
        anglesensor_h
        angledistsensor_h;
        delete_h;
    end

    %Counters and such...
    properties (Access = public, Hidden)
        prv_left = 0;
        prv_right = 0;
        angle = 0;
        anglecount = 0;
        autoconnect = 1;
        com;
        comnum;
        dimensions = [-2 12 -2 12];
        distance = 0;
        h_arrow;
        h_body;
        h_trail;
        loc;            %Holds the location of the robot in simulation
        noise = 1;
        sigma_omega = 0;
        sigma_v = 0;
        steps = 0;
        time;
        totalangle = 0;
        totaldist = 0;
        trail;
        traillength = 100;
        update = 5;
        wheelbase = 0.235;
        version = 1;
        worldfr = 1;
    end

    %Constants to be used...
    properties (Constant, Access = private, Hidden)
        v = 0.4;
        vturn = 0.1;
        d = 0.30;
        op = struct( ...
            'Start', 128, ...
            'Baud', 129, ...
            'Control', 130, ...
            'Safe', 131, ...
            'Full', 132, ...
            'Spot', 134, ...
            'Cover', 135, ...
            'Demo', 136, ...
            'Drive', 137, ...
            'LSD', 138, ...
            'LED', 139, ...
            'Song', 140, ...
            'PlaySong', 141, ...
            'Sensors', 142, ...
            'CoverDock', 143, ...
            'PWMLSD', 144, ...
            'DriveDirect', 145, ...
            'DigitalOutput', 147, ...
            'Stream', 148, ...
            'QueryList', 149, ...
            'PauseResumeStream', 150, ...
            'SendIR', 151, ...
            'Script', 152, ...
            'PlayScript', 153, ...
            'ShowScript', 154, ...
            'WaitTime', 155, ...
            'WaitDistance', 156, ...
            'WaitAngle', 157, ...
            'WaitEvent', 158);
    end

    %The method handles
    methods
        function iRC = iRobotCreate(varargin)
            fprintf('Version 30 June 2015 Updated \n'); 
            %IROBOTCREATE   The iRobot Create constructor
            %   Takes property name/value pairs to configure the robot from
            %   the following options, which can be specified in any order:
            %      'Hz': sets the update rate
            %      'Port': the port number to connect to, if no port is
            %           specified, the robot is set to simulation mode.
            %      'Version': the iRobot create version number
            %
            %   In simulation mode, the robot is drawn by default, thus it is
            %   unecessary to use the drawroomba command.  The figure is
            %   initially drawn in the world frame, but this can be changed
            %   with the worldframe function.
            %
            %   In real mode, the robot automatically connects to with the
            %   port, so it is unnecessary to use the connect command.

            % Create a struct containing the names and default values for the
            % constructor arguments
            args = struct('Hz', 10, 'Port', -1, 'Version', 1);

            % Get the acceptable argument names
            arg_names = fieldnames(args);

            % Validate the property name/value pairs given to the constructor
            if mod(nargin, 2) ~= 0
                error('The constructor takes property name/value pairs')
            end
            for pair = reshape(varargin,2,[]) % pair is {name, value}
                name = pair{1};
                if any(strcmp(name, arg_names))
                    % Overwrite the default value for the given pair
                    args.(name) = pair{2};
                else
                    error('%s is not a valid name', name)
                end
            end

            % Validate the update rate
            if 1 > args.Hz || args.Hz > 20
                error('irc:irobotcreate', ['the update rate of the roomba',...
                    'must be between 1 and 20']);
            else
                iRC.update = args.Hz;
            end
            
            % Validate version number
           
            if ~(args.Version == 1 || args.Version ==2)
                error('Invalid version number')
            else
                iRC.version = args.Version;
            end
      

            % Check for simulation mode; if no port is specified, the robot is
            % set to simulation mode
            sim = 1;
            if args.Port ~= -1
              
                sim = 0;
            end

            if sim
                % Assigns the handles to point to the set of functions
                % specific to simulation mode
                iRC.connect_h = @iRC.connect_sim;
                iRC.drawroomba_h = @iRC.drawroomba_sim;
                iRC.moveroomba_h = @iRC.moveroomba_sim;
                iRC.setnoise_h = @iRC.setnoise_sim;
                iRC.trailsize_h = @iRC.trailsize_sim;
                iRC.setworkspace_h = @iRC.setworkspace_sim;
                iRC.setworldframe_h = @iRC.setworldframe_sim;
                iRC.getpose_h = @iRC.getpose_sim;
                iRC.setupdaterate_h = @iRC.setupdaterate_sim;
                iRC.setvel_h = @iRC.setvel_sim;
                iRC.set_velocity_h = @iRC.set_velocity_sim; 
                iRC.directdrive_h = @iRC.directdrive_sim;
                iRC.forward_h = @iRC.forward_sim;
                iRC.backward_h = @iRC.backward_sim; 
                iRC.rotate_h = @iRC.rotate_sim;
                iRC.left_h = @iRC.left_sim; 
                iRC.right_h = @iRC.right_sim; 
                iRC.distancesensor_h = @iRC.distancesensor_sim;
                iRC.anglesensor_h = @iRC.anglesensor_sim;
                iRC.delete_h = @iRC.delete_sim;

                % Draws the roomba figure
                iRC.drawroomba_h();
            else
                % Assigns the handles to point to the set of functions
                % specific to real mode
                iRC.connect_h = @iRC.connect_real;
                iRC.resumecontrol_h = @iRC.resumecontrol_real;
                iRC.beep_h = @iRC.beep_real;
                iRC.setvel_h = @iRC.setvel_real;
                iRC.set_velocity_h = @iRC.set_velocity_real; 
                iRC.directdrive_h = @iRC.directdrive_real;
                iRC.stop_h = @iRC.stop_real; 
                iRC.forward_h = @iRC.forward_real;
                iRC.backward_h = @iRC.backward_real; 
                iRC.rotate_h = @iRC.rotate_real;
                iRC.left_h = @iRC.left_real; 
                iRC.right_h = @iRC.right_real; 
                iRC.isbumped_h = @iRC.isbumped_real;
                if args.Version == 1
                    iRC.wheelbase = 0.26;
                    iRC.distancesensor_h = @iRC.distancesensor_real;
                    iRC.anglesensor_h = @iRC.anglesensor_real;
                else
                    wheelbase = 0.235;
                    iRC.distancesensor_h = @iRC.invalid_distancesensor_real;
                    iRC.anglesensor_h = @iRC.invalid_anglesensor_real;
                end
                iRC.angledistsensor_h = @iRC.angledistsensor_real;
                iRC.delete_h = @iRC.delete_real;
                  % Assigns the port to be used
                iRC.comnum = args.Port;
                if iRC.autoconnect
                   iRC.connect();
                end
            end
            
            % Starts a timer to ensure the update rate is maintained
            iRC.time = tic;
        end

        function connect(iRC)
            %CONNECT Connects with the robot
            %   Only available in robot mode.
            iRC.connect_h();
        end

        function resumecontrol(iRC)
            %RESUMECONTROL Resumes control if the robot was picked up
            %   Only available in robot mode.
            iRC.resumecontrol_h();
        end

        function drawroomba(iRC, varargin)
            %DRAWROOMBA Draws the Roomba to the figure.
            %   This may take in an argument that draws the Roomba to the
            %   position specified at [x, y, theta]. If there is no
            %   argument, it will draw the Roomba at [0, 0, 0].
            iRC.drawroomba_h(varargin)
        end

        function moveroomba(iRC, pose)
            %MOVEROOMBA Moves the Roomba to position [x, y, theta].
            %   This requires an argument that specifies [x, y, theta].
            %   Only available in simulation mode.
            iRC.drawroomba_h(pose)
        end

        function beep(iRC)
            % BEEP Make the robot beep, like on startup.
            %   This function is only available in robot mode.
            iRC.beep_h();
        end

        function setnoise(iRC, varargin)
            % SETNOISE Sets the amount of noise to be added to the simulation
            %   The simulation will default to no noise, but
            %   this function will add noise to the simulation.
            %   Takes two arguments. First the noise to be added
            %   to the linear velocity, second the noise to be added
            %   to the angular velocity. If these are not specified,
            %   they will default to the values (0.0152, 0.5098). This
            %   function is only available in simulation mode


            if (nargin == 3)
                iRC.setnoise_h(varargin{1}, varargin{2});
            elseif (nargin == 1)
                %Inputs the measured values.
                iRC.setnoise_h(0.0152 , 0.5098);
            else
                error('iRC:noise', ['SETNOISE requires two inputs - one for', ...
                    ' the linear velocity, another for the angular velocity.', ...
                    ' If nothing is inputted, it will default to previously', ...
                    ' measured values.']);
            end
        end

        function setvel(iRC, v, omega)
            % SETVEL Set the robots linear and angular velocity.
            %   Takes two arguments. First the linear velocity in m/s,
            %   second the angular velocity in radian/s. The seperate wheel
            %   speeds will then be set. Wheel speed is limited to 0.5m/s
            %   in either direction.
           iRC.setvel_h(v, omega);
        end
        
        
        function set_velocity(iRC, v_cm, omega_deg)
           % This method is for outreach purposes (PGSET)
           % This method takes velocity in cm/s and deg 
           % Converts to meters and radians to call setvel method
            iRC.set_velocity_h(v_cm, omega_deg); 
        end 

        function directdrive(iRC, v1, v2)
            % DIRECTDRIVE Set wheel velocities directly.
            %   Most likely used for direct driving of the robot.
            iRC.directdrive_h(v1, v2);
        end
        
        function stop(iRC)
            iRC.stop_h(); 
        end

        function hasstopped = forward(iRC,d)
            % FORWARD Move the robot forward.
            %   Takes one argument, the distance to move in inches. A
            %   negative distance will make the robot move backwards.
            %   Blocks until finished, uses the setvel method.
            hasstopped=iRC.forward_h(d);
        end
        
        function backward(iRC, cm)
            % BACKWARD Move the robot backward. 
            %  Takes one argument, the distance to move in inches. Converts 
            %  it into a negative number and calls the forward function
            iRC.backward_h(cm); 
        end

        function rotate(iRC, angle)
            % ROTATE Rotate the robot.
            %   Takes one argument, the angle to turn in degrees. A
            %   positive angle means anticlockwise rotation, a negative
            %   angle means clockwise rotation. The angle should be in the
            %   range of -pi to pi, if it is not, it will be translated to
            %   this range. Blocks until finished, uses the setvel method.
            iRC.rotate_h(angle);
        end
        
        function left(iRC, degrees)
            % LEFT Turns the robot to the left. 
            %  Takes one argument,the angle to turn in degrees. 
            %  Calls the rotate function to turn. 
            iRC.left_h(degrees);
        end
        
        function right(iRC, degrees)
            %  RIGHT Turns the robot to the right. 
            %   Takes one argument, the angle to turn in degrees
            %   Calls the rotate funtion and passed it a negative argument 
            iRC.right_h(degrees);
        end
        
        function bool = isbumped(iRC)
            % ISBUMPED Checks the front bumper for a bump.
            %   Takes no arguments. Only detects bumper bumps at a given
            %   instant using the sensor. This function is only available
            %   in robot mode.
            bool = iRC.isbumped_h();
        end

        function setworkspace(iRC, space)
            % SETWORKSPACE Takes in the bounds of the workspace
            %   This will change the bounds of the workspace. This must be
            %   inputted in the form ([XMin XMax YMin YMax]). Only
            %   available in simulation mode
            iRC.setworkspace_h(space);
        end

        function setworldframe(iRC, frame)
            % SETWORLDFRAME Changes frame to world frame.
            %   If the input to this function is 1, the frame will change
            %   to the world frame. Otherwise, it will be in robot frame.
            %   This function is only available in simulation mode.
            iRC.setworldframe_h(frame);
        end

        function pose = getpose(iRC)
            %GETPOSE Returns the [x, y, theta] of the robot.  This is
            %   useful for evaluating control/planning algorithms with
            %   perfect perception. Only available in simulation mode.
            %   Added by JRS, 22 Mar 2012
            pose = iRC.getpose_h();
        end

        function setupdaterate(iRC, update)
            % SETUPDATERATE Sets the update rate.
            %   This function sets the update rate. Initially, the update
            %   rate defaults to 5 Hz, if not defined within the
            %   constructor.
            iRC.setupdaterate_h(update);
        end

        function trailsize(iRC, newlength)
            % TRAILSIZE Changes the trail length to the inputted number.
            %   Takes 1 argument, the new trail length. The trail length
            %   defaults to 100. However, this function can change that. If
            %   the user inputs inf as the trail length, the trail length
            %   will be infinite. This function is only available in
            %   simulation mode.
            iRC.trailsize_h(newlength);
        end

        function d = distancesensor(iRC)
            % DISTANCESENSOR Gets the distance from the distance sensor.
            %   Takes no arguments. The distance sensor is reset every time
            %   a value is read from it.
            d = iRC.distancesensor_h();
        end

        function a = anglesensor(iRC)
            % ANGLESENSOR Gets the angle from the angle sensor.
            %   Takes no arguments. The angle sensor is reset every time a
            %   value is read from it.
            a = iRC.anglesensor_h();
        end

        function [a, d] = angledistsensor(iRC)
            % ANGLEDISTSENSOR Gets the angle from the angle and distance from
            %   the encoder values. Takes no arguments. The angle and distance
            %   sensor is reset every time a value is read from it.
            [a, d] = iRC.angledistsensor_h();
        end
        function delete(iRC)
            %DELETE Deconstructs the object.
            iRC.delete_h();
        end
    end

    methods (Hidden, Access = private)
        function connect_sim(iRC)
            % CONNECT The GUI constructor
            %   Draws the roomba, in the event that this is accidentally
            %   called during simulation mode.
            drawroomba_sim(iRC);
        end

        function drawroomba_sim(iRC, varargin)
            % DRAWROOMBA The GUI constructor
            %   Creates the roomba object and graphs it at the location
            %   specified by the roomba. If there is no input, it will
            %   graph it at the origin with no angle. Usually, this
            %   function does not need to be called.

            iRC.steps = iRC.steps + 1;
            scale = .5; % the scale (adjust how big it appears) %update on 6/8/2015 from 1 to .05

            rad = 0.3; % the radius
            N = 60; % number of lines in the circle approximation
            t = 0:2*pi/N:2*pi;
            r.const_body_x = rad * cos(t) * scale;
            r.const_body_y = rad * sin(t) * scale;

            const_arrow_x = [-.8 * rad , -.8 * rad , rad * 1] .* scale;
            const_arrow_y = [0.15, -0.15, 0] .* scale;

            % Checks if there was a previously drawn image, by seeing if
            % the pose was passed into the DRAWROOMBA.

            if (nargin > 1)
                pose = varargin{1};
            else
                pose = [0,0,0];
            end

            % Keeps track of the location and angle here.
            iRC.loc = [pose(1) pose(2)];
            iRC.angle = pose(3);

            % The arrow is rotated around the body to indicate direction

            new_arrow_x = [cos(pose(3)), -sin(pose(3))] * ...
                [const_arrow_x; const_arrow_y];
            new_arrow_y = [sin(pose(3)), cos(pose(3))] * ...
                [const_arrow_x; const_arrow_y];

            % The body need not be rotated, as it is a circle.

            poly_body_x = r.const_body_x + pose(1);
            poly_body_y = r.const_body_y + pose(2);

            % The arrow is moved to the new location

            poly_arrow_x = new_arrow_x + pose(1);
            poly_arrow_y = new_arrow_y + pose(2);

            % Checks if there are as many points as trail length. If there are
            % deletes the first, so another can be added.

            if iRC.traillength < inf
                if gt(iRC.steps, iRC.traillength)
                    iRC.trail = iRC.trail(2:iRC.traillength,:);
                    iRC.trail(iRC.traillength, :) = [pose(1), pose(2)];
                else
                    iRC.trail(iRC.steps,:) = [pose(1), pose(2)];
                end
            else
                iRC.trail(iRC.steps,:) = [pose(1), pose(2)];
            end


            % Plots the trail and body.

            if (iRC.steps == 1)
                axis (iRC.dimensions);
                axis square;
                hold on;
                iRC.h_trail = plot(iRC.trail(:,1),iRC.trail(:,2),'k.');
                iRC.h_body = fill(poly_body_x, poly_body_y, 'w');
                iRC.h_arrow = fill(poly_arrow_x, poly_arrow_y, 'r');
            end

            % With each redrawing, the axis are changed depending on
            % whether it should be drawn in the world frame or robot
            if iRC.worldfr
                axis (iRC.dimensions);
            else
                % Centers the figure on the robot
                newaxis = ([(iRC.loc(1) - 1/2*(iRC.dimensions(2) - iRC.dimensions(1)))...
                    (iRC.loc(1) + 1/2*(iRC.dimensions(2) - iRC.dimensions(1)))...
                    (iRC.loc(2) - 1/2*(iRC.dimensions(4) - iRC.dimensions(3)))...
                    (iRC.loc(2) + 1/2*(iRC.dimensions(4) - iRC.dimensions(3)))]);
                axis (newaxis);
            end

            % Does not redraw the figure, sets the data to a new coordinate
            set(iRC.h_trail, 'XData', iRC.trail(:,1), 'YData', iRC.trail(:,2));
            set(iRC.h_arrow, 'XData', poly_arrow_x, 'YData', poly_arrow_y);
            set(iRC.h_body, 'XData', poly_body_x, 'YData', poly_body_y);

        end

        function moveroomba_sim(iRC, pose)
            % MOVEROOMBA Moves the graphical Roomba from the current spot to
            %   the specified position.
            drawroomba_sim(iRC, pose);
        end

        function setnoise_sim(iRC, noise_v, noise_omega)
            % SETNOISE Sets the amount of noise in the simulation
            %   Can take two arguments, noise on linear velocity
            %   and noise on angular velocity. The default values for this
            %   function should be (0.0152, .5098)

            % Sets the noise in the object
            iRC.sigma_v = noise_v;
            iRC.sigma_omega = noise_omega;
        end
        
        function setvel_sim(iRC, v, omega)
            % SETVEL Sets the velocity of the simulation Roomba.
            %   Takes two arguments, linear and angular velocity.
            %   The speeds cannot exceed 0.5 m/s.

            % Checks if the velocity is too large for the robot to handle
            wheel = int16([0.5, 0.5; 1/iRC.wheelbase, -1/iRC.wheelbase] ...
                \ [v; omega] .* 1000);
            if -500 > wheel(1) || wheel(1) > 500 || ...
                    -500 > wheel(2) || wheel(2) > 500
                error('iRC:setvel', ['the speed of each wheel cannot', ...
                    ' exceed 0.5m/s (consider both v and omega)']);
            end
            % Measures the time elapsed so the simulation is true to the update rate
            telapsed = toc(iRC.time);

            % Corrupt the velocities if necessary
            % sigma_v and sigma_omega are 0 by default
            v_hat = v + ( iRC.sigma_v * v * randn);
            omega_hat = omega + ( iRC.sigma_omega * omega * randn );

            % Calculate the displacement in the robot frame and the rotation
            dx = v_hat/iRC.update;
            dtheta = omega_hat/iRC.update;

            % Calculate the displacement in the world frame
            xdisp = dx*cos( iRC.angle + dtheta/2 );
            ydisp = dx*sin( iRC.angle + dtheta/2 );

            % Update the robot pose
            pose = [(xdisp + iRC.loc(1)), (ydisp + iRC.loc(2)), (dtheta + iRC.angle)];

            % Update the robot counters
            iRC.distance = iRC.distance + sqrt(xdisp^2 + ydisp^2);
            iRC.anglecount = iRC.anglecount + dtheta;
            iRC.totaldist = sqrt(xdisp^2 + ydisp^2) + iRC.totaldist;
            iRC.totalangle = iRC.totalangle + dtheta;
            iRC.angle = iRC.angle + dtheta;

            % Move the robot
            iRC.moveroomba(pose);

            % Pause to simulate real-time ops and grab the new time
        % Blocks until the update rate wanted is achieved.
            pause((1/iRC.update - telapsed));
            iRC.time = tic;
        end
        
        function set_velocity_sim(iRC, v_cm, omega_deg)
        % This method is for outreach purposes (PGSET)
        % This method takes velocity in cm/s and deg 
        % Converts to meters and radians to call setvel method
            v_m = v_cm * .01; 
            omega_r = omega_deg * pi/180; 
            iRC.setvel(v_m,omega_r); 
        end
        
        function directdrive_sim(iRC, v1, v2)
            % DIRECTDRIVE Sets the differential velocity of the Roomba.
            %   Takes two arguments, one for each wheel. This translates
            %   the velocities to a linear and angular velocity.

            % Convert the velocity of each wheel into a linear and angular velocity
            linearv = (v1 + v2)/2;
            angv = (v1 - v2)/iRC.wheelbase;

            % Determines if the velocities are too big
            wheel = int16([0.5, 0.5; 1/iRC.wheelbase, -1/iRC.d] ...
                \ [linearv ; angv] .* 1000);

            if -500 > wheel(1) || wheel(1) > 500 || ...
                    -500 > wheel(2) || wheel(2) > 500
                error('iRC:directdrive', ['the speed of each wheel cannot', ...
                    ' exceed 0.5m/s (consider both v and omega)']);
            end


            % Setvel command is run now that we know the linear and angular velocity
            iRC.setvel(linearv,angv);
        end

        function hasstopped = forward_sim(iRC, d)
            % FORWARD Makes the robot move forward.
            %   Takes one argument, the distance to move forward. A
            %   positive distance means it is moving forward. A negative
            %   distance indicates movement backwards. Blocks until
            %   finished, uses the setvel method
            
           % Takes cm and converts to meters 
            d = d * 0.01; 
            hasstopped = []; 
            % check distance
            if abs(d) > 10
                error('iRC:forward', 'distance is too large');
            end

            %Finds final destination of this command
            finald = [(d*cos(iRC.angle)), (d*sin(iRC.angle))];

            displacement = 0;

            % Sets the velocity of the forward command
            speed = 0.5;
            % Runs the setvel command at 0.5 m/s until it has gone the correct distance.
            while ((norm(finald) - 0.01) > displacement)

                    %Finds current traversed distance to be used for counter
                    initdist = iRC.totaldist;
                    iRC.setvel(sign(d)*speed, 0);

                    % Update counters
                    displacement = displacement + iRC.totaldist - initdist;
                    
            end
        end
   
        
        function backward_sim(iRC, cm)
            cm = -1 * cm; 
            % BACKWARD Makes the robot move forward.
            %Converts d into meters from cm (input is in cm) 
            iRC.forward(cm); 
        end

        function rotate_sim(iRC, angle)
            % ROTATE Rotate the robot.
            %   Takes one argument, the angle to turn in radians. A
            %   positive angle means anticlockwise rotation, a negative
            %   angle means clockwise rotation. The angle should be in the
            %   range of -pi to pi, if it is not, it will be translated to
            %   this range. Blocks until finished, uses the setvel method.

            %Convert the angle to the simplest form.
            while pi < angle
                angle = angle - 2 * pi;
            end

            while angle < -pi
                angle = angle + 2 * pi;
            end

            % Calculates the final angle
            finalangle = iRC.anglecount + angle;

            % If the angle is positive, it will spin in the ccw direction
            if (angle > 0)
                while (finalangle > (iRC.anglecount + 0.01))
                    iRC.setvel(0, pi/8);
                end

            % If the angle is negative, it will spin in the cw direction
            else
                while (finalangle < (iRC.anglecount - 0.01))
                    iRC.setvel(0, -pi/8);
                end
            end
        end
        
        function left_sim(iRC, degrees) 
            % Left Turns the robot left 
            theta = degrees * pi/180; 
            iRC.rotate(theta);
        end
        
        function right_sim(iRC, degrees) 
            % Right Turns the robot right
            theta =  -1 * degrees * pi/180; 
            iRC.rotate(theta);  
        end
        
        function setworkspace_sim(iRC, dimensions)
            % SETWORKSPACE Takes in the bounds of the workspace
            %   This will change the bounds of the workspace. This must be
            %   inputted in the form ([XMin XMax YMin YMax])
            %   If nothing is inputted, it will default to the original
            %   workspace ([-2 12 -2 12])

            if (size(dimensions) == [1 4])
                iRC.dimensions = dimensions;
                axis(iRC.dimensions);
            else
                error('iRC:setworkspace', ['The inputted workspace is ', ...
                    'invalid. It must be in the form [XMin XMax YMin', ...
                    ' YMax].']);
            end
        end

        function setworldframe_sim(iRC, worldframe)
            % SETWORLDFRAME Changes frame to world frame.
            %   If the input to this function is 1, the frame will change
            %   to the world frame. Otherwise, it will be in robot frame.
            %   The default frame is the world frame.

            % Allows this to be accessible to the robot object
            iRC.worldfr = worldframe;

            % If the worldframe is active, it will change the axis to what
            % the user has defined, or else it will center on the robot.
            if worldframe
                axis (iRC.dimensions);
            else
                axis equal;
            end
        end

        function pose = getpose_sim(iRC)
            % GETPOSE Returns the robot's position and orientation.
            %   Added by JRS, 22 Mar 2012
          pose = [iRC.loc iRC.angle];
        end

        function setupdaterate_sim(iRC, update)
            % SETUPDATERATE Sets the update rate.
            %   This function sets the update rate. Initially, the update
            %   rate defaults to 5 Hz, if not defined within the
            %   constructor.

            % The update rate must be set between 1 and 20
                if 1 > update || update > 20
                error('iRC:setupdaterate', ['The update rate of the Roomba',...
                    'must be between 1 and 20']);
                end
            iRC.update = update;
        end

        function trailsize_sim(iRC, newlength)
            % TRAILSIZE Changes the trail length to the inputted number.
            %   Takes 1 argument, the new trail length. The trail length
            %   defaults to 100. However, this function can change that. If
            %   the user inputs inf as the trail length, the trail length
            %   will be infinite.

            iRC.traillength = newlength;
        end

        function d = distancesensor_sim(iRC)
            % DISTANCESENSOR Returns the distance the Roomba has moved since
            %   the last time the function was called.
            d = iRC.distance;
            iRC.distance = 0;
        end

        function a = anglesensor_sim(iRC)
            % ANGLESENSOR Returns the angle the Roomba has rotated since the
            %   the last time the function was called.

            a = iRC.anglecount;
            iRC.anglecount = 0;
        end

        function delete_sim(iRC)
            % DELETE Deconstructs the GUI

            delete(iRC.h_trail);
            delete(iRC.h_body);
            delete(iRC.h_arrow);
        end

        function connect_real(iRC)
            % CONNECT Connects with the robot
            try

                % Adjust the port name to reflect the operating system
                if (strcmp('GLNX86',computer) || strcmp('GLNXA64',computer))
                    % Linux
                    name = strcat('/dev/ttyUSB', round(num2str(iRC.comnum)));
                else
                    % Windows
                    name = strcat('COM', round(num2str(iRC.comnum)));
                end

                % check port availability
                if ~isempty(instrfind('port', name, 'status', 'open'))
                    error('iRC:init', ['serial port does not exist', ...
                        ' or is already in use']);
                end
                % create port
                iRC.com = serial(name, ...
                    'BaudRate',  115200 , ... %57600  115200
                    'Terminator','LF', ...
                    'InputBufferSize',100, ...
                    'Timeout', 0.1, ...
                    'ByteOrder','bigEndian', ...
                    'Tag', 'Roomba');
                % open the port
                fopen(iRC.com);
                % start the bot
                fwrite(iRC.com, [iRC.op.Start]);
                pause(0.3);
                % set control
                fwrite(iRC.com, [iRC.op.Safe]);
                pause(0.3);
                % light LEDs
                fwrite(iRC.com, [iRC.op.LED, 10, 0, 128]);
                pause(0.3);
                % select song
                fwrite(iRC.com, [iRC.op.Song, 1, 1, 48, 20]);
                pause(0.3);
                % play song
                fwrite(iRC.com, [iRC.op.PlaySong, 1]);
                pause(0.3);

                % initialize the encoder values
                if iRC.version == 2
                    warning off MATLAB:serial:fread:unsuccessfulRead;
                    fwrite(iRC.com, [iRC.op.Sensors, 101]);
                    out = fread(iRC.com, 28, 'uint8');
                    while isempty(out)
                        out = fread(iRC.com, 28, 'uint8');
                    end
                    iRC.prv_left = iRC.mergebytes16(out(1), out(2));
                    iRC.prv_left = double(iRC.invtwoscompl16(iRC.prv_left));
                    iRC.prv_right = iRC.mergebytes16(out(3), out(4));
                    iRC.prv_right =  double(iRC.invtwoscompl16(iRC.prv_right));
                    warning on MATLAB:serial:fread:unsuccessfulRead;
                end

            catch err
                if ~isempty(iRC.com)
                    fclose(iRC.com);
                end
                rethrow(err);
            end
                 
        end

        function resumecontrol_real(iRC)
            %RESUMECONTROL Resumes control if the robot was picked up

            % set control
            fwrite(iRC.com, [iRC.op.Safe]);
            pause(0.1);
            % light LEDs
            fwrite(iRC.com, [iRC.op.LED, 10, 0, 128]);
            pause(0.1);
        end

        function beep_real(iRC)
            % BEEP Make the robot beep, like on startup.

            % select song
            fwrite(iRC.com, [iRC.op.Song, 1, 1, 48, 20]);
            pause(0.1);
            % play song
            fwrite(iRC.com, [iRC.op.PlaySong, 1]);
            pause(0.1);
        end

        function setvel_real(iRC, v, omega)
            % SETVEL Set the robots linear and angular velocity.
            %   Takes two arguments. First the linear velocity in m/s,
            %   second the angular velocity in radian/s. The seperate wheel
            %   speeds will then be set. Wheel speed is limited to 0.5m/s
            %   in either direction.

            % find wheel velocities
            A =[0.5, 0.5; 1/iRC.wheelbase, -1/iRC.wheelbase];
            wheel = ([0.5, 0.5; 1/iRC.wheelbase, -1/iRC.wheelbase] ...
                \ [v; omega] .* 1000);
%            fprintf('wheel1: %d and wheel2: %d;\n',wheel(1),wheel(2));
            if -500 > wheel(1) || wheel(1) > 500 || ...
                    -500 > wheel(2) || wheel(2) > 500
         
                if abs(wheel(1)) > abs(wheel(2))
                    alpha = 500/abs(wheel(1)); 
                    wheel = (alpha * wheel); 
                    
                else
                    alpha = 500/abs(wheel(2));
                    wheel = (alpha * wheel); 
                end
                    
                x = A * double(wheel);
                x = x * 0.001;
                
                warning('wheel velocities are being scaled down.'); 
                fprintf('New omega is %f rad/s and new velocity is %f m/s\n',x(2),x(1)); 
                %error('iRC:setvel', ['the speed of each wheel cannot', ...
                 %   ' exceed 0.5m/s (consider both v and omega)']);
            end
            wheel = int16(wheel);

            % calculate wheel velocities
            wheel = iRC.twoscompl16(wheel);
            w1h = iRC.highbyte(wheel(1));
            w1l = iRC.lowbyte(wheel(1));
            w2h = iRC.highbyte(wheel(2));
            w2l = iRC.lowbyte(wheel(2));

            % Time elapsed since last setvel and/or robot creation
            telapsed = toc(iRC.time);

            % Pause for remaining time in last update to enforce update
            % rate 
            pause((1/iRC.update - telapsed));

            % Set new update
            iRC.time = tic;

            % write to serial port
            fwrite(iRC.com, [iRC.op.DriveDirect, w1h, w1l, w2h, w2l]);
        end
        
        function set_velocity_real(iRC, v_cm, omega_deg)
        % This method is for outreach purposes (PGSET)
        % This method takes velocity in cm/s and deg 
        % Converts to meters and radians to call setvel method
            v_m = v_cm * 0.01;  
            omega_r = omega_deg * pi/180; 
            iRC.setvel(v_m,omega_r); 
        end

        function directdrive_real(iRC, v1, v2)
            % DIRECTDRIVE Set wheel velocities directly.
            %   Most likely used for direct driving of the robot.

            wheel = int16([v1 * 1000, v2 * 1000]);
            if -500 > wheel(1) || wheel(1) > 500 || ...
                    -500 > wheel(2) || wheel(2) > 500
                error('iRC:directdrive', ['the speed of each wheel', ...
                    'cannot exceed 0.5m/s']);
            end

            % calculate wheel velocities
            wheel = iRC.twoscompl16(wheel);
            w1h = iRC.highbyte(wheel(1));
            w1l = iRC.lowbyte(wheel(1));
            w2h = iRC.highbyte(wheel(2));
            w2l = iRC.lowbyte(wheel(2));
            % write to serial port
            fwrite(iRC.com, [iRC.op.DriveDirect, w2h, w2l, w1h, w1l]);
        end
        
        function stop_real(iRC)
            iRC.setvel(0,0); 
        end

        function hasstopped = forward_real(iRC, d)
            % FORWARD Move the robot forward.
            %   Takes one argument, the distance to move in meters. A
            %   negative distance will make the robot move backwards.
            %   Blocks until finished, uses the setvel method.

            % resume control in case of pickup
            hasstopped = 0; 
            iRC.resumecontrol();
            % Takes cm and converts to meters 
            d = d * 0.01; 
            % check distance
            if abs(d) > 10
                error('iRC:forward', 'distance is too large');
            end
            % stop the robot
            %fwrite(iRC.com, [iRC.op.Drive, 0, 0, 0, 0]);

            % set v
            v = iRC.twoscompl16(iRC.v * sign(d) * 1000);
            % initialize distance
            dr = 0;
            if iRC.version == 1
                iRC.distancesensor();
            else
                iRC.angledistsensor();
            end

            % drive forward (or backwards)
            fwrite(iRC.com, [iRC.op.Drive, iRC.highbyte(v), iRC.lowbyte(v), 128, 0]);
          
            % loop checking distance
            while abs(dr) < abs(d)
                if iRC.version == 1
                    dr = dr + iRC.distancesensor();
                else
                    [alpha, dist] = iRC.angledistsensor();
                    dr = dr + dist;
                end
                hasstopped=iRC.isbumped(); 
                if hasstopped, break; end
            end
            % stop the robot
           % iRC.setvel(0,0);
            fwrite(iRC.com, [iRC.op.Drive, 0, 0, 0, 0]); 
        end
        
        function backward_real(iRC, d)
            % BACKWARD moves the robot backward. 
               iRC.resumecontrol();
            % Takes cm and converts to meters 
             d = -1 * d * 0.01; 
            % check distance
            if abs(d) > 10
                error('iRC:forward', 'distance is too large');
            end
            % stop the robot
            %fwrite(iRC.com, [iRC.op.Drive, 0, 0, 0, 0]);

            % set v
            v = iRC.twoscompl16(iRC.v * sign(d) * 1000);
            % initialize distance
            dr = 0;
            if iRC.version == 1
                iRC.distancesensor();
            else
                iRC.angledistsensor();
            end

            % drive forward (or backwards)
            fwrite(iRC.com, [iRC.op.Drive, iRC.highbyte(v), iRC.lowbyte(v), 128, 0]);

            % loop checking distance
            while abs(dr) < abs(d)
                if iRC.version == 1
                    dr = dr + iRC.distancesensor();
                else
                    [alpha, dist] = iRC.angledistsensor();
                    dr = dr + dist;
                end
                if iRC.isbumped(), break; end
            end
            % stop the robot
            fwrite(iRC.com, [iRC.op.Drive, 0, 0, 0, 0]);
        end

        function rotate_real(iRC, angle)
            % ROTATE Rotate the robot.
            %   Takes one argument, the angle to turn in radians. A
            %   positive angle means anticlockwise rotation, a negative
            %   angle means clockwise rotation. The angle should be in the
            %   range of -pi to pi, if it is not, it will be translated to
            %   this range. Blocks until finished, uses the setvel method.

            % resume control in case of pickup
            iRC.resumecontrol();
            % shift the angle into -pi to pi
            while pi < angle
                angle = angle - 2 * pi;
            end

            while angle < -pi
                angle = angle + 2 * pi;
            end

            % get direction
            if angle < 0

                % turn clockwise
                turn = [255, 255];
            else

                % turn anticlockwise
                turn = [0, 1];
            end

            % set v
            v = iRC.twoscompl16(iRC.vturn * 1000);

            % initialize angle
            angler = 0;
            if iRC.version == 1
                iRC.anglesensor();
            else
                iRC.angledistsensor();
            end

            % rotate
            fwrite(iRC.com, [iRC.op.Drive, iRC.highbyte(v), iRC.lowbyte(v), turn]);

            % loop checking
            while abs(angler) < abs(angle)
                if iRC.version == 1
                   angler = angler + iRC.anglesensor();
               else
                    [alpha, dist] = iRC.angledistsensor();
                    angler = angler + alpha;
                end
                if iRC.isbumped(), break; end
            end
            % stop the robot
            fwrite(iRC.com, [iRC.op.Drive, 0, 0, 0, 0]);
        end
        
        function left_real(iRC, degrees)
            % LEFT Turns the robot left. 
            theta = degrees * pi/180; 
            iRC.rotate(theta); 
        end
        
        function right_real(iRC, degrees)
            % Right Turns the robot right. 
            theta = -1 * degrees * pi/180; 
            iRC.rotate(theta); 
        end

        function bool = isbumped_real(iRC)
            % ISBUMPED Checks the front bumper for a bump.
            %   Takes no arguments. Only detects bumper bumps at a given
            %   instant using the sensor.

            warning off MATLAB:serial:fread:unsuccessfulRead;
            fwrite(iRC.com, [iRC.op.Sensors, 7]);
            out = fread(iRC.com, 1, 'uint8');
            while isempty(out)
                out = fread(iRC.com, 1, 'uint8');
            end
            if bitand(out, uint8(3))
                bool = 1;
            else
                bool = 0;
            end
            warning on MATLAB:serial:fread:unsuccessfulRead;
        end

        function d = distancesensor_real(iRC)
            % DISTANCESENSOR Gets the distance from the distance sensor.
            %   Takes no arguments. The distance sensor is reset every time
            %   a value is read from it.

            warning off MATLAB:serial:fread:unsuccessfulRead;
            fwrite(iRC.com, [iRC.op.Sensors, 19]);
            out = fread(iRC.com, 2, 'uint8');
            while isempty(out)
                out = fread(iRC.com, 2, 'uint8');
            end
            d = iRC.mergebytes16(out(1), out(2));
            d = double(iRC.invtwoscompl16(d)) / 1000;
            warning on MATLAB:serial:fread:unsuccessfulRead;
        end

        function a = anglesensor_real(iRC)
            % ANGLESENSOR Gets the angle from the angle sensor.
            %   Takes no arguments. The angle sensor is reset every time a
            %   value is read from it.

            warning off MATLAB:serial:fread:unsuccessfulRead;
            fwrite(iRC.com, [iRC.op.Sensors, 20]);
            out = fread(iRC.com, 2, 'uint8');
            while isempty(out)
                out = fread(iRC.com, 2, 'uint8');
            end
            a = iRC.mergebytes16(out(1), out(2));
            a = double(iRC.invtwoscompl16(a)) * pi / 180;
            warning on MATLAB:serial:fread:unsuccessfulRead;
        end

        function [a, d] = angledistsensor_real(iRC)
            % ANGLEDISTSENSOR Gets the angle from the angle and distance from
            %   the encoder values. Takes no arguments. The angle and distance
            %   sensor is reset every time a value is read from it.
            if iRC.version == 1
                error('angledistsensor not compatible with version 1')
            end

            warning off MATLAB:serial:fread:unsuccessfulRead;
            fwrite(iRC.com, [iRC.op.Sensors, 101]);
            out = fread(iRC.com, 28, 'uint8');
            while isempty(out)
                out = fread(iRC.com, 28, 'uint8');
            end

            left = iRC.mergebytes16(out(1), out(2));
            left = double(iRC.invtwoscompl16(left));
            right = iRC.mergebytes16(out(3), out(4));
            right =  double(iRC.invtwoscompl16(right));
            leftcount = iRC.rollover(left-iRC.prv_left);
            rightcount = iRC.rollover(right-iRC.prv_right);

            l_wheel = leftcount * (pi * 72.0/508.8);
            r_wheel = rightcount * (pi * 72.0/508.8);
            d = ((l_wheel+ r_wheel)/2)*.001;
            a = (r_wheel - l_wheel)/(iRC.wheelbase*1000);
            iRC.prv_left=left;
            iRC.prv_right=right;
            warning on MATLAB:serial:fread:unsuccessfulRead;
        end

        function d = invalid_distancesensor_real(iRC)
            error('distancesensor not compatible with iRobot Create Version 2')
        end

        function a = invalid_anglesensor_real(iRC)
            error('anglesensor not compatible with iRobot Create Version 2')
        end

        function delete_real(iRC)
            % DELETE Deconstructs the object
            if ~isempty(iRC.com)
                fclose(iRC.com);
            end
        end
    end

    methods (Access = public, Static)
        function n = mergebytes16(hb, lb)
            n = uint16(lb);
            n = n + bitshift(uint16(hb), 8);
        end
        function h = highbyte(word)
            h = bitshift(uint16(word), -8);
        end
        function l = lowbyte(word)
            l = bitand(uint16(word), 255);
        end
        function out = twoscompl16(in)
            in = double(in);
            valid = logical((-32768 <= in) .* (in <= 32767));
            in(~valid) = 0;
            mask = in < 0;
            in(mask) = 65535 + in(mask) + 1;
            out = uint16(in);
        end
        function out = invtwoscompl16(in)
            in = double(in);
            valid = logical((0 <= in) .* (in <= 65535));
            in(~valid) = 0;
            mask = in > 32767;
            in(mask) = in(mask) - 65535 - 1;
            out = int16(in);
        end
        function wheel = rollover(delta)
            roll = 65536; % 2^16
            threshold = roll/2;
            if (delta > threshold)
                delta = delta - roll;
            elseif (delta < -threshold)
                delta = delta + roll;
            end
            wheel = delta;
        end
    end
end
