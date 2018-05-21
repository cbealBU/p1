%This function creates a steering input specified by the string input_type
%The steering input types are as follows:
%
%'step': steering('step', init_angle, final_angle, steptime, 
%simulationtime)) creates a step from initial steer angle init_angle (rad)  
%to final steer angle final_angle (rad) when simulationtime == steptime 
%(in s)
%
%'from data': steering('from data', steering_data, time_data,
%simulationtime) uses steering_data and time_data to compute the steering
%angle corresponding to simulation time (by linear interpolation, if
%necessary)
%
%'ramp': steering('ramp', init_angle, rate, ramptime, simulationtime)
%creates a ramp steer beginning at init_angle and changing according to
%rate (rad/s) when ramptime >= simulation time

function steering_input = steering(input_type, varargin)

%If desired steering input is a step steer
if strcmp(input_type, 'step')
    %If the proper number of input arguments have been specified
    if nargin == 5
        init_angle = varargin{1}; %Initial steer angle (rad)
        final_angle = varargin{2}; %Final steer angle (rad)
        steptime = varargin{3}; %Time of steep steer (s)
        simulationtime = varargin{4}; %Simulation time (s)
        if (simulationtime < steptime)
            steering_input = init_angle;
        elseif (simulationtime >= steptime)
            steering_input = final_angle;
        end
    else
        error('Incorrect number of inputs for step steer input');
    end
elseif strcmp(input_type, 'from data')
    %If the proper number of input arguments have been specified
    if nargin == 4
        steering_data = varargin{1}; %Vector of Steering angle data (rad)
        time_data = varargin{2};     %Vector of Time data (s)
        simulationtime = varargin{3}; %Simulation time (s)
        
        %Use interp1 to interpolate between data points to obtain steering
        %estimate at desired time
        steering_input = interp1(time_data, steering_data, simulationtime);
            
    else
        error('Incorrect number of inputs for steering from data input');
    end
elseif strcmp(input_type, 'ramp')
    %If the proper number of input arguments have been specified
    if nargin == 5
        init_angle = varargin{1}; %Initial steer angle (rad)
        rate = varargin{2}; %Ramp rate (rad/s)
        ramptime = varargin{3}; %Time to begin ramp (s)
        simulationtime = varargin{4}; %Simulation time (s)
        
        if (simulationtime < ramptime)
            steering_input = init_angle;
        elseif (simulationtime >= ramptime)
            steering_input = init_angle + rate*simulationtime;
        end
    else
        error('Incorrect number of inputs for ramp steer input');
    end        
else
    error('Incorrect steering input type specified');
end


            
            
            

