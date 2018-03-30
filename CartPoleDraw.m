classdef CartPoleDraw < handle
% CartPole 
%
% Created by S. Baek on 2018-01-18
% Bioinspiration and Intelligent Robotics Lab
% University of Michigan - Dearborn
%
% Last updated: 2018-03-17 by S. Baek
%   - Made a few changes for ECE560
%
% Copyright (c) University of Michigan.
% 
% DO NOT DISTRIBUTE THIS CODE WITHOUT PERMISSION OF THE UNIVERSITY OF MICHIGAN.
%
% THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND, 
% EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED 
% WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.

    properties(GetAccess = 'public', SetAccess = 'private')
    % public read access, but private write access.
    
        % cart pole parameters
        PoleLength = 0.5;
        CartLength = 0.2;
        CartHeight = 0.1;
        
    end

    properties(GetAccess = 'private', SetAccess = 'private')
    % private read and write access
    
        Paused = false;      % Is simulation pause? need to change the name to IsPause
        Terminated = false;
        
        PlotRecorder;
        
        SimulationRefreshSpeed;       
        % pause simulation
        PauseButton;    
        
        % To store video frames
        FrameQueue;
        
        FigureHandle;
        LineHandle;
        AnimationAxes;
        AxesRange = [-1, 1, -0.6, 0.6];
        
    end
            
    methods   
        
        function self = CartPoleDraw(name, simulationRefreshSpeed)% , gcf)
            figure('name', name);
            self.SimulationRefreshSpeed = simulationRefreshSpeed;
            self.FigureHandle = gcf;
            self.AnimationAxes = self.DrawMissionMap(self.FigureHandle);
        end

        function plotaxes = GetAnimationAxes(self)
            plotaxes = self.AnimationAxes; 
        end
        
        function SetAxesRange(self, axes_range)
            self.AxesRange = axes_range; 
        end
        
        function EnablePlotRecoder(self)
            
            saveLocation = strcat('./save_', datestr(now,'yyyy-mm-dd-HH-MM-SS'));
            mkdir(saveLocation);
            
            videoFileName = strcat(saveLocation, '/VideoOutput.avi');
            self.PlotRecorder = VideoWriter(videoFileName); % Name it.
            self.PlotRecorder.FrameRate = self.SimulationRefreshSpeed; % How many frames per second.
            open(self.PlotRecorder); 
            self.FrameQueue = Queue();
            
        end
            
        function DisablePlotRecoder(self)
            self.PlotRecorder = []; 
            self.FrameQueue = Queue(); % empty out.
        end
        
        function RecordFrame(self)            
            assert(~isempty(self.PlotRecorder), 'Plot recoder has not been enabled.');
            frame = getframe(self.FigureHandle); 
            self.FrameQueue.Push(frame);
        end
        
        function Close(self)
            self.Terminate()
        end
           
        function bool = IsPaused(self)
            bool = self.Paused;            
        end

        function bool = IsRunning(self)
            bool = ~self.Terminated;            
        end                
        
        function Draw(self, cartposx, angle, time)
            
            
            simulationTimeStep = 1/self.SimulationRefreshSpeed;
            index = 1;
            
            while true,
                
                tic;
                if (index > length(time))
                    return;
                end
                
                if (~self.IsRunning())
                    self.Close();                    
                    break;
                end
                
                if (self.IsPaused())
                    pause(0.5);
                    continue;
                end
                             
                self.DrawOne(cartposx(index), angle(index), time(index));
                
                elapsedTime = toc;
                if (elapsedTime > simulationTimeStep)
                    fprintf('Warning: computational time %f is greater than simulation period of %f\n', ...
                            elapsedTime, simulationTimeStep);
                end
                pause(simulationTimeStep - elapsedTime);
                
                index = index + 1;
            end
         
        end % end of function
                
    end % end of public methods
        
    
    %% Private methods
    methods (Access = private)   
        
        function DrawOne(self, cartposx, angle, time)
   
            delete(self.LineHandle); % remove lines from the privous frame
            
            %set(gcf, 'currentaxes', self.AnimationAxes);
            
            cartpos = [cartposx; 0];
            % Draw cart
            bottomLeft = cartpos + [-self.CartLength/2; -self.CartHeight/2];
            self.LineHandle(1) = rectangle('Position', [bottomLeft' self.CartLength, self.CartHeight], 'Curvature', 0.4, 'LineWidth', 2); 

            % Draw pole
            theta = pi/2 - angle;
            pole_end = cartpos + self.PoleLength*[cos(theta); sin(theta)];
            points = [cartpos,  pole_end];
            rgbCode = [1, 0, 0];
            self.LineHandle(2) = line(points(1,:), points(2,:), 'Color', rgbCode, 'LineWidth', 3);
            
            % Draw ball at the end of the pole
            radius = 0.02;
            % Curvature of [1 1] draws a circle.
            self.LineHandle(3) = rectangle('Position', [pole_end(1)-radius, pole_end(2)-radius, radius*2, radius*2], 'Curvature', [1, 1], 'LineWidth', 2); 
          
            % |--------|--------|--------|--------|
            % a        25       50       75       b
            % 75 = ((a+b)/2 + b)/2 = (a+3b)/4
            % the x-axis range = [self.AxesRange(1), self.AxesRange(2)]
            axis75 = (self.AxesRange(1) + 3*self.AxesRange(2))/4; % 3/4 (75%) location on the axis
            axis25 = (3*self.AxesRange(1) + self.AxesRange(2))/4; % 3/4 (75%) location on the axis
            
            if (cartpos(1) > axis75)
                dx = cartpos(1) - axis75;
                self.AxesRange(1:2) = self.AxesRange(1:2) + dx; 
            elseif (cartpos(1) < axis25)
                dx = cartpos(1) - axis25;
                self.AxesRange(1:2) = self.AxesRange(1:2) + dx; 
            end
            
            % Put the time 
            textposx =  self.AxesRange(1) + .6*(self.AxesRange(2)-self.AxesRange(1));
            textposy = 0.47;
            
            theta = mod(angle*180/pi, 360);
            self.LineHandle(4) = text(textposx, textposy, sprintf('time: %2.3f s, cart: %2.2f m, angle: %4.1f deg', time, cartposx, theta));
           
            axis(self.AxesRange);
            
            if ~isempty(self.PlotRecorder)
                self.RecordFrame();
            end
        end
        
        
        function PauseSimulation(self, varargin)        
            disp 'Simulation Paused. Press Resume to resume'
            
            if (self.Paused)
                set(self.PauseButton, 'String', 'Pause');
                self.Paused = false;
            else
                set(self.PauseButton, 'String', 'Resume');
                self.Paused = true;
            end
        end
                
        function StopSimulation(self, varargin)        
            self.Terminated = true;
        end
        
        function Terminate(self)
            
            disp 'Terminating Cart Pole Simulation ....'
            
            if isempty(self.PlotRecorder)
                close(self.FigureHandle);
                return;
            end
            
            % pause for 0.1 second to keep it from VideoWriter crashing. 
            % This is a workaround for the error, 'OBJ must be open before 
            % writing video.  Call open(obj) before calling writeVideo.'
            pause(0.1); 
            
            % TODO: waitbar is not working.
            % waitbarHandle = waitbar(0, '1', 'Name', 'Creating Video...', ...
            %                         'CreateCancelBtn',...
            %                         'setappdata(gcbf,''canceling'',1)');
            % setappdata(waitbarHandle,'canceling',0);

            % totalFrameSize = self.FrameQueue.Size;
            % currentFrameIndex = 1;
            % set(0, 'CurrentFigure', waitbarHandle); 
            while (self.FrameQueue.Size > 0)
                frame = self.FrameQueue.GetFirst();
                writeVideo(self.PlotRecorder, frame);
            %    if getappdata(waitbarHandle,'canceling')
            %        break
            %    end
            %    waitbar(currentFrameIndex / totalFrameSize); %, ...
                        %sprintf('%d frame out of %d frames have been exported', currentFrameIndex, totalFrameSize));
            %    currentFrameIndex = currentFrameIndex + 1;    
            end
            
            %pause(1);
            
            % close(waitbarHandle);
            % close(self.FigureHandle);
            close(self.PlotRecorder);
        end
                
        
        function missionMapAxes = DrawMissionMap(self, figureHandler)
        % DrawMissionPlan sets up a figure with plots for mission map. 
       
            % figure size
            xmin = 100;
            ymin = 100;
            width = 800;
            height = 500;
            
            figurePosition = [xmin, ymin, width, height];
            set(figureHandler, 'Position', figurePosition); 
                
            %% Set the plot area
            xmin = 0.05;
            ymin = 0.12;
            height = 0.8;
            width = 0.9;                

            plotPosition = [xmin, ymin, width, height];            
            missionMapAxes = subplot('Position', plotPosition);  % plot to draw uav mission
            set(gcf, 'currentaxes', missionMapAxes);            
            axis(self.AxesRange);
            grid on;
            
            %% Draw user interface
            uicStopSimulation = uicontrol('Style', 'pushbutton', 'String', 'Exit', ...
                                        'Position', [20 15 60 20], ...
                                        'Callback', @self.StopSimulation);
                                    
            self.PauseButton = uicontrol('Style', 'pushbutton', 'String', 'Pause', ...
                                         'Position', [100 15 60 20], ...
                                         'Callback', @self.PauseSimulation);                        

        end
                
    end
            
    methods (Static, Access = public)
        
        function RunSimpleTest()           
        
            %% Plot Preparation            
            simulationRefreshSpeed = 10;
            animation = CartPoleDraw('Cart Pole', simulationRefreshSpeed);%, gcf);
            animation.EnablePlotRecoder();            
            
            %% Run Simulation
            t = linspace(0, 4, 400);
            cartpos = sin(pi*t);
            poleangle = 4*pi*t;
                             
            animation.Draw(cartpos, poleangle, t);
           
            % close...
            animation.Close();
        end
        
    end
end












        