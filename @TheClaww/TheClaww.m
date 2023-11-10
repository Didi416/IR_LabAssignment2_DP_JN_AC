classdef TheClaww < RobotBaseClass

    properties(Access = public)   
        plyFileNameStem = 'TheClaww';
    end
    
    methods
%% Constructor
         function self = TheClaww()
%             %baseTr = transl(0,0,0); 
%           
              self.CreateModel(); 
              self.model.base = self.model.base.T; %* baseTr;
              self.model.tool = self.toolTr;
              self.PlotAndColourRobot();
 
              drawnow
         end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.01519,'a',0,'alpha',pi/2,'qlim',deg2rad([-0 0]), 'offset',0);
            link(2) = Link('d',0,'a',-0.073,'alpha',0,'qlim', deg2rad([0 45]), 'offset',0);
            link(3) = Link('d',0,'a',-0.103,'alpha',0,'qlim', deg2rad([0 45]), 'offset',0);
            %link(1) = Link('d',0.06,'a',-0.055,'alpha',180,'qlim',deg2rad([-180 180]), 'offset',0);
            %link(2) = Link('d',-0.043,'a',-0.035,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',pi);
      
            self.model = SerialLink(link,'name',self.name);
        end 
    end
end
        


