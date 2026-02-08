classdef PlatformPredictionEdge < g2o.core.BaseBinaryEdge
    % PlatformPredictionEdge summary of PlatformPredictionEdge
    %
    % This class stores the factor representing the process model which
    % transforms the state from timestep k to k+1
    %
    % The process model is as follows.
    %
    % Define the rotation vector
    %
    %   M = dT * [cos(theta) -sin(theta) 0; sin(theta) cos(theta) 0;0 0 1];
    %
    % The new state is predicted from 
    %
    %   x_(k+1) = x_(k) + M * [vx;vy;theta]
    %
    % Note in this case the measurement is actually the mean of the process
    % noise. It has a value of 0. The error vector is given by
    %
    % e(x,z) = inv(M) * (x_(k+1) - x_(k))
    %
    % Note this requires estimates from two vertices - x_(k) and x_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k) and slot 2 contains x_(k+1).
    
    properties(Access = protected)
        % The length of the time step
        dT;
    end
    
    methods(Access = public)
        function obj = PlatformPredictionEdge(dT)
            % PlatformPredictionEdge for PlatformPredictionEdge
            %
            % Syntax:
            %   obj = PlatformPredictionEdge(dT);
            %
            % Description:
            %   Creates an instance of the PlatformPredictionEdge object.
            %   This predicts the state from one timestep to the next. The
            %   length of the prediction interval is dT.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a PlatformPredictionEdge

            assert(dT >= 0);
            obj = obj@g2o.core.BaseBinaryEdge(3);            
            obj.dT = dT;
        end
       
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of a platform.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the platform x_(k+1) given
            %   an estimate of the platform at time x_(k) and the control
            %   input u_(k+1)
            
            xk = obj.edgeVertices{1}.estimate();
            u = obj.z;

            c = cos(xk(3));
            s = sin(xk(3));
            M = [c -s 0; s c 0; 0 0 1];

            xkp = xk + obj.dT * M * u;
            xkp(3) = g2o.stuff.normalize_theta(xkp(3));

            obj.edgeVertices{2}.setEstimate(xkp);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the measurement and the parameter state in the
            %   vertex. Note the error enters in a nonlinear manner, so the
            %   equation has to be rearranged to make the error the subject
            %   of the formulat
            
            xk = obj.edgeVertices{1}.estimate();
            xkp = obj.edgeVertices{2}.estimate();

            c = cos(xk(3));
            s = sin(xk(3));
            Mt = [c s 0; -s c 0; 0 0 1];

            dx = xkp - xk;
            dx(3) = g2o.stuff.normalize_theta(dx(3));
            uhat = (1 / obj.dT) * Mt * dx;
            obj.errorZ = obj.z - uhat;
        end
        
        % Compute the Jacobians
        function linearizeOplus(obj)
            % LINEARIZEOPLUS Compute the Jacobians for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the Jacobians for the edge. Since we have two
            %   vertices which contribute to the edge, the Jacobians with
            %   respect to both of them must be computed.
            %
            xk = obj.edgeVertices{1}.estimate();
            xkp = obj.edgeVertices{2}.estimate();

            c = cos(xk(3));
            s = sin(xk(3));
            Mt = [c s 0; -s c 0; 0 0 1];

            dMtdtheta = [-s c 0; -c -s 0; 0 0 0];
            dx = xkp - xk;
            dx(3) = g2o.stuff.normalize_theta(dx(3));

            J2 = -(1 / obj.dT) * Mt;
            J1 = (1 / obj.dT) * Mt;
            J1(:, 3) = J1(:, 3) - (1 / obj.dT) * (dMtdtheta * dx);

            obj.J{1} = J1;
            obj.J{2} = J2;
        end
    end    
end
