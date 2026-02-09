classdef LandmarkRangeBearingEdge < g2o.core.BaseBinaryEdge
    % LandmarkRangeBearingEdge summary of LandmarkRangeBearingEdge
    %
    % This class stores an edge which represents the factor for observing
    % the range and bearing of a landmark from the vehicle. Note that the
    % sensor is fixed to the platform.
    %
    % The measurement model is
    %
    %    z_(k+1)=h[x_(k+1)]+w_(k+1)
    %
    % The measurements are r_(k+1) and beta_(k+1) and are given as follows.
    % The sensor is at (lx, ly).
    %
    %    dx = lx - x_(k+1); dy = ly - y_(k+1)
    %
    %    r(k+1) = sqrt(dx^2+dy^2)
    %    beta(k+1) = atan2(dy, dx) - theta_(k+1)
    %
    % The error term
    %    e(x,z) = z(k+1) - h[x(k+1)]
    %
    % However, remember that angle wrapping is required, so you will need
    % to handle this appropriately in compute error.
    %
    % Note this requires estimates from two vertices - x_(k+1) and l_(k+1).
    % Therefore, this inherits from a binary edge. We use the convention
    % that vertex slot 1 contains x_(k+1) and slot 2 contains l_(k+1).
    
    methods(Access = public)
    
        function obj = LandmarkRangeBearingEdge()
            % LandmarkRangeBearingEdge for LandmarkRangeBearingEdge
            %
            % Syntax:
            %   obj = LandmarkRangeBearingEdge();
            %
            % Description:
            %   Creates an instance of the LandmarkRangeBearingEdge object.
            %   Note we feed in to the constructor the landmark position.
            %   This is to show there is another way to implement this
            %   functionality from the range bearing edge from activity 3.
            %
            % Outputs:
            %   obj - (handle)
            %       An instance of a LandmarkRangeBearingEdge.
            
            obj = obj@g2o.core.BaseBinaryEdge(2);

        end
        
        function initialEstimate(obj)
            % INITIALESTIMATE Compute the initial estimate of the landmark.
            %
            % Syntax:
            %   obj.initialEstimate();
            %
            % Description:
            %   Compute the initial estimate of the landmark given the
            %   platform pose and observation.

            pose = obj.edgeVertices{1}.estimate();
            x = pose(1);
            y = pose(2);
            theta = pose(3);

            r = obj.z(1);
            beta = obj.z(2);

            alpha = theta+beta;
            l = [x+r*cos(alpha); y+r*sin(alpha)];

            obj.edgeVertices{2}.setEstimate(l);
        end
        
        function computeError(obj)
            % COMPUTEERROR Compute the error for the edge.
            %
            % Syntax:
            %   obj.computeError();
            %
            % Description:
            %   Compute the value of the error, which is the difference
            %   between the predicted and actual range-bearing measurement.
           
            obj.errorZ = zeros(2, 1);
            pose = obj.edgeVertices{1}.estimate();
            x = pose(1);
            y = pose(2);
            theta = pose(3);
            
            l = obj.edgeVertices{2}.estimate();
            lx = l(1);
            ly = l(2);

            dy = ly-y;
            dx = lx-x;
            r = sqrt(dy.^2+dx.^2);
            beta = atan2(dy,dx)-theta;
            
            obj.errorZ(1) = obj.z(1)-r;
            obj.errorZ(2) = g2o.stuff.normalize_theta(obj.z(2)-beta);
        end
        
        function linearizeOplus(obj)
            % linearizeOplus Compute the Jacobian of the error in the edge.
            %
            % Syntax:
            %   obj.linearizeOplus();
            %
            % Description:
            %   Compute the Jacobian of the error function with respect to
            %   the vertex.
            %

            obj.J{1} = eye(2, 3);
            
            obj.J{2} = eye(2);

            pose = obj.edgeVertices{1}.estimate();
            x = pose(1);
            y = pose(2);
            theta = pose(3);

            l = obj.edgeVertices{2}.estimate();
            lx = l(1);
            ly = l(2);
            
            dx = lx-x;
            dy = ly-y;

            q = dx*dx + dy*dy;
            r = sqrt(q);

            %lets be safe incase pose==landmark
            eps=1e-12;
            if q<eps
                q=eps;
                r=sqrt(q);
            end

            obj.J{1} = [dx/r, dy/r, 0; -dy/q, dx/q, 1];

            obj.J{2} = [-dx/r, -dy/r; dy/q, -dx/q];
        end        
    end
end
