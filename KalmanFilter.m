classdef KalmanFilter
    properties
        A, B, Gamma, C, D, Qv, Rv
        P, xhat, K, e
    end
    methods
        function obj = KalmanFilter(A,B, Gamma, C, D, Qv, Rv, P0, x0)
            obj.A = A;
            obj.B = B;
            obj.Gamma = Gamma;
            obj.C = C;
            obj.D = D;
            obj.Qv = Qv;
            obj.Rv = Rv;
            obj.P = P0;
            obj.xhat = x0;
        end
        function obj = estimate(obj, y, u)
            obj.xhat    = obj.A*obj.xhat + obj.B*u;
            obj.P       = obj.A*obj.P*obj.A.' + obj.Gamma*obj.Qv*obj.Gamma.';
            obj.e       = y-obj.C*obj.xhat;

            obj.K       = obj.P*obj.C.' / (obj.C*obj.P*obj.C.' + obj.Rv);
            obj.xhat    = obj.xhat + obj.K*(obj.e); 
            obj.P       = obj.P - obj.K*obj.C*obj.P; 
        end
    end
end