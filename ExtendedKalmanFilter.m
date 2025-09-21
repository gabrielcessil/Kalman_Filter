classdef ExtendedKalmanFilter
    properties
        f        % função de estado
        h        % função de medição
        Jf       % Jacobiano de f
        Jh       % Jacobiano de h
        Qv       % covariância do processo
        Rv       % covariância da medição
        P        % covariância do estado
        xhat     % estado estimado
    end
    
    methods
        function obj = ExtendedKalmanFilter(f, h, Jf, Jh, Qv, Rv, P0, x0)
            obj.f = f;
            obj.h = h;
            obj.Jf = Jf;
            obj.Jh = Jh;
            obj.Qv = Qv;
            obj.Rv = Rv;
            obj.P  = P0;
            obj.xhat = x0;
        end
        
        function obj = estimate(obj, y, u)
            % Predição
            obj.xhat = obj.f(obj.xhat, u);
            Jf_mat = obj.Jf(obj.xhat, u);
            obj.P = Jf_mat * obj.P * Jf_mat.' + obj.Qv;
            
            % Atualização
            K = obj.P * obj.Jh.' / (obj.Jh * obj.P * obj.Jh.' + obj.Rv);
            e = y - obj.h(obj.xhat);
            obj.xhat = obj.xhat + K*e;
            obj.P = obj.P - K*obj.Jh*obj.P;
        end
    end
end