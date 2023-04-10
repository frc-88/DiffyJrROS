classdef DiffSwerveModule
    %DIFFSWERVE Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        M_inv  % inverse kinematics for diff swerve gear box
        J_w  % moment of inertia of the wheel
        Kt  % Kt motor property
        Kv  % Kv motor property
        R  % motor resistance
        sysc  % continuous state space system object
        sysd  % discrete state space system object
        lqr_K  % LQR gain
        Kff  % feedforward gain
        D_w  % wheel diameter
        Ts  % discrete time sample size
    end
    
    methods
        function obj = DiffSwerveModule()
            %DIFFSWERVE Construct an instance of this class
            %   Detailed explanation goes here
            
            M_inv = [
                14, 6;
                -10, 6
            ];
            J_w = 0.015;

            D_w = 0.0445;
            
            Kt = 0.01825;
            Kv = 56.0;
            R = 0.0467 / 2;
            
            A = -Kt / (Kv * R * J_w) * M_inv * M_inv;
            A = [1, 0; A];
            A_column_0 = zeros([3, 1]);
            A = [A_column_0, A];
            
            B = [0, 0;
                M_inv * Kt / (R * J_w)];
            
            C = eye(3);
            D = zeros([3, 2]);
            
            Ts = 0.005;
            
            sysc = ss(A, B, C, D);
            sysd = c2d(sysc, Ts);

            Q = [1.0, 0, 0;
                 0, 10.0, 0;
                 0, 0, 10.0];
            
            R = [12, 0;
                 0, 12];

            Q = eye(3) / Q^2;
            R = eye(2) / R^2;

            lqr_K = lqr(sysd, Q, R);
            Kff = pinv(sysd.B);

            obj.M_inv = M_inv;
            obj.J_w = J_w;
            obj.D_w = D_w;
            obj.Kt = Kt;
            obj.Kv = Kv;
            obj.R = R;
            obj.sysc = sysc;
            obj.sysd = sysd;
            obj.lqr_K = lqr_K;
            obj.Kff = Kff;
            obj.Ts = Ts;
        end
    end
end

