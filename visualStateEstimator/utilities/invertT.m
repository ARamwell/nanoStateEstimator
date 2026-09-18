        
        function T_inv = invertT(T)

            R = T(1:3, 1:3);
            t = T(1:3, 4);

            R_inv = transpose(R);
            t_inv = -1 * R_inv * t;

            Rt_inv = [R_inv t_inv];
            T_inv = [Rt_inv; 0 0 0 1];
        end