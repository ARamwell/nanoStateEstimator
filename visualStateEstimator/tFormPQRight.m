function pq_c2b = tFormPQRight(pq_a2b, T_c2a)
            
            pq_c2b = nan(7, size(pq_a2b, 2));

            for c = 1: size(pq_a2b, 2)
                p_a2b = pq_a2b(1:3, c);
                q_a2b = pq_a2b(4:7, c);

                T_a2b = quat2tform(q_a2b');
                T_a2b(1:3, 4) = p_a2b;

                T_c2b = T_a2b * T_c2a;

                q_c2b = tform2quat(T_c2b)';
                p_c2b = T_c2b(1:3, 4);

                pq_c2b(1:7,c) = [p_c2b(1:3); q_c2b(1:4)];
            end
               
end
