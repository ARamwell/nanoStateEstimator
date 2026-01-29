function pose = rtToPose(Rt)
            
            pos = Rt(1:3,4);

            orient = rotm2quat(Rt(1:3, 1:3));

            pose = [pos; orient'];
            
end