function [state, lastCalcTimes] = fastIMUProp(w_ib_b, sf_b, prevState, g_i, clock, prevLastCalcTimes, imuPropInterval)

lastCalcTimes = prevLastCalcTimes;
state = prevState;


if clock - prevLastCalcTimes(1) >= imuPropInterval
    dt = clock - max(prevLastCalcTimes(1:4));

    
    
    

    rot_vec = 1.0 * w_ib_b * dt;
    q = prevState(1:4)';
    
    rot_vec_norm = norm(rot_vec);
    axis = rot_vec / rot_vec_norm;
   
    if min(rot_vec_norm) < 1e-9 % Small angle approx if rally small angle
        dq = [1; 0.5*rot_vec]';
    else
        dq = [cos(rot_vec_norm / 2.0); (axis * sin(rot_vec_norm / 2.0))]';
    end
    
    q = quatmultiply(q, dq);
    q = q / norm(q);
    
    state(1:4) = q';
    
    v_dot = quat2rotm(q) * sf_b + g_i;
    
    v = prevState(5:7) + v_dot * dt;
    
    r = prevState(8:10) + v * dt;
    
    state(5:7) = v;
    state(8:10) = r;
    lastCalcTimes(1) = clock;
    %{
    %}
end



end


