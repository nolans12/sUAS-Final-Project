function [commandState] = calcCommandFromVec(waypoint_follow, vec_follow)

    % Input is commanded waypoint [pn, pe, pd] and commanded velocity vector [ue_e, ve_e, we_e]

    % Now, calculate the control objectives
    h_c = -waypoint_follow(3);
    h_dot_c = -vec_follow(3);
    chi_c = atan2(vec_follow(2), vec_follow(1));
    chi_dot_c = 0;
    Va_c = norm(vec_follow);
    
    commandState = [h_c, h_dot_c, chi_c, chi_dot_c, Va_c];
end

