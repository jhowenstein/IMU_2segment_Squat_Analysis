function [ dist_end_row, dist_end_col, prox_end_row, prox_end_col ] = segment_pts_center_rot( L_segment, origin_row, origin_col, seg_angle )

dist_end_row = round(origin_row - (.5 * L_segment * sind(seg_angle) * 200));
prox_end_row = round(origin_row + (.5 * L_segment * sind(seg_angle) * 200));

dist_end_col = round(origin_col + (.5 * L_segment * cosd(seg_angle) * 200));
prox_end_col = round(origin_col - (.5 * L_segment * cosd(seg_angle) * 200));

end

