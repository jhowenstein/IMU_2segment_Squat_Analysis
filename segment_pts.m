function [ end_row, end_col ] = segment_pts( L_segment, start_row, start_col, seg_angle )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
end_col = round(start_col + (L_segment * cosd(seg_angle) * 200));
end_row = round(start_row - (L_segment * sind(seg_angle) * 200));


end

