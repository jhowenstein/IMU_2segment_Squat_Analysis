function [ end_row, end_col ] = bar_pts( start_row, start_col, offset, lift )

if strcmp(lift,'BS')
    end_row = start_row;
    end_col = round(start_col + (offset * 200));
elseif strcmp(lift,'FS')
    end_row = start_row;
    end_col = round(start_col + (offset * 200));
end

