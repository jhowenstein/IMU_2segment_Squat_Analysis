function [ time ] = imu_extract_time( csv_doc )

time(1,:) = csv_doc(:,1);

end

