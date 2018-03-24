function [ Ex, Ey, Ez  ] = imu_extract_euler( csv_doc )

Ex(1,:) = csv_doc(:,8);
Ey(1,:) = csv_doc(:,9);
Ez(1,:) = csv_doc(:,10);

end

