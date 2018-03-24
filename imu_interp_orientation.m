function [ Ex2, Ey2, Ez2 ] = imu_interp_orientation( t , Ex, Ey, Ez, trial_t )

Ex2 = interp1(t, Ex, trial_t, 'spline');
Ey2 = interp1(t, Ey, trial_t, 'spline');
Ez2 = interp1(t, Ez, trial_t, 'spline');

end

