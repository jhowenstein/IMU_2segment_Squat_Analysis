function [ L_trunk, L_thigh, L_shank, mass_trunk, mass_thigh, mass_shank ] = subject_parameters( subject_height, subject_mass, gender )

    if gender == 'male'
        L_trunk = subject_height * .30; 
        L_thigh = subject_height * .232;
        L_shank = subject_height * .247;
        mass_trunk = subject_mass * .5534;
        mass_thigh = subject_mass * .1416;
        mass_shank = subject_mass * .0433;
        mass_foot = subject_mass * .0137;
    elseif gender == 'female'
        L_trunk = subject_height * .29; 
        L_thigh = subject_height * .249;
        L_shank = subject_height * .257;
        mass_trunk = subject_mass * .5375;
        mass_thigh = subject_mass * .1478;
        mass_shank = subject_mass * .0481;
        mass_foot = subject_mass * .0129;
    end
end

