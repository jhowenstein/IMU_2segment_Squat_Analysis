# IMU_2segment_Squat_Analysis
This project analyzes the squatting mechanics of a subject wearing two IMU sensors.
One sensor is located on the user's trunk (similar location to a heart rate monitor) and the other is placed on the side of one thigh.

This analysis script visualized the orientations and motions of the two primary segments in the squatting motion.
Weightlifting and squatting are primarily planar motions and thus the analysis was calculated in two-dimensions from the sagittal view.

Because weightlifting is a closed system with a known force (bar weight + acceleration) at the end of the chain, inverse dynamics can be estimated.

Given the kinetic calculations, segment orientiations, subject parameters (height/weight), and the mass of the bar, a joint power analysis and segment power analysis was able to be calculated.

The joint power analysis calculated the relative contributions of the hip and knee joint of the motion.

The segment power analysis was able to expand upon this and estimate the contributions of individual muscle groups. 
-Anterior thigh (quadriceps)
-Posterior thight (glutes and hamstrings)
-Posterior trunk (Back musculature)

This can be used to analyze athletes on an individual level to quantify technique, identify weaknesses, and created targeted training strategies with objective feedback.
