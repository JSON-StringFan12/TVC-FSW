# TVC\_teensey\_hutto

LRA flight computer 19-20 Gen 2, adapted for TVC rocket at 2026 Dec 6th Hutto launch



The rocket is named Alberto btw. 



## Data



There are 3 csv files in the data folder. Since datalogging activates upon triggering an accelerometer threshold, garbage files might be created due to handling before the rocket launches off rail. However, we know it resets after 40 seconds, and the rockets sits still on the pad for 5-10 minutes. Therefore the true data should have a long idle time before the true launch.



The most likely data that matches this feature is towards the end of the second csv file (the largest one). It seems the IMU failed to give correct orientation at the moment of initial acceleration, and the TVC never recovered from it since. I suggest a detailed look at the sensor fusion algorithm that might've used accelerometer for filtering and caused the problem.



However, this is all based on assuming the data we are looking for is towards the end of the second csv file. Feel free to poke around more to see what you think is more likely our true data. Also take a close look at the video to see if everything matches.



A video link is needed here:

(if not, it's also in TVC teams, trying scrolling to Dec 6th 2025)



