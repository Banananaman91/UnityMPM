# UnityMPM
A simple implementation of MPM in Unity. This is intended to analyse the performance of MPM in real time,
breaking down components of the method to view performance impacts for each study, as well as see the visual results
of a solid material impact.

## Information for analyising performance data
All scenes in the DataStudy folder are setup for analysing data.
Case studies 1 - 3 analyse the performance impacts on increasing particle counts.
Case studies 4 - 6 analyse the performance impacts of the deltaX (DX) for calculations.
Case studies 7 - 9 analyse the performance impacts of the DT for calculations

All scenes inside of the VisualStudy folder are setup without performance in mind.
These scenes have variations of the Mass and Size of the cube object which collides with the MPM particles.
They are intended to simulate a solid cube in MPM and see the visual impacts in action.

## Collecting and Analysing DataStudy scenes
Install Profile Anlyzer in the package manager. This is a preview package.
Open up the Unity Profiler in Window > Analysis > Profiler as well as the Profile Analyzer in the same place.
With profiler recording, run your scene. Press space to have the cube begin moving and collide with the MPM particles.
Once complete, on the Profile Analyzer window click Pull Data. This will import all of the data from the profiler.
If desired, click Save Data. You can also import two sets of data for comparison. In this case, it is beneficial to compare the data
of two studies to view the performance impacts of each method.