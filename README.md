# UnityMPM Material Point Method for Solid Physics Simulation
A simple implementation of MPM in Unity. This is intended to analyse the performance of MPM in real time,
breaking down components of the method to view performance impacts for each study, as well as see the visual results
of a solid material impact.

# Features
- ![Grid-Based collision detection](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L560)
- ![DeltaX Stress term](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L360)
- ![Particle to Grid calculation](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L318)
- ![Grid-velocity boundary checks](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L433)
- ![Grid to Particle calculation](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L462)
- ![Particle velocity correction at boundary](https://github.com/Banananaman91/UnityMPM/blob/4ddfd7f5767691db6bc5d9a7bc46c2d83b3c1dbc/Assets/MpmBodyCollision.cs#L540)

![](https://github.com/Banananaman91/UnityMPM/blob/main/128cubeMass16.gif)

## Information for analyising performance data
All scenes in the DataStudy folder are setup for analysing the performance impacts of MPM, with focuses on Particle Counts, DX and DT applications.

All scenes inside of the VisualStudy folder are setup without performance in mind.
These scenes have variations of the Mass and Size of the cube object which collides with the MPM particles.
They are intended to simulate a solid cube in MPM and see the visual impacts in action.

## Dependencies
This project requires Unity Jobs package installed from the package manager, along with Burst Compiler and Mathematics

## Collecting and Analysing DataStudy scenes
Install Profile Anlyzer in the package manager. This is a preview package.
Open up the Unity Profiler in Window > Analysis > Profiler as well as the Profile Analyzer in the same place.
With profiler recording, run your scene. Press space to have the cube begin moving and collide with the MPM particles.
Once complete, on the Profile Analyzer window click Pull Data. This will import all of the data from the profiler.
If desired, click Save Data. You can also import two sets of data for comparison. In this case, it is beneficial to compare the data
of two studies to view the performance impacts of each method.

## Acknowledgements
This project uses [Nialltl](https://github.com/nialltl/incremental_mpm) original implementation, with a replaced render simulation, adapted code to expose more variables,
as well as exposing DX control and using a stiky grid-based collision.
