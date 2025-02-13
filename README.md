# ENGR 290 AUTONOMOUS HOVERCRAFT PROJECT

This is the ATMega328P-based autonomous hovercraft designed and built in light of the ENGR 290 design project competition. We placed 4th out of 21 teams.

## Project Description
- Autonomous hovercraft capable of detecting and avoiding obstacles, as well as detecting and finishing under a finish line
- Capable of generating ~4mm of lift

## Features

### Skirt
- Ergonomic 2-story skirt cut out of a trash bag provides enough airflow to support the body of the hovercraft
- Enough lift generated to fly over 2 stacked 2mm thich rulers

### Ultra-sonic sensors
- Use of 2 ultra-sonic sensors to determine distances:
- one sensor is mounted on a steering servo and detects walls
- other sensor is facing upwards to detect finish line
- governing functions defined in "us.c" file

### Inertial Measurement Unit
- detects drifting in the hovercraft's yaw
- creates a feedback loop with the steering servo to redirect the hovercraft allowing it to follow straight-ish paths
- allows hovercraft to precisely turn by a desired angle by measuring the yaw

  
  
Here's a short clip of the hovercraft in action!

https://github.com/user-attachments/assets/e6d9f7b8-9478-4b37-bdd1-0f0a62cabccc

