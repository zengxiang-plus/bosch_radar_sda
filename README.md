#  bosch radar

## Intro

This repo contains bosch cr5tp side radar driver. The repo is managed as catkin packages.

## Protocol

[CR5TP Bosch Radar Protocol](https://docs.google.com/spreadsheets/d/1dtoR8A5vH02-1UfuA_udxV4BcNNtSkHo/edit#gid=443239856)

[CANFD UDP Protocol](https://drive.google.com/drive/folders/1CYwRwl2yf1yShXIDAN1coOaq-FXLp2MP)


## Acceptance Test
[Bosch Radar Test Cases](https://docs.google.com/spreadsheets/d/1eoi9dxy_SBbq6y0my4rCs1EQMo3PKHm86JUokSVs214/edit#gid=75548805)

[Bosch Radar Test Result](https://docs.google.com/document/d/1VyBEweBlPY1tcbSkRhZvu_zroAR7Bp0gGdBnoMMxx4U/edit)

## Device Dependency
Wheel speed info from vehicle CAN.

Yaw rate info from IMU


## Radar Property

| Key               | Value           | 
|:---               |:---                            
| Working frequency                | 76~77GHz                    | 
| Target update period             | ≤ 70ms                      |
| Horizontal view angle:           | ≥ ±75°                      |
| Install location                 | 25 degrees forward          | 
| Target output                    | Only output dynamic targets |
| Target speed measurement range   | -288km/h~288km/h            |
| Speed measurement accuracy       | ±0.2km/h                    |
| Speed resolution                 | 1.2km/h                     |
| Horizontal angle resolution      | 5°                          |
| Distance Resolution              | 0.5m                        |
| Angular Accuracy                 | 0.5°                        |
| Distance Resolution              | 1m                          |
| Ranging accuracy                 | ± 0.1m                      |
| The max num of target detections | 12                          |
| Functional Safety                | ASIL B                      |

## Side Radar Target Filter Algorithm
Single side radar can only output up to 12 targets, so it has a built-in target priority filtering algorithm.

1. Based on the center of bumper radar, calculate the 2D Euclidean distance of the target obstacle. 
2. Use the Euclidean distance to sort target obstacle, and output the obstacles with a shorter distance. 
3. For obstacles with similar Euclidean distances, determine the angle between the side face of the corresponding vehicle and the line which connects the target obstacle and the center of the radar. 
4. Output the obstacle with the smaller angle value preferentially.