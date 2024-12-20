# infraredHorizonSensor
A project to build a prototype infrared horizon sensor using a thermopile imaging array for CubeSat orbital attitude determination.

Check out the [video presentation summarising the research here](https://youtu.be/ipN0h2YGPOc?si=D6N8nWLN3bS8p15P).

Traditional image-based horizon sensors provide orientation information by detecting planetary edges against the background of space, a process often performed using visible light cameras due to the typically limited resolution of infrared imaging sensors. This study illustrates some of the advantages of using the infrared band for horizon detection, and explores recent advancements in sub-pixel edge localisation techniques, specifically the Zernike polynomials, to improve the performance of low-resolution thermal imaging sensors for horizon detection. To evaluate the utility of this technique, a prototype infrared horizon sensor and simulated infrared orbital view test stand was built to assess its performance.

<img src="_doc/images/_horizonSensor_gimbal_testStand.jpg" alt="Simulated infrared orbital horizon view, and horizon sensor prototype mounted to pitch-roll gimbal" width="600">

## Further Details
See research thesis at: [Infrared Horizon Sensor Thesis.pdf](_doc/_InfraredHorizonSensorThesis.pdf)

## Examples

*Infrared horizon sensor and example orbital attitude determination of planets nadir vector:*
<p align="left">
  <img src="_doc/images/PCB_Assembled.jpg" alt="Prototype infrared horizon sensor PCB" height="300">
  <img src="_doc/images/attitudeExample_R(z) -20deg.PNG" alt="example nadir vector relative attitude estimation" height="300">
</p>

*Coarse pixel-level edge detection, image gradient vector field represented as 3D surface:*

<img src="_doc/images/imageGradientVectorField_3DSurface.PNG" alt="Coarse edge detection image gradient vector field" width="600">

*Example sub-pixel correction to detected pixel-level orbital horizon edge:* 

<img src="_doc/images/subPixelEx_top.PNG" alt="orbital horizon edge sub-pixel correction example 2" width="500">
<img src="_doc/images/subPixelEx_right.PNG" alt="orbital horizon edge sub-pixel correction example 1" width="500">
<img src="_doc/images/subPixelEx_topLeft.PNG" alt="orbital horizon edge sub-pixel correction example 3" width="500">

*Example of Program Execution Flowchart:*

<img src="_doc/images/ProgramFlowChart.drawio.png" alt="Program Execution Flowchart" width="900">

## Permissive Open-Source Licensing
- **Software**: Licensed under the permissive [MIT License](LICENSE-MIT).
- **Hardware Design Files**: Licensed under the [CERN-OHL-Permissive Open-Source License](LICENSE-CERN-OHL-P).
