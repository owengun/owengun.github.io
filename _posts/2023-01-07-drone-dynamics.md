---
layout: post
title: "Drone Study: Drone Dynamics"
excerpt_separator: "<!--more-->"
categories:
  - Drone Study
tags:
  - Drone
  - Dynamics
use_math: true
---
# Introduction

At the beginning of my junior year in DGIST (Daegu Kyeonbuk Institute of Science and Technology), I set my UGRP (Undergraduate Group Research Program) project's theme as drone. At that very moment, I did not know much about drone, although I like aircraft. However, I was aware of the future potential of it. Therefore, I began to study about drone.

As a candidate to be an Engineer, it was natural for me to study its dynamics, from which many things can be derived: control, hardware, software, electrics, sensors, etc..
<br> </br>
<br> </br>

# Coordinate Definition

Before figuring out dynamics of drone, it's imperative to know about the coordinate system; drone (body frame) and the global world (reference frame) are not the same.
<br/><br>

<img src ="./ref_frames.png" width = "" height = "" title ="PX4 Reference frame">
[Image 1]: Coordinate frames used in PX4 program (Left) and coordinate frames used typically (Right) <br/>source: PX4 Official Website<br>

<br/><br>
One needs to make a transformation matrix from the reference fame to body frame. To do so, rotating one at a time (thus making a transformation matrix each and multiplying them) will be sufficient.
<br/>
There are lots of seqeunces of rotation (X-Y-Z, Z-Y-X, etc.). I used Z-Y-X rotation, which is convention. 
<br/>
<br/>

First, let's define notation:

$$[x^E, y^E, z^E]$$: Reference Frame (Earth)

$[x^B, y^B, z^B]$: Body Frame (Drone)

$[\phi, \theta, \psi]$: orientation of quadcopter (roll, pitch, yaw)
<br/><br>

As I mentioned earlier, I will use Z-Y-X convention:
<br/>
<br/>1st: rotation about Z-axis:

$\begin{bmatrix}x_b^1\\y_b^1\\z_b^1\end{bmatrix} = \begin{bmatrix}sin(\psi) & sin(\psi) & 0\\-sin(\psi) & cos(\psi) &0\\0 & 0 & 1\end{bmatrix}\begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix} = R_{\psi} \begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix}$

$[x_b^1, y_b^1, z_b^1]$: body frame when rotated about Z-axis
</br>
</br>
2nd: rotation about $y_b^1$-axis

$\begin{bmatrix}x_b^2\\y_b^2\\z_b^2\end{bmatrix} = \begin{bmatrix}cos(\theta) & 0 & -sin(\theta)\\0 & 1 & 0\\sin(\theta) & 0 & cos(\theta)\end{bmatrix}\begin{bmatrix}x_b^1\\y_b^1\\z_b^1\end{bmatrix} = R_{\theta} \begin{bmatrix}x_b^1\\y_b^1\\z_b^1\end{bmatrix}$

$[x_b^2, y_b^2, z_b^2]$: body frame when rotated about Z-axis
</br>
</br>
3rd: rotation about $z_b^2$-axis

$\begin{bmatrix}x^B\\y^B\\z^B\end{bmatrix} = \begin{bmatrix}1 & 0 & 0\\0 & cos(\phi) & sin(\phi)\\0 & -sin(\phi) & cos(\phi)\end{bmatrix}\begin{bmatrix}x_b^2\\y_b^2\\z_b^2\end{bmatrix} = R_{\psi} \begin{bmatrix}x_b^2\\y_b^2\\z_b^2\end{bmatrix}$

</br>
</br>
Combination:

<br/>

$\begin{bmatrix}x^B\\y^B\\z^B\end{bmatrix} = \begin{bmatrix} cos(\theta)cos(\phi) & cos(\theta)sin(\phi) & -sin(\theta)\\ sin(\phi)sin(\theta)cos(\psi) - cos(\phi)sin(\psi) & sin(\phi)sin(\theta)sin(\psi) + cos(\phi)cos(\psi) & sin(\phi)cos(\theta)\\ cos(\phi)sin(\theta)cos(\psi) + sin(\phi)sin(\psi) & cos(\phi)sin(\theta)sin(\psi) - sin(\phi)cos(\psi) & cos(\phi)cos(\theta) \end{bmatrix}\begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix} = R\begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix}$

<br/>

since $R$ is orthonormal matrix,

$R^{-1} =R^T$

$\therefore \begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix} = R^{T}\begin{bmatrix}x^B\\y^B\\z^B\end{bmatrix} = \begin{bmatrix} cos(\theta)cos{\psi} & sin(\phi)sin(\theta)cos(\psi) - cos(\phi)sin(\psi) &  cos(\phi)sin(\theta)cos(\psi) + sin(\phi)sin(\psi) \\ cos(\theta)sin(\psi) & sin(\phi)sin(\theta)sin(\psi) + cos(\phi)cos(\psi) & cos(\phi)sin(\theta)sin(\psi) - sin(\phi)cos(\psi) \\ -sin(\theta)  & sin(\phi)cos(\theta) & cos(\phi)cos(\theta) \end{bmatrix}\begin{bmatrix}x^E\\y^E\\z^E\end{bmatrix}$

<br/>

while the equation above is the rotational matrix for position, velocity, and force, there is a separate matrix from body to global regarding angular rates:
<br/><br>

$\omega = [p, q, r]$: Angular rate for body frame (drone)

$[\frac{d\phi}{dt}, \frac{d\theta}{dt}, \frac{d\psi}{dt}]$: Angular rate from reference frame (Earth)

<br/><br>

One needs to understand that for this case, there's no need to rotate about all sequence (Z-X-Y). One can think about projecting into one coordinate frame level (e.g. $Ox_b^2y_b^2z_b^2$, before final rotation) 

Therefore,

$\omega = \begin{bmatrix} p \\ q \\ r \end{bmatrix} = R_{\phi}R_{\theta} \begin{bmatrix} 0 \\ 0 \\ \frac{d\psi}{dt} \end{bmatrix} + R_{\phi} \begin{bmatrix} 0 \\ \frac{d\theta}{dt} \\ 0 \end{bmatrix} + \begin{bmatrix} \frac{d\phi}{dt} \\ 0 \\ 0 \end{bmatrix}= \begin{bmatrix} 1 & 0 & -sin(\theta) \\ 0 & cos(\phi) & sin(\phi)cos(\theta) \\ 0 & -sin(\phi) & cos(\phi)cos(\theta) \end{bmatrix} \begin{bmatrix}  \frac{d\phi}{dt} \\ \frac{d\theta}{dt} \\ \frac{d\psi}{dt} \end{bmatrix} $

$\therefore \begin{bmatrix}  \frac{d\phi}{dt} \\ \frac{d\theta}{dt} \\ \frac{d\psi}{dt} \end{bmatrix} = \begin{bmatrix} 1 & sin(\phi)tan(\theta) & cos(\phi)tan(\theta) \\ 0 & cos(\phi) & -sin(\phi) \\ 0 & \frac{sin(\phi)}{cos(\theta)} & \frac{cos(\phi)}{cos(\theta)} \end{bmatrix}$
<br></br>
<br></br>

# Dynamics

< to be continued... >