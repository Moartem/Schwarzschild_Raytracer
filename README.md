# Schwarzschild_Raytracer
This project implements a realtime raytracer in the Schwarzschild-metric (a sphere with a black hole inside).

It is written with Visual Studio in C++, but has no real dependence on it.  
It used FreeGlut 3.0.0 and GLEW 1.13.0 for OpenGl and SOIL (licenses included).

You can refer to my bachelor thesis about background info:  
[link to be placed here]

Feel free to modify, improve, or use functionality of this code (like proposed in chapter 7 of the theses, easy integration into conventional ray tracers)

To make the executeable work, make sure you have the right vs redistributable installed, the dlls available to the .exe and the fragment shader in the same directory. You further need a picture in the same folder, whose name is to enter in the console. For pictures I recommend the NASA website.

If anyone is intrested, I could provide more information.

###Controls:
Click the screen and move your mouse for view  

1 stationary mode  
2 free falling snapshot  
3 free fall  
4 orbit  

WASD horizontal movement  
QE vertical movement  

P reset  
