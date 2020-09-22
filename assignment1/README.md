# 3DSSF
## Subtask 1: Image matching

   - Read images using OpenCV or a similar tool.
   - Design and implement these stereo-matching schemes:
        - Naive stereo matching.
        - Dynamic programming approach.
        - Note: make the parameters (window size, weigths) tunable -- GUI or commandline!
        - Optional: compare your solution to an existing implementation (e.g. in OpenCV)!
   - Display the output disparity images (e.g. using OpenCV).

## Subtask 2: 3D display

Given the input camera parameters convert the disparities to 3D point cloud. Either

    -save it as a file (.xyz or .ply file formats suggested) and display it using MeshLab,or use some form of 3D graphical display directly from your application.


## Compiling
```
g++ image_matching.cpp -o image_matching -std=c++11 `pkg-config --cflags --libs opencv`
```

##Usage
```
./image_matching view1.png  view2.png 
```
