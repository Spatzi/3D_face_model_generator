Setup the environment:
Install PCL - http://pointclouds.org/downloads/
For Windows use the dependencies' stand alone installers. Build the Point Cloud Library from source following the instructions here http://pointclouds.org/documentation/tutorials/compiling_pcl_windows.php#compiling-pcl-windows . For this CMake tool is essential http://www.cmake.org/download/ .

Install OpenCV (at least 2.4.9) - http://opencv.org/downloads.html
Add all required dependencies of OpenCV to PCL’s kinfu_app project.

For Microsoft Visual Studio follow the instructions here 
http://docs.opencv.org/doc/tutorials/introduction/windows_visual_studio_Opencv/windows_visual_studio_Opencv.html .

In kinfu_app project, replace kinfu_app.cpp with our file, and add FaceDetection.cpp and 3DfaceDetection_GIP.h to the project as well.

Running the Application
In order to run our application, one should face a Kinect camera, then run the command with the following flags:
‘pcl_kinfu_app_release.exe -r -ic’
This will run the kinfu_app application with our work in it. The flags are for enabling colors integration.
While the application is running, the built model will be presented on screen.
The subject should make slow movements of his/her head, in all directions.
When the constructed model’s resolution is satisfying, one should press ‘d’ in the keyboard.
This will generate a colored mesh, and will save it as a ply file under the name ‘mesh.ply’ in the working directory.