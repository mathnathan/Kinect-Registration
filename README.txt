Author: Nathan Crock
        mathnathan@gmail.com
        http://www.mathnathan.com


Date:    03/23/11    14:53


----------- Kinect Camera Registraion ------------


OS SUPPORT: Only tested on Linux so far

            Ubuntu 12.04, 12.10

FILES:
        KinReg.cpp - The registraion program
        CMakeLists.txt - Cmake file with build commands
        cmake |
              | find |
                     | Find*.cmake all of the cmake files to find the
                     | dependencies


DEPENDENCIES:   Freenect
                OpenCV
                OpenGL
                GLUT

COMPILATION INSTRUCTIONS: 

-If you don't have cmake you can get it with aptitude

    sudo apt-get install cmake

-For a breif introduction to CMake follow my tutorial

    http://mathnathan.com/2010/07/11/getting-started-with-cmake/

-Make a build directory and from it run 

    cmake ..
    make

	NOTE: If you have the dependencies installed (to your root directories, or
	their paths in your ldpaths), then it should "just work" in UBUNTU 12.04 and
	12.10. Please let me know otherwise


---------------------------Simple Project Manual--------------------------------------


    This program is intended to register multiple point clouds
    into one coordinate system. The point clouds are obtained
    from Microsoft's Kinect sensors.

        NOTE: Only support for 2 cameras for now

    It begins by finding correspondences between the two cameras.
    The process is interactive.

        Once KinReg is running, a window pops up containing both images side by side.

	All you do is click on pixels in both cameras which correspond to the same
	point in world coordinates. Currently the process is rough, it can be
	improved in the future by drawing circles around the points you click on
	and adding colors to signify if the correspondence is accepted or not. If
	both depth measurements for your clicks are found it stores the
	correspondence. 
	
		If at least one of the corespondences is bad (no depth) it deletes the match.

	Once you've collected around 10 or so correspondences pushing 'p' applies 
	procrustes analysis on the correspondences and calculates the transformation 
	matrices. 

		Press 't' to see the translation of the centroids to the origin

		Press 'a' to see the translation and rotation applied to both point clouds

======================================================================================

	"Believing in the way, makes the way!"



