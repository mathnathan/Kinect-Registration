Author: Nathan Crock
        mathnathan@gmail.com
        http://www.mathnathan.com


Date:    03/23/11    14:53




----------- Kinect Camera Registraion ------------



OS SUPPORT: Only tested on Linux so far

            Ubuntu


FILES:
        KinReg.cpp - The registraion program
        CMakeLists.txt - Cmake file with build commands
        include |
                | libfreenect - open source driver for the kinect


DEPENDENCIES:   OpenCV
                OpenGL
                GLUT

    DISCLAIMER:

        I've bundled a slimmed down version of the libfreenect driver
        into this package. Get it straight from git if you want all 
        their samples and goodies. 

            libfreenect: https://github.com/OpenKinect/libfreenect

        Also check out their wiki to learn about the organization
        that put it together.

            OpenKinect: http://openkinect.org/wiki/Main_Page 


COMPILATION INSTRUCTIONS: 

-If you don't have cmake you can get it with aptitude

    sudo apt-get install cmake

-For a breif introduction to CMake follow my tutorial

    http://mathnathan.com/2010/07/11/getting-started-with-cmake/

-Make a build directory and from it run 

    cmake ..
    make

    NOTE: It should "just work". Please let me know otherwise


-------------------------------------------------------------------------------


    This program is intended to register multiple point clouds
    into one coordinate system. The point clouds are obtained
    from Microsoft's Kinect sensors.

        NOTE: Only support for 2 cameras for now

    It begins by finding correspondences between the two cameras.
    The correspondences are found using SIFT features.

        Once KinReg is running, press 'f' to launch the feature finder.

    It will usually find a few hundred features, a lot of which are no good.
    To filter out the bad features, the selection process is interactive.
    It will display the correspondences one at a time by connecting two
    small circles with a line.

        If it's a good features press 'y', if not press any other key

    After you give it one good correspondence, it will filter out a
    majority of the remaining bad correspondences, and the task will be much
    easier. It usually takes about 30 seconds to find 20-30 good features.

    I am currently still working on implenting a form of Procrustes Analysis
    to do the translation and rotation of the point clouds, to align the
    correspondences.
