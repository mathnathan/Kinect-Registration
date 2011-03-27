// ---- OpenGL -----
#include <GL/glut.h>
// ---- OpenCV -----
#include <cv.h>
#include <highgui.h>
// -- libfreenect --
#include "libfreenect.h"
#include "libfreenect_sync.h"
#include "libfreenect_cv.h"
// --- C++ ---
#include <stdio.h>
#include <fstream>
#include <vector>
#include <math.h>

using namespace cv;

// Shorten the name for easier typing
typedef pair< Point3f, Point3f > match;
typedef vector< pair< Point3f, Point3f > > matchList;
matchList correspondences;
match centroids;

// Variables for Calculations and Loops
const int NUM_CAMS = 2;
GLuint rgbTexGL;
int window;
float angle_between_cams = 0.f;
int mx=-1,my=-1;        // Prevous mouse coordinates
int rotangles[2] = {0}; // Panning angles
float zoom = 1;         // zoom factor
int color = 1;          // Use the RGB texture or just draw it as color

// Window Size and Position
const int window_width = 640, window_height = 480;
int window_xpos = 1000, window_ypos = 100;

// OpenGl Initialization Routines
void setupGL( int argc, char** argv );

// Callback Functions
void cbRender();
void cbTimer( int ms );
void cbReSizeGLScene(int Width, int Height);
void keyPressed(unsigned char key, int x, int y);
void mouseMoved(int x, int y);
void mousePress(int button, int state, int x, int y);

// Called insde the display function
void kernel();
void loadVertexMatrix();
void loadRGBMatrix();
void noKinectQuit();
void loadBuffers(int cameraIndx, short*& depthGL, unsigned char*& rgbGL, unsigned int indices[window_height][window_width], short xyz[window_height][window_width][3]);

// Computer Vision Functions
void findLines( const Mat& img1, const Mat& img2 );
matchList findFeatures( const Mat& img1, const Mat& img2 );
match calcCentroids( matchList corrs );

// Store the matrices from all cameras here
vector<Mat> rgbCV;
vector<Mat> depthCV;

int main( int argc, char** argv ) {

    rgbCV.push_back( freenect_sync_get_rgb_cv(0) );
    depthCV.push_back( freenect_sync_get_depth_cv(0) );
    rgbCV.push_back( freenect_sync_get_rgb_cv(1) );
    depthCV.push_back( freenect_sync_get_depth_cv(1) );

    setupGL( argc, argv );
    glutMainLoop();

    return 0;
}

void setupGL( int argc, char** argv ) {

    // Initialize Display Mode
    glutInit( &argc, argv);
    glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH );
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH);

    // Initialize Window
    glutInitWindowSize( window_width, window_height );
    glutInitWindowPosition( window_xpos, window_ypos );
    window = glutCreateWindow("LibFreenect");

    // Textures
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glEnable(GL_DEPTH_TEST);
    glGenTextures(1, &rgbTexGL);
    glBindTexture(GL_TEXTURE_2D, rgbTexGL);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Setup The Callbacks
    glutDisplayFunc( &cbRender );
    glutIdleFunc( &cbRender );
    glutReshapeFunc(&cbReSizeGLScene);
    cbReSizeGLScene(window_width, window_height);
    glutKeyboardFunc(&keyPressed);
    glutMotionFunc(&mouseMoved);
    glutMouseFunc(&mousePress);

    // The time passed in here needs to be the same as waitKey() for OpenCV
    glutTimerFunc( 10, cbTimer, 10);

    glScalef( .5, .5, .5 );
    glPushMatrix();
    glClear( GL_COLOR_BUFFER_BIT );

}

void keyPressed(unsigned char key, int x, int y) {

    if (key == 27) {
        freenect_sync_stop();
        glutDestroyWindow(window);
        exit(0);
    }
    if (key == 'w')
        zoom *= 1.1f;
    if (key == 's')
        zoom /= 1.1f;
    if (key == 'c')
        color = !color;
    if (key == 't') {}
}

void mouseMoved(int x, int y) {

    if (mx>=0 && my>=0) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }
    mx = x;
    my = y;
}

void mousePress(int button, int state, int x, int y) {

    if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
    }
    if (button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
        mx = -1;
        my = -1;
    }
}

void cbRender() {

    short* depthGL = 0;
    unsigned char* rgbGL = 0;
    unsigned int indices[window_height][window_width];
    short xyz[window_height][window_width][3];

    // Flush the OpenCV Mat's from last frame
    rgbCV.clear();
    depthCV.clear();

    // Todo: finish processing BEFORE clearing or we might 
    // process too long and see blank screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
//------------------------------

    for( int cam = 0; cam < NUM_CAMS; cam++ ) {

        loadBuffers(cam, depthGL, rgbGL, indices, xyz); 
        glPushMatrix();
        glScalef(zoom,zoom,1);
        glTranslatef(0,0,-3.5);
        glRotatef(rotangles[0], 1,0,0);
        glRotatef((cam*angle_between_cams)+rotangles[1], 0,1,0);
        glTranslatef(0,0,1.5);

//------------------------------
        loadVertexMatrix();
//------------------------------

        // Set the projection from the XYZ to the texture image
        glMatrixMode(GL_TEXTURE);
        glLoadIdentity();
        glScalef(1/640.0f,1/480.0f,1);
        loadRGBMatrix();
//------------------------------
        // TODO: Load a unique projection per camera to calibrate together
//------------------------------
        loadVertexMatrix();
//------------------------------
        glMatrixMode(GL_MODELVIEW);

        glPointSize(1);

        // ---------------
        glEnableClientState(GL_VERTEX_ARRAY);
        glVertexPointer(3, GL_SHORT, 0, xyz);
        glEnableClientState(GL_TEXTURE_COORD_ARRAY);
        glTexCoordPointer(3, GL_SHORT, 0, xyz);

        // ---------------
        glEnableClientState(GL_VERTEX_ARRAY);
        if (color)
            glEnable(GL_TEXTURE_2D);
        glBindTexture(GL_TEXTURE_2D, rgbTexGL);
        glTexImage2D(GL_TEXTURE_2D, 0, 3, window_width, window_height, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbGL);
        // ---------------

        glPointSize(2.0f);
        glDrawElements(GL_POINTS, window_width*window_height, GL_UNSIGNED_INT, indices);
        // ---------------

        glPopMatrix();
    }

    glDisable(GL_TEXTURE_2D);

    kernel();
    glutSwapBuffers();

}

void kernel() {

    if( correspondences.empty() ) {

        imshow("RGB_0", rgbCV[0]);
        imshow("RGB_1", rgbCV[1]);
    
        // This needs to be the same time as in cbTimer( int ms )
        char c = waitKey(10);
        if( c == 'f' ) {
            correspondences = findFeatures( rgbCV[0], rgbCV[1] );
            centroids = calcCentroids( correspondences );
        }
        if( c == 'l' )
            findLines( rgbCV[0], rgbCV[1] );

    }
    else {
    }

}

// This ensures that OpenGL and OpenCV play nicely together
void cbTimer( int ms ) {

    glutTimerFunc( ms, cbTimer, ms );
    glutPostRedisplay();

}

void cbReSizeGLScene(int Width, int Height) {

    glViewport(0,0,Width,Height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(60, 4/3., 0.3, 200);
    glMatrixMode(GL_MODELVIEW);
}


void noKinectQuit() {
    printf("Error: Kinect not connected?\n");
    exit(1);
}

void loadBuffers(int cameraIndx, short*& depthGL, unsigned char*& rgbGL, unsigned int indices[window_height][window_width], short xyz[window_height][window_width][3]) {

    // Load the new Mats
    rgbCV.push_back( freenect_sync_get_rgb_cv(cameraIndx) );
    depthCV.push_back( freenect_sync_get_depth_cv(cameraIndx) );

    // shouldn't need to make a tmp...
    Mat tmp = rgbCV[cameraIndx].clone();
    rgbGL = rgbCV[cameraIndx].data;
    cvtColor(rgbCV[cameraIndx], tmp, CV_RGB2BGR);
    rgbCV[cameraIndx] = tmp.clone();

    // Switched to loading depth from OpenCV (It's much faster, somehow)
# if 0
    uint32_t ts;
    if (freenect_sync_get_depth((void**)&depthGL, &ts, cameraIndx, FREENECT_DEPTH_11BIT) < 0)
        noKinectQuit();
    if (freenect_sync_get_video((void**)&rgbGL, &ts, cameraIndx, FREENECT_VIDEO_RGB) < 0)
        noKinectQuit();
# endif
    int i,j;
    for (i = 0; i < window_height; i++) {
        for (j = 0; j < window_width; j++) {
            xyz[i][j][0] = j;
            xyz[i][j][1] = i;
            xyz[i][j][2] = *((short *)(depthCV[cameraIndx].data + depthCV[cameraIndx].step[0]*i + depthCV[cameraIndx].step[1]*j));
            indices[i][j] = i*window_width+j;
        }
    }

}

// Do the projection from u,v,depth to X,Y,Z directly in an opengl matrix
// These numbers come from a combination of the ros kinect_node wiki, and
// nicolas burrus' posts.
void loadVertexMatrix() {

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;
    GLfloat mat[16] = {
        1/fx,     0,  0, 0,
        0,    -1/fy,  0, 0,
        0,       0,  0, a,
        -cx/fx, cy/fy, -1, b
    };
    glMultMatrixf(mat);
}

// This matrix comes from a combination of nicolas burrus's calibration post
// and some python code I haven't documented yet.
void loadRGBMatrix() {

    float mat[16] = {
        5.34866271e+02,   3.89654806e+00,   0.00000000e+00,   1.74704200e-02,
        -4.70724694e+00,  -5.28843603e+02,   0.00000000e+00,  -1.22753400e-02,
        -3.19670762e+02,  -2.60999685e+02,   0.00000000e+00,  -9.99772000e-01,
        -6.98445586e+00,   3.31139785e+00,   0.00000000e+00,   1.09167360e-02
    };
    glMultMatrixf(mat);
}

matchList findFeatures( const Mat& img1, const Mat& img2 ) {

    // Store all of the SIFT features and their respective correspondences here
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    printf("threshold = %f\n", cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD());
    printf("Edgethreshold = %f\n", cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());

    // Control the first parameter to increase or decrease the number of features found.
    // Lowering the threshold, increases the features. Raising it has the inverse effect.
    SiftFeatureDetector sift(.06, 10);

    printf("\nFinding Features....\n\n");
    // Detect sift features and store them in the keypoints
    sift.detect( img1, keypoints1 );
    sift.detect( img2, keypoints2 );

    printf("Features Found!\n\n");
    printf("Each Potential match is drawn in Red. \n\n1) If it's an accurate correspondence "  
            "\n\n    press 'y' for yes \n\n    press any other key for no \n\n2) The selected corresondences "
            "will be drawn in green on the screen. \n3) After each selection, "  
            "more 'bad' correspondences will be filtered out from the remaining matches.\n");

    // Extract the descriptors from each image for comparison and matching
    SiftDescriptorExtractor extractor;
    extractor.compute( img1, keypoints1, descriptors1 );
    extractor.compute( img2, keypoints2, descriptors2 );

    // Do some matching!
    BruteForceMatcher< L2<float> > matcher;
    vector<DMatch> matches;
    matcher.match(descriptors1, descriptors2, matches );

    // Draw the matches on to the img_matches matrix
    Mat img_matches;
    drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);

    // Find Out Some Information 
#if 0
    for( int i = 0; i < matches.size(); i++ ) {
        double x1 = keypoints1[matches[i].queryIdx].pt.x;
        double y1 = keypoints1[matches[i].queryIdx].pt.y;
        double x2 = keypoints2[matches[i].trainIdx].pt.x;
        double y2 = keypoints2[matches[i].trainIdx].pt.y;
        printf("matches[%d].queryIdx = %d\n", i,matches[i].queryIdx);
        printf("keypoints1[matches[%d].queryIdx].pt.x = %f\n", i,x1);
        printf("keypoints1[matches[%d].queryIdx].pt.y = %f\n", i,y1);
        printf("keypoints2[matches[%d].trainIdx].pt.x = %f\n", i,x2);
        printf("keypoints2[matches[%d].trainIdx].pt.y = %f\n", i,y2);
        double d = sqrt( pow(x2-x1,2) + pow(y2-y1,2) );
        printf("Distance between keypoints = %f\n", d);
        printf("matches[%d].distance = %f\n", i,matches[i].distance);
        printf("matches[%d].trainIdx = %d\n", i,matches[i].trainIdx);
        printf("matches[%d].imgIdx = %d\n", i,matches[i].imgIdx);
    }
#endif

    // Put both images side by side in one image
    Mat rslt = Mat::zeros(img1.rows, img1.cols*2, img1.type());
    for( int i = 0; i < img1.rows; i++ )
        for( int j = 0; j < img1.cols; j++ ) {
            Vec3b p = img1.at<Vec3b>(i,j);
            Vec3b q = img2.at<Vec3b>(i,j);
            rslt.at<Vec3b>(i,j) = p;
            rslt.at<Vec3b>(i,j+window_width) = q;
        }



    // -----------------Examine each coorespondence one at a time--------------

    // Store the cumulative slopes of correspondences here
    double totSlope;
    // This is where we'll store the correspondences for future calculations
    matchList corrs;

    int i;
    for( i=0; i < (int)matches.size(); i++ ) {
        Mat tmp = rslt.clone();
        // These are the actual points in the matrices
        Point2f pt1 = keypoints1[matches[i].queryIdx].pt;
        Point2f pt2 = keypoints2[matches[i].trainIdx].pt;
        // These will be the points transformed to the joined image coordinate system
        // to use in slope calculations and comparisons
        Point2f transpt1 = Point2f(pt1.x,window_height-pt1.y);
        Point2f transpt2 = Point2f(pt2.x+window_width,window_height-pt2.y);
        // Draw circles around the correspondences
        circle( tmp, pt1, 3, Scalar(0,0,255), 1, CV_AA );
        circle( tmp, pt2+Point2f(window_width,0), 3, Scalar(0,0,255), 1, CV_AA );
        line( tmp, pt1, pt2+Point2f((float)window_width,0.0), Scalar(0,0,255), 1, CV_AA );
        // Show this match and ask the user if it's an accurate correspondence
        imshow("Matching Correspondences", tmp);
        char c = waitKey(0);
        if( c == 'y' || c == 'Y' ) {
            // Store the correspondences in a vector of pairs, where the members of each pair are points
            corrs.push_back( match( Point3f(pt1.x,pt1.y,0), Point3f(pt2.x,pt2.y,0) ) );
            // Calculate the slope of the line between these, to filter out bad future matches
            float slp = (transpt2.y-transpt1.y)/(transpt2.x-transpt1.x);
            totSlope = slp; 
            printf("-------------------MACTH ACCEPTED------------------\n");
            printf("(x1, y1) (x2, y2) = (%f,%f) (%f,%f)\n", transpt1.x, transpt1.y, transpt2.x, transpt2.y);
            printf("pt = %d, slope = %f\n",i,slp);
            break;
        }
    }
    // Continue going through the matches to find more accurate correspondences
    for( i=i+1; i < (int)matches.size(); i++ ) {
        bool broken = false;
        Mat tmp = rslt.clone();
        // These are the actual points in the matrices
        Point2f pt1 = keypoints1[matches[i].queryIdx].pt;
        Point2f pt2 = keypoints2[matches[i].trainIdx].pt;
        // These will be the points transformed to the joined image coordinate system
        // to use in slope calculations and comparisons
        Point2f transpt1 = Point2f(pt1.x,window_height-pt1.y);
        Point2f transpt2 = Point2f(pt2.x+window_width,window_height-pt2.y);
        float slp = (transpt2.y-transpt1.y)/(transpt2.x-transpt1.x);
        printf("\n-----------CORRESPONDENCE CANDIDATE--------\n"); 
        printf("(x1, y1) (x2, y2) = (%f,%f) (%f,%f)\n", transpt1.x, transpt1.y, transpt2.x, transpt2.y);
        printf( "match # = %d, slope = %f, abs( slp - average )  = %f\n", i, slp, abs(slp - totSlope/corrs.size()) );
        printf("Avgerage Slope = %f\n", totSlope/corrs.size());
        printf("Correspondences Found = %d\n", (int)corrs.size());

        // ---------------------------Filter Out Points----------------------

        // If slope is not close to average don't even look at it
        if( abs( slp - totSlope/corrs.size() ) > .05 ) {
            printf("-----------------INVALID SLOPE---------------\n");
            printf( "slope = %f, abs( slp - average )  = %f\n", slp, abs(slp - totSlope/corrs.size()) );
            continue;
        }

        // If this point is one of the current correspondences, get rid of it
        // because one point can't have two distinct correspondences
        for( int j = 0; j < (int)corrs.size(); j++ ) {

            // Draw current correspondences in green
            Point2f firstpt2D = Point2f(corrs[j].first.x,corrs[j].first.y);
            Point2f secondpt2D = Point2f(corrs[j].second.x,corrs[j].second.y);
            circle( tmp, firstpt2D, 3, Scalar(0,255,0), 1, CV_AA );
            circle( tmp, secondpt2D+Point2f(window_width,0), 3, Scalar(0,255,0), 1, CV_AA );
            line( tmp, firstpt2D, secondpt2D+Point2f(window_width,0), Scalar(0,255,0), 1, CV_AA );
            if( pt1 == firstpt2D || pt2 == secondpt2D ) {
                broken = true;
                break;
            }
        }
        if( broken ) {
            broken = false;
            continue;
        }

        // -------------------------------------------------------------------

        // Draw each match in question in Red so the user can see it and decide if it's accurate
        circle( tmp, pt1, 3, Scalar(0,0,255), 1, CV_AA );
        circle( tmp, pt2+Point2f(window_width,0), 3, Scalar(0,0,255), 1, CV_AA );
        line( tmp, pt1, pt2+Point2f((float)window_width,0.0), Scalar(0,0,255), 1, CV_AA );
        imshow("Matching Correspondences", tmp);
        char c = waitKey(0);
        if( c == 'y' || c == 'Y' ) {
            // Store the correspondences in a vector of pairs, where the members of each pair are points
            corrs.push_back( match( Point3f(pt1.x,pt1.y,0), Point3f(pt2.x,pt2.y,0) ) );
            totSlope += slp;
            printf("-------------------MACTH ACCEPTED------------------\n");
            printf("(x1, y1) (x2, y2) = (%f,%f) (%f,%f)\n", transpt1.x, transpt1.y, transpt2.x, transpt2.y);
            printf("pt = %d, slope = %f\n",i,slp);
        }
        else printf("-------------------MATCH DENIED---------------\n");
    }

    printf("\n\nCorrespondences have been found!\n\n");

    // Show all of the matches to compare to those the user decided to be accurate correspondences
    imshow("SIFT matches", img_matches);

    return corrs;

}

void findLines( const Mat& img1, const Mat& img2 ) {

    Mat dst1, dst2, cdst1, cdst2;

    // convert to gray scale 
    cvtColor(img1, dst1, CV_BGR2GRAY);
    cvtColor(img2, dst2, CV_BGR2GRAY);

    // detect edges
    Canny(dst1, cdst1, 50, 200, 3);
    Canny(dst2, cdst2, 50, 200, 3);

    // convert back to color
    printf("cdst1 channels = %d\n", cdst1.channels()); 
    printf("cdst1 type = %d\n", cdst1.type()); 

    vector<Vec4i> lines1, lines2;
    HoughLinesP(cdst1, lines1, .5, CV_PI/180, 125, 50, 10 );
    HoughLinesP(cdst2, lines2, .5, CV_PI/180, 125, 50, 10 );
    cvtColor(dst1, cdst1, CV_GRAY2BGR);
    cvtColor(dst2, cdst2, CV_GRAY2BGR);
    for( size_t i = 0; i < lines1.size(); i++ )
    {
        Vec4i l = lines1[i];
        line( cdst1, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
    }
    for( size_t i = 0; i < lines2.size(); i++ )
    {
        Vec4i l = lines2[i];
        line( cdst2, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, CV_AA);
    }

    Mat rslt = Mat::zeros(cdst1.rows, cdst1.cols*2, cdst1.type());
    for( int i = 0; i < cdst1.rows; i++ )
        for( int j = 0; j < cdst1.cols; j++ ) {
            Vec3b p = cdst1.at<Vec3b>(i,j);
            Vec3b q = cdst2.at<Vec3b>(i,j);
            rslt.at<Vec3b>(i,j) = p;
            rslt.at<Vec3b>(i,j+window_width) = q;
        }

    imshow("detected lines", rslt);
}

match calcCentroids( matchList corrs ) {

    printf("\n\nCalculating the centroids for each cloud of correspondences\n");

    Point3f centr1(0,0,0), centr2(0,0,0);

    for( int i = 0; i < (int)corrs.size(); i ++ ) {

        // First image 
        centr1.x += corrs[i].first.x; 
        centr1.y += corrs[i].first.y; 
        centr1.z += corrs[i].first.y; 

        // Second image
        centr2.x += corrs[i].second.x;
        centr2.y += corrs[i].second.y;
        centr2.z += corrs[i].second.y; 

    }

    centr1.x /= corrs.size();
    centr1.y /= corrs.size();
    centr1.z /= corrs.size();
    centr2.x /= corrs.size();
    centr2.y /= corrs.size();
    centr2.z /= corrs.size();

    match centroids(centr1, centr2);

    printf("\nCorrespondence points in Image 1\n");
    for( int i = 0; i < (int)corrs.size(); i++ ) 
        printf("%d: (%f,%f)\n", i, corrs[i].first.x, corrs[i].first.y);
    printf("\nCorrespondence points in Image 2\n");
    for( int i = 0; i < (int)corrs.size(); i++ ) 
        printf("%d: (%f,%f)\n\n", i, corrs[i].second.x, corrs[i].second.y);

// Put both images side by side in one image
    Mat rslt = Mat::zeros(rgbCV[0].rows, rgbCV[0].cols*2, rgbCV[0].type());
    for( int i = 0; i < rgbCV[0].rows; i++ )
        for( int j = 0; j < rgbCV[0].cols; j++ ) {
            Vec3b p = rgbCV[0].at<Vec3b>(i,j);
            Vec3b q = rgbCV[1].at<Vec3b>(i,j);
            rslt.at<Vec3b>(i,j) = p;
            rslt.at<Vec3b>(i,j+window_width) = q;
        }

// Draw green circles around the correspondences in each image
    for( int i = 0; i < (int)corrs.size(); i++ ) {
        circle( rslt, Point2f( corrs[i].first.x, corrs[i].first.y ), 3, Scalar(0,255,0), 1, CV_AA );
        circle( rslt, Point2f( corrs[i].second.x, corrs[i].second.y )+Point2f(window_width,0), 3, Scalar(0,255,0), 1, CV_AA );
    }
        

    circle( rslt, Point2f(centr1.x,centr1.y), 3, Scalar(0,0,255), 1, CV_AA );
    circle( rslt, Point2f(centr2.x,centr2.y)+Point2f(window_width,0), 3, Scalar(0,0,255), 1, CV_AA );

    printf("Centroid 1 (x,y) = (%f,%f)\n", centr1.x, centr1.y);
    printf("Centroid 2 (x,y) = (%f,%f)\n", centr2.x, centr2.y);
    imshow("Centroids", rslt);
    waitKey(0);

    return centroids;

} 

#if 0
Mat calcRotation( matchList corrs ) {

    SVD svd();

}
#endif 
