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
typedef pair< Vec4f, Vec4f > match;
typedef vector< match > matchList;
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
void cbReSizeGLScene( int Width, int Height);
void keyPressed( unsigned char key, int x, int y);
void mouseMoved( int x, int y);
void mousePress( int button, int state, int x, int y);

// Called insde the display function
void kernel();
void loadVertexMatrix();
void loadRGBMatrix();
void noKinectQuit();
void loadBuffers( int cameraIndx, unsigned int indices[window_height][window_width], short xyz[window_height][window_width][3]);

// Computer Vision Functions
Mat joinFrames( const Mat& img1, const Mat& img2 );
matchList findFeatures( const Mat& img1, const Mat& img2 );
void projectDepth( matchList& corrs );
match calcCentroids( const matchList& corrs, const Mat& img1, const Mat& img2 );
Mat calcRotation( matchList corrs );


// Store the matrices from all cameras here
vector<Mat> rgbCV;
vector<Mat> depthCV;

int main( int argc, char** argv ) {

	// load the first frames 
	for( int cam = 0; cam < NUM_CAMS; cam++ ) {
		rgbCV.push_back( freenect_sync_get_rgb_cv(cam) );
		depthCV.push_back( freenect_sync_get_depth_cv(cam) );
	}

    setupGL( argc, argv );
    glutMainLoop();

    return 0;
}

void setupGL( int argc, char** argv ) {

    // Initialize Display Mode
    glutInit( &argc, argv );
    glutInitDisplayMode( GLUT_RGB | GLUT_DEPTH );
    glutInitDisplayMode( GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH );

    // Initialize Window
    glutInitWindowSize( window_width, window_height );
    glutInitWindowPosition( window_xpos, window_ypos );
    window = glutCreateWindow("Kinect Registration");

    // Textures and Color
    glClearColor( 0.0f, 0.0f, 0.0f, 0.0f);
    glEnable( GL_DEPTH_TEST);
    glGenTextures( 1, &rgbTexGL);
    glBindTexture( GL_TEXTURE_2D, rgbTexGL);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Setup The Callbacks
    glutDisplayFunc( &cbRender );
    glutIdleFunc( &cbRender );
    glutReshapeFunc( &cbReSizeGLScene);
    cbReSizeGLScene( window_width, window_height);
    glutKeyboardFunc( &keyPressed);
    glutMotionFunc( &mouseMoved);
    glutMouseFunc( &mousePress);

    // The time passed in here needs to be the same as waitKey() for OpenCV
    glutTimerFunc( 10, cbTimer, 10);

    glScalef( .5, .5, .5 );
    glPushMatrix();
    glClear( GL_COLOR_BUFFER_BIT );

}

void keyPressed( unsigned char key, int x, int y ) {

	// Press esc to exit
    if ( key == 27 ) {
        freenect_sync_stop();
        glutDestroyWindow( window );
        exit( 0 );
    }
    if( key == 'f' ) {
		correspondences = findFeatures( rgbCV[0], rgbCV[1] );
		projectDepth( correspondences );
		centroids = calcCentroids( correspondences, rgbCV[0], rgbCV[1] );
		calcRotation( correspondences );
	}
    if ( key == 'w' )
        zoom *= 1.1f;
    if ( key == 's' )
        zoom /= 1.1f;
    if ( key == 'c' )
        color = !color;
    if ( key == 't' ) {}
}

void mouseMoved( int x, int y) {

    if ( mx>=0 && my>=0) {
        rotangles[0] += y-my;
        rotangles[1] += x-mx;
    }

    mx = x;
    my = y;

}

void mousePress( int button, int state, int x, int y) {

    if ( button == GLUT_LEFT_BUTTON && state == GLUT_DOWN) {
        mx = x;
        my = y;
    }

    if ( button == GLUT_LEFT_BUTTON && state == GLUT_UP) {
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
    glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
    glLoadIdentity();

//------------------------------

    for( int cam = 0; cam < NUM_CAMS; cam++ ) {

        loadBuffers( cam, indices, xyz ); 
        glPushMatrix();
        glScalef( zoom,zoom,1 );
        glTranslatef( 0,0,-3.5 );
        glRotatef( rotangles[0], 1,0,0 );
        glRotatef( (cam*angle_between_cams)+rotangles[1], 0,1,0 );
        glTranslatef( 0,0,1.5 );

//------------------------------
        loadVertexMatrix();
//------------------------------

        // Set the projection from the XYZ to the texture image
        glMatrixMode( GL_TEXTURE) ;
        glLoadIdentity();
        glScalef( 1/640.0f,1/480.0f,1 );
        loadRGBMatrix();

        // TODO: Load a unique projection per camera to calibrate together

//------------------------------
        loadVertexMatrix();
//------------------------------

        glMatrixMode( GL_MODELVIEW );

        glPointSize( 1 );

        // ---------------
        glEnableClientState( GL_VERTEX_ARRAY );
        glVertexPointer( 3, GL_SHORT, 0, xyz );
        glEnableClientState( GL_TEXTURE_COORD_ARRAY ); glTexCoordPointer( 3, GL_SHORT, 0, xyz );

        // ---------------
        glEnableClientState( GL_VERTEX_ARRAY );
        if ( color ) 
            glEnable( GL_TEXTURE_2D );
        glBindTexture( GL_TEXTURE_2D, rgbTexGL );
        glTexImage2D( GL_TEXTURE_2D, 0, 3, window_width, window_height, 0, GL_RGB, GL_UNSIGNED_BYTE, rgbCV[cam].data );

        // ---------------
        glPointSize( 2.0f );
        glDrawElements( GL_POINTS, window_width*window_height, GL_UNSIGNED_INT, indices );
        // ---------------

        glPopMatrix();
    }

    glDisable( GL_TEXTURE_2D );

    kernel();
    glutSwapBuffers();
}

void kernel() {

	bool CVWindows_Open;

    if( correspondences.empty() ) {

		Mat tmp = rgbCV[0].clone();
		for( int cam = 0; cam < NUM_CAMS; cam++ ) {
			cvtColor( rgbCV[cam], tmp, CV_RGB2BGR );
			rgbCV[cam] = tmp.clone();
		}

		Mat rgb = joinFrames( rgbCV[0], rgbCV[1] );
        imshow( "Camera 0 | Camera 1", rgb );
    
		// Time here needs to be the same as cbTimer
		// returns -1 if no key pressed
		char key = waitKey( 10 );

		// If someone presses a button while a cv window 
		// is in the foreground we want the behavior to
		// be the same as for the OpenGL window, so call 
		// OpenGL's keyPressed callback function
		if( key != -1 ) 
			keyPressed( key, 0, 0 );
		CVWindows_Open = 1;
    }
    else {
		if( CVWindows_Open ) {
			cvDestroyWindow("Camera 0 | Camera 1");
		}
	}

}

// This ensures that OpenGL and OpenCV play nicely together
void cbTimer( int ms ) {

    glutTimerFunc( ms, cbTimer, ms );
    glutPostRedisplay();

}

void cbReSizeGLScene( int Width, int Height) {

    glViewport( 0,0,Width,Height );
    glMatrixMode( GL_PROJECTION );
    glLoadIdentity();
    gluPerspective( 60, 4/3., 0.3, 200 );
    glMatrixMode( GL_MODELVIEW );
}


void noKinectQuit() {
    printf( "Error: Kinect not connected?\n" );
    exit( 1 );
}

void loadBuffers( int cameraIndx, 
				unsigned int indices[window_height][window_width], 
				short xyz[window_height][window_width][3] ) {

    // Load the new Mats
    rgbCV.push_back( freenect_sync_get_rgb_cv(cameraIndx) );
    depthCV.push_back( freenect_sync_get_depth_cv(cameraIndx) );

    // shouldn't need to make a tmp...

    // Switched to loading depth from OpenCV (It's much faster, somehow)
    for ( int i = 0; i < window_height; i++) {
        for ( int j = 0; j < window_width; j++) {
            xyz[i][j][0] = j;
            xyz[i][j][1] = i;
            xyz[i][j][2] = *( (short *)(depthCV[cameraIndx].data + 
							depthCV[cameraIndx].step[0]*i + depthCV[cameraIndx].step[1]*j));
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
    glMultMatrixf( mat);
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
    glMultMatrixf( mat);
}

Mat joinFrames( const Mat& img1, const Mat& img2 ) {

	Mat rslt = Mat::zeros( img1.rows, img1.cols*2, img1.type());
    for( int i = 0; i < img1.rows; i++ )
        for( int j = 0; j < img1.cols; j++ ) {
            Vec3b p = img1.at<Vec3b>( i,j);
            Vec3b q = img2.at<Vec3b>(i,j);
            rslt.at<Vec3b>(i,j) = p;
            rslt.at<Vec3b>(i,j+window_width) = q;
        }

	return rslt;
}

matchList findFeatures( const Mat& img1, const Mat& img2 ) {

    // Store all of the SIFT features and their respective correspondences here
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;

    printf( "threshold = %f\n", cv::SIFT::DetectorParams::GET_DEFAULT_THRESHOLD());
    printf( "Edgethreshold = %f\n", cv::SIFT::DetectorParams::GET_DEFAULT_EDGE_THRESHOLD());

    // Control the first parameter to increase or decrease the number of features found.
    // Lowering the threshold, increases the features. Raising it has the inverse effect.
    SiftFeatureDetector sift( .06, 10);

    printf( "\nFinding Features....\n\n");
    // Detect sift features and store them in the keypoints
    sift.detect( img1, keypoints1 );
    sift.detect( img2, keypoints2 );

    printf( "Features Found!\n\n");
    printf( "Each Potential match is drawn in Red. \n\n1) If it's an accurate correspondence "  
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
    matcher.match( descriptors1, descriptors2, matches );

    // Draw the matches on to the img_matches matrix
    Mat img_matches;
    drawMatches( img1, keypoints1, img2, keypoints2, matches, img_matches);

    // Put both images side by side in one image
	Mat rslt = joinFrames( img1, img2 );

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
        Point2f transpt1 = Point2f( pt1.x,window_height-pt1.y);
        Point2f transpt2 = Point2f( pt2.x+window_width,window_height-pt2.y);
        // Draw circles around the correspondences
        circle( tmp, pt1, 3, Scalar( 0,0,255), 1, CV_AA );
        circle( tmp, pt2+Point2f(window_width,0), 3, Scalar(0,0,255), 1, CV_AA );
        line( tmp, pt1, pt2+Point2f( (float)window_width,0.0), Scalar(0,0,255), 1, CV_AA );
        // Show this match and ask the user if it's an accurate correspondence
        imshow( "Matching Correspondences", tmp);
        char c = waitKey( 0);
        if( c == 'y' || c == 'Y' ) {
            // Store the correspondences in a vector of pairs, where the members of each pair are points
            corrs.push_back( match( Vec4f(pt1.x,pt1.y,1,1), Vec4f(pt2.x,pt2.y,1,1) ) );
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
            Point2f firstpt2D = Point2f(corrs[j].first[0], corrs[j].first[1]);
            Point2f secondpt2D = Point2f(corrs[j].second[0], corrs[j].second[1]);
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
            corrs.push_back( match( Vec4f(pt1.x,pt1.y,1,1), Vec4f(pt2.x,pt2.y,1,1) ) );
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

match calcCentroids( const matchList& corrs, const Mat& img1, const Mat& img2 ) {

    printf( "\n\nCalculating the centroids for each cloud of correspondences\n" );

    Vec4f centr1(0,0,0,0), centr2(0,0,0,0);

    for( int i = 0; i < (int)corrs.size(); i ++ ) {

        // First image 
        centr1[0] += corrs[i].first[0]; 
        centr1[1] += corrs[i].first[1]; 
        centr1[2] += corrs[i].first[2]; 

        // Second image
        centr2[0] += corrs[i].second[0];
        centr2[1] += corrs[i].second[1];
        centr2[2] += corrs[i].second[2]; 

    }

	// first
    centr1[0] /= corrs.size();
    centr1[1] /= corrs.size();
    centr1[2] /= corrs.size();
	// second
    centr2[0] /= corrs.size();
    centr2[1] /= corrs.size();
    centr2[2] /= corrs.size();

    match centroids( centr1, centr2 );

    printf( "\nCorrespondence points in Image 1\n" );
    for( int i = 0; i < (int)corrs.size(); i++ ) 
        printf( "%d: (%f,%f)\n", i, corrs[i].first[0], corrs[i].first[1] );
    printf( "\nCorrespondence points in Image 2\n" );
    for( int i = 0; i < (int)corrs.size(); i++ ) 
        printf( "%d: (%f,%f)\n\n", i, corrs[i].second[0], corrs[i].second[1] );

// Put both images side by side in one image
    Mat rslt = joinFrames( img1, img2 );

// Draw green circles around the correspondences in each image
    for( int i = 0; i < (int)corrs.size(); i++ ) {
        circle( rslt, Point2f( corrs[i].first[0], corrs[i].first[1] ), 3, Scalar(0,255,0), 1, CV_AA );
        circle( rslt, Point2f( corrs[i].second[0], corrs[i].second[1] )
				+Point2f(window_width,0), 3, Scalar(0,255,0), 1, CV_AA );
    }
        
// Draw red circles around the centroids
    circle( rslt, Point2f(centr1[0],centr1[1]), 3, Scalar(0,0,255), 1, CV_AA );
    circle( rslt, Point2f(centr2[0],centr2[1])+Point2f(window_width,0), 3, Scalar(0,0,255), 1, CV_AA );

    printf("Centroid 1 (x,y) = (%f,%f)\n", centr1[0], centr1[1]);
    printf("Centroid 2 (x,y) = (%f,%f)\n", centr2[0], centr2[1]);
    imshow("Centroids", rslt);

    waitKey(0);

	cvDestroyWindow("Centroids");
	cvDestroyWindow("SIFT Matches");
	cvDestroyWindow("Matching Correspondences");

    return centroids;

} 

void projectDepth( matchList& corrs ) {

    float fx = 594.21f;
    float fy = 591.04f;
    float a = -0.0030711f;
    float b = 3.3309495f;
    float cx = 339.5f;
    float cy = 242.7f;

	// required to find projected depth
	float mat[16] = { 1/fx,     0,  0, 0,
					  0,    -1/fy,  0, 0,
					  0,       0,  0, a,
					  -cx/fx, cy/fy, -1, b };

	Mat projectionMat( 4, 4, CV_32F, mat );
	Mat P1 = Mat::ones( 4, corrs.size(), CV_32F );
	Mat P2 = Mat::ones( 4, corrs.size(), CV_32F );

	//printf("Proj.type = %d\n", projectionMat.type());
	//printf("P1.type = %d\n", P1.type());
	
#if 1

	// This is retarded... Need to restructure
	for( int pt = 0; pt < corrs.size(); pt++ ) {
		// first point cloud
		float x = corrs[pt].first[0]; 
		float y = corrs[pt].first[1];
		float d = depthCV[0].at<float>( (int)x, (int)y );
		P1.at<float>(0,pt) = x; 
		P1.at<float>(1,pt) = y; 
		P1.at<float>(2,pt) = d; 
		// second point cloud
		float x2 = corrs[pt].second[0]; 
		float y2 = corrs[pt].second[1];
		float d2 = depthCV[1].at<float>( (int)x2, (int)y2 );
		P2.at<float>(0,pt) = x2; 
		P2.at<float>(1,pt) = y2; 
		P2.at<float>(2,pt) = d2; 
	}

	// Multiply the points by the 
	// projection matrix
	Mat newCorrs1 = projectionMat*P1;	
	Mat newCorrs2 = projectionMat*P2;	

	vector< pair< Vec4f, Vec4f > > actCoors;

	for( int pt = 0; pt < corrs.size(); pt++ ) {
		Vec4f point1( newCorrs1.col(pt) );
		Vec4f point2( newCorrs2.col(pt) );
		pair< Vec4f, Vec4f > cor( point1, point2 );
		actCoors.push_back( cor );
	}
	
#endif


}

#if 1
Mat calcRotation( matchList corrs ) {


	Mat P1 = Mat::ones( 4, corrs.size(), CV_32F );
	Mat P2 = Mat::ones( 4, corrs.size(), CV_32F );

	// This is retarded... Need to restructure
	for( int pt = 0; pt < corrs.size(); pt++ ) {
		// first point cloud
		float x = corrs[pt].first[0]; 
		float y = corrs[pt].first[1];
		float d = depthCV[0].at<float>( (int)x, (int)y );
		P1.at<float>(0,pt) = x; 
		P1.at<float>(1,pt) = y; 
		P1.at<float>(2,pt) = d; 
		// second point cloud
		float x2 = corrs[pt].second[0]; 
		float y2 = corrs[pt].second[1];
		float d2 = depthCV[1].at<float>( (int)x2, (int)y2 );
		P2.at<float>(0,pt) = x2; 
		P2.at<float>(1,pt) = y2; 
		P2.at<float>(2,pt) = d2; 
	}
	
    SVD P1svd( P1 );
    SVD P2svd( P2 );

	Mat rot1 = P1svd.u*P1svd.vt;
	Mat rot2 = P2svd.u*P2svd.vt;

	printf(" rot1.cols = %d\n", rot1.cols );
	printf(" rot2.cols = %d\n", rot2.cols );
	printf(" rot1.rows = %d\n", rot1.rows );
	printf(" rot2.rows = %d\n", rot2.rows );
	printf(" rot1.type = %d\n", rot1.type() );
	printf(" rot2.type = %d\n", rot2.type() );

	return rot1;
}
#endif 
