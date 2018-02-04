/** @file
    @brief Example program that uses the OSVR direct-to-display interface
           and OpenGL to render a scene with low latency.
    @date 2015
    @author
    Russ Taylor <russ@sensics.com>
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Internal Includes
#include <osvr/ClientKit/Context.h>
#include <osvr/ClientKit/Interface.h>
#include <osvr/RenderKit/RenderManager.h>
#include <osvr/RenderKit/RenderKitGraphicsTransforms.h>
#include <osvr/ClientKit/InterfaceStateC.h>
#include <osvr/ClientKit/Export.h>
#include <osvr/Util/APIBaseC.h>
#include <osvr/Util/ReturnCodesC.h>
#include <osvr/Util/AnnotationMacrosC.h>
#include <osvr/Util/ClientOpaqueTypesC.h>
#include <osvr/Util/ClientReportTypesC.h>
#include <osvr/Util/TimeValueC.h>
#include <vrpn_Shared.h>
#include <quat.h>
#include "font.h" // Simple helper functions to generate and draw OpenGL bitmapped text

// Library/third-party includes
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#ifdef __APPLE__
#include <OpenGL/gl.h>
#else
#include <GL/gl.h>
#endif
#include <opencv2/core/core.hpp> // for basic OpenCV types
#include <opencv2/core/operations.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/version.hpp>
#include <opencv2/videoio.hpp> // for image capture
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <vector>

// Standard includes
#include <iostream>
#include <string>
#include <stdlib.h> // For exit()

#include "ros/ros.h"
#include "std_msgs/String.h"

//Posiciones gafas
#include "asr_flir_ptu_driver/State.h"
#include <sensor_msgs/JointState.h>

// This must come after we include <GL/gl.h> so its pointer types are defined.
#include <osvr/RenderKit/GraphicsLibraryOpenGL.h>

double roll_global = 0;
double pitch_global = 0;
// Forward declarations of rendering functions defined below.
void show_image(cv::Mat image);

// Set to true when it is time for the application to quit.
// Handlers below that set it to true when the user causes
// any of a variety of events so that we shut down the system
// cleanly.  This only works on Windows, but so does D3D...
static bool quit = false;

#ifdef _WIN32
// Note: On Windows, this runs in a different thread from
// the main application.
static BOOL CtrlHandler(DWORD fdwCtrlType) {
    switch (fdwCtrlType) {
    // Handle the CTRL-C signal.
    case CTRL_C_EVENT:
    // CTRL-CLOSE: confirm that the user wants to exit.
    case CTRL_CLOSE_EVENT:
    case CTRL_BREAK_EVENT:
    case CTRL_LOGOFF_EVENT:
    case CTRL_SHUTDOWN_EVENT:
        quit = true;
        return TRUE;
    default:
        return FALSE;
    }
}
#endif

using namespace cv;

asr_flir_ptu_driver::State toEulerAngle(const OSVR_Quaternion &spinQuat){
//std_msgs::String toEulerAngle(const OSVR_Quaternion &spinQuat){
        double spinX = osvrQuatGetX(&spinQuat);
        double spinY = osvrQuatGetY(&spinQuat);
        double spinZ = osvrQuatGetZ(&spinQuat);
        double spinW = osvrQuatGetW(&spinQuat);
        //To Euler Angles
        double ysqr = spinY * spinY;
        double t0 = 2 * (spinW * spinX + spinY * spinZ);
        double t1 = 1 - 2 * (spinX *spinX + ysqr);
        double t2 = 2 * (spinW * spinY - spinZ * spinX);
        if(t2 > 1) t2 = 1;
        if(t2 < -1) t2 = -1;
        //double t3 = 2 * (spinW * spinZ + spinX * spinY);
        //double t4 = 1 - 2 * (ysqr + spinZ * spinZ);

        //NORMALIZAR ANGULOS
        double incremento_roll = roll_global - std::atan2(t0,t1);
        double incremento_pitch = pitch_global - std::asin(t2);

        roll_global = std::atan2(t0,t1);
        pitch_global = std::asin(t2);

        //double yaw = std::atan2(t3,t4);

        //No pintar siempre, para poder observar bien los cambios???
        
        /*std_msgs::String msg;
        std::string tmp(std::to_string(roll) + " " + std::to_string(pitch) + " " + std::to_string(yaw));
        //std::string tmp("prueba");
        msg.data = tmp;*/

        asr_flir_ptu_driver::State movement_goal;
        if(incremento_roll > 0.1 || incremento_pitch > 0.1){
            movement_goal.state.position.push_back(incremento_roll);
            movement_goal.state.position.push_back(incremento_pitch);
            movement_goal.state.velocity.push_back(1.0);
            movement_goal.state.velocity.push_back(1.0); 
        }
        
        return movement_goal;
        //return msg;
        
}

asr_flir_ptu_driver::State getOrientationState(osvr::clientkit::ClientContext &client,
                         osvr::clientkit::Interface     &head) {
//std_msgs::String getOrientationState(osvr::clientkit::ClientContext &client,
 //                        osvr::clientkit::Interface     &head) {
    // Note that there is not currently a tidy C++ wrapper for
    // state access, so we're using the C API call directly here.
    OSVR_OrientationState state;
    OSVR_TimeValue timestamp;
    OSVR_ReturnCode ret = osvrGetOrientationState(head.get(), &timestamp, &state);
    asr_flir_ptu_driver::State movement_goal;
    //std_msgs::String msg;
    //std::cout << ret << std::endl;
    if (OSVR_RETURN_SUCCESS != ret) {
        std::cout << "No pose state!" << std::endl;
    } else {
        movement_goal = toEulerAngle(state);
        //msg = toEulerAngle(state);
        //printQuaternion(state);
    }
    return movement_goal;
    //return msg;
}

// This callback sets a boolean value whose pointer is passed in to
// the state of the button that was pressed.  This lets the callback
// be used to handle any button press that just needs to update state.
void myButtonCallback(void* userdata, const OSVR_TimeValue* /*timestamp*/,
                      const OSVR_ButtonReport* report) {
    bool* result = static_cast<bool*>(userdata);
    *result = (report->state != 0);
}

bool SetupRendering(osvr::renderkit::GraphicsLibrary library) {
    // Make sure our pointers are filled in correctly.  The config file selects
    // the graphics library to use, and may not match our needs.
    if (library.OpenGL == nullptr) {
        std::cerr << "SetupRendering: No OpenGL GraphicsLibrary, this should "
                     "not happen"
                  << std::endl;
        return false;
    }

    osvr::renderkit::GraphicsLibraryOpenGL* glLibrary = library.OpenGL;

    // Turn on depth testing, so we get correct ordering.
    glEnable(GL_DEPTH_TEST);

    return true;
}

// Callback to set up a given display, which may have one or more eyes in it
void SetupDisplay(
    void* userData //< Passed into SetDisplayCallback
    , osvr::renderkit::GraphicsLibrary library //< Graphics library context to use
    , osvr::renderkit::RenderBuffer buffers //< Buffers to use
    ) {
    // Make sure our pointers are filled in correctly.  The config file selects
    // the graphics library to use, and may not match our needs.
    if (library.OpenGL == nullptr) {
        std::cerr
            << "SetupDisplay: No OpenGL GraphicsLibrary, this should not happen"
            << std::endl;
        return;
    }
    if (buffers.OpenGL == nullptr) {
        std::cerr
            << "SetupDisplay: No OpenGL RenderBuffer, this should not happen"
            << std::endl;
        return;
    }

    osvr::renderkit::GraphicsLibraryOpenGL* glLibrary = library.OpenGL;

    // Clear the screen to black and clear depth
    glClearColor(0, 0, 0, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
}

// Callback to set up for rendering into a given eye (viewpoint and projection).
void SetupEye(
    void* userData //< Passed into SetViewProjectionCallback
    , osvr::renderkit::GraphicsLibrary library //< Graphics library context to use
    , osvr::renderkit::RenderBuffer buffers //< Buffers to use
    , osvr::renderkit::OSVR_ViewportDescription
        viewport //< Viewport set by RenderManager
    , osvr::renderkit::OSVR_ProjectionMatrix
        projectionToUse //< Projection matrix set by RenderManager
    , size_t whichEye //< Which eye are we setting up for?
    ) {
    // Make sure our pointers are filled in correctly.  The config file selects
    // the graphics library to use, and may not match our needs.
    if (library.OpenGL == nullptr) {
        std::cerr
            << "SetupEye: No OpenGL GraphicsLibrary, this should not happen"
            << std::endl;
        return;
    }
    if (buffers.OpenGL == nullptr) {
        std::cerr << "SetupEye: No OpenGL RenderBuffer, this should not happen"
                  << std::endl;
        return;
    }

    // Set the viewport
    glViewport(static_cast<GLint>(viewport.left),
      static_cast<GLint>(viewport.lower),
      static_cast<GLint>(viewport.width),
      static_cast<GLint>(viewport.height));

    // Set the OpenGL projection matrix based on the one we
    // received.
    GLdouble projection[16];
    OSVR_Projection_to_OpenGL(projection,
      projectionToUse);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMultMatrixd(projection);

    // Set the matrix mode to ModelView, so render code doesn't mess with
    // the projection matrix on accident.
    glMatrixMode(GL_MODELVIEW);
}

// Callbacks to draw things in world space, left-hand space, and right-hand
// space.
void DrawWorld(
    void* userData //< Passed into AddRenderCallback
    , osvr::renderkit::GraphicsLibrary library //< Graphics library context to use
    , osvr::renderkit::RenderBuffer buffers //< Buffers to use
    , osvr::renderkit::OSVR_ViewportDescription
        viewport //< Viewport we're rendering into
    , OSVR_PoseState pose //< OSVR ModelView matrix set by RenderManager
    , osvr::renderkit::OSVR_ProjectionMatrix
        projection //< Projection matrix set by RenderManager
    , OSVR_TimeValue deadline //< When the frame should be sent to the screen
    ) {
    // Make sure our pointers are filled in correctly.  The config file selects
    // the graphics library to use, and may not match our needs.
    if (library.OpenGL == nullptr) {
        std::cerr
            << "DrawWorld: No OpenGL GraphicsLibrary, this should not happen"
            << std::endl;
        return;
    }
    if (buffers.OpenGL == nullptr) {
        std::cerr << "DrawWorld: No OpenGL RenderBuffer, this should not happen"
                  << std::endl;
        return;
    }

    osvr::renderkit::GraphicsLibraryOpenGL* glLibrary = library.OpenGL;

    /// Put the transform into the OpenGL ModelView matrix
    GLdouble modelView[16];
    osvr::renderkit::OSVR_PoseState_to_OpenGL(modelView, pose);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glMultMatrixd(modelView);

    cv::VideoCapture vcap;
    cv::Mat image;
    HOGDescriptor hog;
    std::vector<Rect> rects;
    const std::string videoStreamAddress = "http://10.5.5.9:8080/live/amba.m3u8"; 
   
    if(!vcap.open(videoStreamAddress)) {
        std::cout << "Error opening video stream or file" << std::endl;
    }
   
    if(!vcap.read(image)) {
        std::cout << "No frame" << std::endl;
        cv::waitKey();
    }

    //Aplicar detector de personas

    hog.setSVMDetector( HOGDescriptor::getDefaultPeopleDetector() );

        
    copyMakeBorder( image,image,50,50,50,50,0);
    hog.detectMultiScale(image, rects, 0, Size(8,8), Size(32,32), 1.05, 2);

    size_t i;

    for (i=0; i<rects.size(); i++)
    {
        rectangle( image, rects[i], cv::Scalar(0,255,0), 2);
    }
    /// Draw a cube with a 5-meter radius as the room we are floating in.
    show_image(image);
}

void Usage(std::string name) {
    std::cerr << "Usage: " << name << " spinRateRadiansPerSecond" << std::endl;
    exit(-1);
}

int main(int argc, char* argv[]) {
    // Parse the command line
    double spinRateRadiansPerSecond = 0.5;
    int realParams = 0;
    for (int i = 1; i < argc; i++) {
#if 0
      if (argv[i][0] == '-') {
        Usage(argv[0]);
      }
      else
#endif
        switch (++realParams) {
        case 1:
            spinRateRadiansPerSecond = atof(argv[i]);
            break;
        default:
            Usage(argv[0]);
        }
    }
    if (realParams > 1) {
        Usage(argv[0]);
    }

    // Get an OSVR client context to use to access the devices
    // that we need.
    osvr::clientkit::ClientContext context(
        "com.osvr.renderManager.openGLExample");

    osvr::renderkit::RenderManager* render =
        osvr::renderkit::createRenderManager(context.get(), "OpenGL");

    if ((render == nullptr) || (!render->doingOkay())) {
        std::cerr << "Could not create RenderManager" << std::endl;
        return 1; 
    }

    osvr::clientkit::Interface head = context.getInterface("/me/head");

    // Set callback to handle setting up rendering in a display
    render->SetDisplayCallback(SetupDisplay);

    // Set callback to handle setting up rendering in an eye
    render->SetViewProjectionCallback(SetupEye);

    // Keeps track of the frame index as a string that should be
    // printed into the window.
    std::string frameStringToPrint;

    // Register callbacks to render things in left hand, right
    // hand, and world space.
    render->AddRenderCallback("/", DrawWorld);

// Set up a handler to cause us to exit cleanly.
#ifdef _WIN32
    SetConsoleCtrlHandler((PHANDLER_ROUTINE)CtrlHandler, TRUE);
#endif

    // Open the display and make sure this worked.
    osvr::renderkit::RenderManager::OpenResults ret = render->OpenDisplay();
    if (ret.status == osvr::renderkit::RenderManager::OpenStatus::FAILURE) {
        std::cerr << "Could not open display" << std::endl;
        delete render;
        return 2;
    }

    // Set up the rendering state we need.
    if (!SetupRendering(ret.library)) {
        return 3;
    }

    // Keep track of our frame index, incrementing every time we render.
    size_t frameIndex = 0;

#define DEBUG_FRAME_TIMING
#ifdef DEBUG_FRAME_TIMING
    // Fine-grained timing of frame update rate.  This is a specialized
    // debugging tool to help with the case where we're supposed to be
    // rendering at exactly 60fps and sometimes we skip frames on some
    // particular HMD devices.
    struct timeval lastRenderTime;
    vrpn_gettimeofday(&lastRenderTime, nullptr);
#endif


    ros::init(argc,argv, "Rotation_publisher");
    ros::NodeHandle n;
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("Rotacion", 1000);
    ros::Publisher rotation_pub = n.advertise<asr_flir_ptu_driver::State>("Rotacion", 1);
    // Continue rendering until it is time to quit.
    struct timeval start;
    vrpn_gettimeofday(&start, nullptr);
    while (!quit) {
        // Update the context so we get our callbacks called and
        // update tracker state.
        context.update();

        // Keep our frame-index message up to date.
        frameStringToPrint = std::to_string(frameIndex++);

        // Figure out how much to spin the world based on time since we
        // started. Then adjust the room-to-world rotation to match.
        // NOTE: This is better debugging demo than a stand-inside demo;
        // it will tend you make you uncomfortable.
        struct timeval now;
        vrpn_gettimeofday(&now, nullptr);
        double rads =
            spinRateRadiansPerSecond * vrpn_TimevalDurationSeconds(now, start);
        q_type spinQ;
        q_from_axis_angle(spinQ, 0, 0, 0, rads);
        OSVR_Quaternion spinQuat;
        osvrQuatSetX(&spinQuat, spinQ[Q_X]);
        osvrQuatSetY(&spinQuat, spinQ[Q_Y]);
        osvrQuatSetZ(&spinQuat, spinQ[Q_Z]);
        osvrQuatSetW(&spinQuat, spinQ[Q_W]);
        OSVR_PoseState spin;
        spin.translation = {};
        spin.rotation = spinQuat;
        osvr::renderkit::RenderManager::RenderParams params;
        params.worldFromRoomAppend = &spin;

        asr_flir_ptu_driver::State movement_goal = getOrientationState(context, head);
        //std_msgs::String msg = getOrientationState(context, head);

        //ROS_INFO("%s", finalMsg.data.c_str());
        if(!movement_goal.state.position.empty()){
            rotation_pub.publish(movement_goal);
        }
        
        //chatter_pub.publish(msg);

        // Render, spinning the world by the specified amount
        if (!render->Render(params)) {
            std::cerr
                << "Render() returned false, maybe because it was asked to quit"
                << std::endl;
            quit = true;
        }

#ifdef DEBUG_FRAME_TIMING
        struct timeval thisRenderTime;
        vrpn_gettimeofday(&thisRenderTime, nullptr);
        double renderTime =
            vrpn_TimevalDurationSeconds(thisRenderTime, lastRenderTime);
        if ((renderTime < 10e-3) || (renderTime > 20e-3)) {
            std::cerr << "Frame " << frameIndex << ", expected render time of "
                      << 1.0e3 / 60 << "ms, got " << renderTime * 1e3
                      << std::endl;
        }
        lastRenderTime = thisRenderTime;
#endif
    }

    // Close the Renderer interface cleanly.
    delete render;
    return 0;
}

GLuint matToTexture(cv::Mat &mat, GLenum minFilter, GLenum magFilter, GLenum wrapFilter)
{
    // Generate a number for our textureID's unique handle
    GLuint textureID;
    glGenTextures(1, &textureID);
 
    // Bind to our texture handle
    glBindTexture(GL_TEXTURE_2D, textureID);
 
    // Catch silly-mistake texture interpolation method for magnification
    if (magFilter == GL_LINEAR_MIPMAP_LINEAR  ||
        magFilter == GL_LINEAR_MIPMAP_NEAREST ||
        magFilter == GL_NEAREST_MIPMAP_LINEAR ||
        magFilter == GL_NEAREST_MIPMAP_NEAREST)
    {
        std::cout << "You can't use MIPMAPs for magnification - setting filter to GL_LINEAR" << std::endl;
        magFilter = GL_LINEAR;
    }
 
    // Set texture interpolation methods for minification and magnification
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, minFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, magFilter);
 
    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, wrapFilter);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, wrapFilter);



 
    // Set incoming texture format to:
    // GL_BGR       for CV_CAP_OPENNI_BGR_IMAGE,
    // GL_LUMINANCE for CV_CAP_OPENNI_DISPARITY_MAP,
    // Work out other mappings as required ( there's a list in comments in main() )
    GLenum inputColourFormat = GL_BGR;
    if (mat.channels() == 1)
    {
        inputColourFormat = GL_LUMINANCE;
    }
    // Create the texture
    glTexImage2D(GL_TEXTURE_2D,     // Type of texture
                 0,                 // Pyramid level (for mip-mapping) - 0 is the top level
                 GL_RGB,            // Internal colour format to convert to
                 mat.cols,          // Image width  i.e. 640 for Kinect in standard mode
                 mat.rows,          // Image height i.e. 480 for Kinect in standard mode
                 0,                 // Border width in pixels (can either be 1 or 0)
                 inputColourFormat, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
                 GL_UNSIGNED_BYTE,  // Image data type
                 mat.ptr());        // The actual image data itself
 
    return textureID;
}

void show_image(cv::Mat image){

    double offsetX = 0.15;
    double offsetY = 0.15;
    double parteLejos = -1.5;
    double parteCerca = -1.0;
    GLuint tex = matToTexture(image, GL_NEAREST, GL_NEAREST, GL_CLAMP);
    glEnable(GL_TEXTURE_2D);
    //glActiveTexture(GL_TEXTURE_2D);
    
    glPushMatrix();
    
    /**
    * Prueba cargar la imagen en la cara frontal del cubo
    */
    glBindTexture(GL_TEXTURE_2D, tex);

    /**
    * Prueba cargar la imagen en las 5 caras excepto la trasera
    */

    /** CARA DE DELANTE NO SE USA*/

    //Cara trasera

   /*glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, -1.0);
        
        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, -1.0);
    
        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, -1.0);
   
        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, -1.0);

    glEnd();

    
    //Izquierda

    glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, 1.0);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, -1.0);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, -1.0);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, 1.0);
    glEnd();

    //Derecha

    glBegin(GL_POLYGON);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, -1.0);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, 1.0);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, -1.0);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, 1.0);
    glEnd();


    //Arriba

    glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, 1.0);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, 1.0);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, -1.0);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, -1.0);
    glEnd();

    //Abajo

    glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, -1.0);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, -1.0);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, 1.0);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, 1.0);
    glEnd();*/


    /** CARA DE ATRAS NO SE USA(MUCHO MEJOR)**/
    //Cara delante

   /* glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, 1.0);
        
        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, 1.0);
    
        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, 1.0);
   
        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, 1.0);

    glEnd();

    
    //Izquierda

   glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, -1.0);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, 1.0);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, 1.0);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, -1.0);
    glEnd();

    //Derecha

    glBegin(GL_POLYGON);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, 1.0);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, -1.0);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, 1.0);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, -1.0);
    glEnd();


    //Arriba

    glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, -1.0);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, -1.0);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, 1.0);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, 1.0);
    glEnd();

    //Abajo

    glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, 1.0);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, 1.0);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, -1.0);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, -1.0);
    glEnd();*/

    /** CARA DE DELANTE NO SE USA PRUEBA PARA VERLOS DE MAS LEJOS (EL QUE MEJOR VA)*/

    //Cara trasera

   glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, parteLejos);
        
        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, parteLejos);
    
        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, parteLejos);
   
        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, parteLejos);

    glEnd();

    
    //Izquierda

    glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, parteCerca);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, parteLejos);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, parteLejos);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, parteCerca);
    glEnd();

    //Derecha

    glBegin(GL_POLYGON);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, parteLejos);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, parteCerca);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, parteLejos);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, parteCerca);
    glEnd();


    //Arriba

    glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, parteCerca);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, parteCerca);

        glTexCoord2f(offsetX,offsetY);
        glVertex3f(-1.0, 1.0, parteLejos);

        glTexCoord2f(1-offsetX,offsetY);
        glVertex3f(1.0, 1.0, parteLejos);
    glEnd();

    //Abajo

    glBegin(GL_POLYGON);

        glTexCoord2f(offsetX,1-offsetY);
        glVertex3f(-1.0, -1.0, parteLejos);

        glTexCoord2f(1-offsetX,1-offsetY);
        glVertex3f(1.0, -1.0, parteLejos);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, parteCerca);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, parteCerca);
    glEnd();

    /**SOLO UNA CARA DE DELANTE (NO VA BIEN) hay que definir las demas caras**/

    /*glBegin(GL_POLYGON);

        glTexCoord2f(0.0,0.0);
        glVertex3f(-1.0, 1.0, 1.0);

        glTexCoord2f(1.0,0.0);
        glVertex3f(1.0, 1.0, 1.0);

        glTexCoord2f(0.0,1.0);
        glVertex3f(-1.0, -1.0, 1.0);

        glTexCoord2f(1.0,1.0);
        glVertex3f(1.0, -1.0, 1.0);
    glEnd();*/



    glPopMatrix();
    glDeleteTextures(1, &tex);
    glDisable(GL_TEXTURE_2D);
}