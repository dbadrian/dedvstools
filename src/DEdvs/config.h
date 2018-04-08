
#ifndef DEDVS_CONFIG_H
#define DEDVS_CONFIG_H

#include <cstdint>
#include <cmath>

#include "dedvs_types.h"
#include "dedvs_enums.h"

namespace dedvs {

  /*  ### Depth: Supported Video Modes ### 
  
    * Video Mode | FPS: 30 | RES (X/Y): (320/240) | Pixelformat: 100 (PIXEL_FORMAT_DEPTH_1_MM)
    * Video Mode | FPS: 60 | RES (X/Y): (320/240) | Pixelformat: 100 (PIXEL_FORMAT_DEPTH_1_MM)
    * Video Mode | FPS: 30 | RES (X/Y): (640/480) | Pixelformat: 100 (PIXEL_FORMAT_DEPTH_1_MM)
    * Video Mode | FPS: 25 | RES (X/Y): (320/240) | Pixelformat: 100 (PIXEL_FORMAT_DEPTH_1_MM)
    * Video Mode | FPS: 25 | RES (X/Y): (640/480) | Pixelformat: 100 (PIXEL_FORMAT_DEPTH_1_MM)

    ! There are more Pixelformats available, but those are not supported within DEdvs !
          USE AT OWN RISK (See OpenNI2 Documentation for more)
  */
  constexpr int kXtionDepthResolutionX  = 320;
  constexpr int kXtionDepthResolutionY  = 240;
  constexpr int kXtionDepthFPS          = 30;

  // constexpr int kXtionDepthResolutionX  = 640;
  // constexpr int kXtionDepthResolutionY  = 480;
  // constexpr int kXtionDepthFPS          = 30;

  /*  ### Color: Supported Video Modes ### 
  
    Video Mode | FPS: 30 | RES (X/Y): (320/240)   | Pixelformat: 200 (PIXEL_FORMAT_RGB888)
    Video Mode | FPS: 60 | RES (X/Y): (320/240)   | Pixelformat: 200 (PIXEL_FORMAT_RGB888)
    Video Mode | FPS: 30 | RES (X/Y): (640/480)   | Pixelformat: 200 (PIXEL_FORMAT_RGB888)
    Video Mode | FPS: 25 | RES (X/Y): (320/240)   | Pixelformat: 200 (PIXEL_FORMAT_RGB888)
    Video Mode | FPS: 25 | RES (X/Y): (640/480)   | Pixelformat: 200 (PIXEL_FORMAT_RGB888)

    ! There are more Pixelformats available, but those are not supported within DEdvs !
          USE AT OWN RISK (See OpenNI2 Documentation for more)
  */
  constexpr int kXtionColorResolutionX  = 320;
  constexpr int kXtionColorResolutionY  = 240;
  constexpr int kXtionColorFPS          = 30;

  // constexpr int kXtionColorResolutionX  = 640;
  // constexpr int kXtionColorResolutionY  = 480;
  // constexpr int kXtionColorFPS          = 30;
  
  /* As the depth and color sensors are spatially divided, the images shown are 
     taken from differentperspectives. Set this to true, to enable hardware 
     image registration to correlate the images */
  constexpr bool kXtionImageRegistration = true;

  /* Set this to true, to ensure that the current color/depth frame are synced */
  constexpr bool kXtionFramesync = false;

  /* Set this to true, to activate the mirroring of both xtion streams */
  constexpr bool kXtionMirroring = false; 

  /* ATM: Only 128x128 available */
  constexpr int kEdvsResolution   = 128;
  constexpr int kEdvsResolution_U = kEdvsResolution;
  constexpr int kEdvsResolution_V = kEdvsResolution;

  constexpr bool kEdvsMirrorX = true;
  constexpr bool kEdvsMirrorY = true;
  constexpr bool kEdvsFlipXY  = false;

  //CAMERA PARAMETERS
  constexpr double kKinectFocalPX = 520.0;

  // constexpr double kEDVSFocalPX = 152.647;
  // constexpr double kEDVSKappa1 = 0.00190703;
  // constexpr double kEDVSKappa2 = 3.23689e-12;// * static_cast<double>(pow(10,-13));

  // constexpr double kR11 =  0.999769;
  // constexpr double kR12 = -0.0214026;
  // constexpr double kR13 = -0.00215436;
  // constexpr double kR21 =  0.021341;
  // constexpr double kR22 =  0.99945;
  // constexpr double kR23 =  -0.0253858;
  // constexpr double kR31 =  0.00269649;
  // constexpr double kR32 =  0.025334;
  // constexpr double kR33 =  0.999675;

  // constexpr double kT1 = 0.00311304;
  // constexpr double kT2 = 0.0253035;
  // constexpr double kT3 = -0.00661515;

  constexpr double kEDVSFocalPX = 241.168;
  constexpr double kEDVSKappa1 = 0.00126144;
  constexpr double kEDVSKappa2 = 5.86338e-13;// * static_cast<double>(pow(10,-13));

  constexpr double kR11 =  0.0250249;
  constexpr double kR12 = -0.999602;
  constexpr double kR13 =  0.0130048;
  constexpr double kR21 =  0.999567;
  constexpr double kR22 =  0.0252209;
  constexpr double kR23 =  0.0151377;
  constexpr double kR31 = -0.0154597;
  constexpr double kR32 =  0.0126203;
  constexpr double kR33 =  0.999801;

  constexpr double kT1 =  0.0019529;
  constexpr double kT2 =  0.00845518;
  constexpr double kT3 = -0.0528266;

  /*************************************************************************/
  /* DO NOT MODIFY BEYOND THIS POINT - UNLESS YOU KNOW WHAT YOU ARE DOING! */
  /*************************************************************************/

  /* URI and baudrate with which we open the EDVS stream */
  constexpr char kEdvsURI[] = "/dev/ttyUSB0?baudrate=4000000";

  /*Pixelformat to be used to set up the video stream from xtion */
  constexpr int kXtionDepthPixelFormat  = 100;
  constexpr int kXtionColorPixelFormat  = 200;

  /* What range of depth shall be processed (officially 800-3500 supported) */
  constexpr int kMinDepth = 500;
  constexpr int kMaxDepth = 3500;

  /* Resolution of a lookup table to match (10 = 10mm = 1cm) */
  constexpr int kDepthLUTResolution   = 3;
  constexpr int kDepthLUTEntries   = (kMaxDepth-kMinDepth)/kDepthLUTResolution; /* DO NOT CHANGE! */

  /* Frequency Filtering of Events (RecordCalibData) */
  constexpr int kFreqFilterDT = 1920;
  constexpr int kFreqFilterTolerance = 300;

  /* Size of one row of the depth frame. We don't offer the same for color, as it is more flexible
     depending on the RGB or YUV etc. */
  constexpr size_t kDepthSizeOfFrame = static_cast<size_t>(sizeof(uint16_t)*kXtionDepthResolutionX*kXtionDepthResolutionY);

  /* Find Nearest Depth Edge */
  constexpr int kSearchRange = 1; //in px
  constexpr int kThreshold = 40; //in mm
}

#endif