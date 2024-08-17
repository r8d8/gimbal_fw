General configuration for OpenCV 4.6.0 =====================================
  Version control:               unknown

  Extra modules:
    Location (extra):            <<PKGBUILDDIR>>/contrib/modules
    Version control (extra):     unknown

  Platform:
    Host:                         aarch64
    CMake:                       3.25.1
    CMake generator:             Ninja
    CMake build tool:            /usr/bin/ninja
    Configuration:               Release

  CPU/HW features:
    Baseline:                    NEON FP16

  C/C++:
    Built as dynamic libs?:      YES
    C++ standard:                11
    C++ Compiler:                /usr/bin/c++  (ver 12.2.0)
    C++ flags (Release):         -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2   -fsigned-char -W -Wall -Wreturn-type -Wnon-virtual-dtor -Waddress -Wsequence-point -Wformat -Wformat-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wuninitialized -Wsuggest-override -Wno-delete-non-virtual-dtor -Wno-comment -Wimplicit-fallthrough=3 -Wno-strict-overflow -fdiagnostics-show-option -pthread -fomit-frame-pointer -ffunction-sections -fdata-sections    -fvisibility=hidden -fvisibility-inlines-hidden -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security  -DNDEBUG
    C++ flags (Debug):           -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2   -fsigned-char -W -Wall -Wreturn-type -Wnon-virtual-dtor -Waddress -Wsequence-point -Wformat -Wformat-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wuninitialized -Wsuggest-override -Wno-delete-non-virtual-dtor -Wno-comment -Wimplicit-fallthrough=3 -Wno-strict-overflow -fdiagnostics-show-option -pthread -fomit-frame-pointer -ffunction-sections -fdata-sections    -fvisibility=hidden -fvisibility-inlines-hidden -g  -DDEBUG -D_DEBUG
    C Compiler:                  /usr/bin/cc
    C flags (Release):           -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2   -fsigned-char -W -Wall -Wreturn-type -Waddress -Wsequence-point -Wformat -Wformat-security -Wmissing-declarations -Wmissing-prototypes -Wstrict-prototypes -Wundef -Winit-self -Wpointer-arith -Wshadow -Wuninitialized -Wno-comment -Wimplicit-fallthrough=3 -Wno-strict-overflow -fdiagnostics-show-option -pthread -fomit-frame-pointer -ffunction-sections -fdata-sections    -fvisibility=hidden -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security  -DNDEBUG
    C flags (Debug):             -g -O2 -ffile-prefix-map=<<PKGBUILDDIR>>=. -fstack-protector-strong -Wformat -Werror=format-security -Wdate-time -D_FORTIFY_SOURCE=2   -fsigned-char -W -Wall -Wreturn-type -Waddress -Wsequence-point -Wformat -Wformat-security -Wmissing-declarations -Wmissing-prototypes -Wstrict-prototypes -Wundef -Winit-self -Wpointer-arith -Wshadow -Wuninitialized -Wno-comment -Wimplicit-fallthrough=3 -Wno-strict-overflow -fdiagnostics-show-option -pthread -fomit-frame-pointer -ffunction-sections -fdata-sections    -fvisibility=hidden -g  -DDEBUG -D_DEBUG
    Linker flags (Release):      -Wl,-z,relro -Wl,-z,now  -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined -Wl,-z,relro -Wl,-z,now 
    Linker flags (Debug):        -Wl,-z,relro -Wl,-z,now  -Wl,--gc-sections -Wl,--as-needed -Wl,--no-undefined  
    ccache:                      NO
    Precompiled headers:         NO
    Extra dependencies:          dl m pthread rt
    3rdparty dependencies:

  OpenCV modules:
    To be built:                 alphamat aruco barcode bgsegm bioinspired calib3d ccalib core cvv datasets dnn dnn_objdetect dnn_superres dpm face features2d flann freetype fuzzy hdf hfs highgui img_hash imgcodecs imgproc intensity_transform java line_descriptor mcc ml objdetect optflow phase_unwrapping photo plot quality rapid reg rgbd saliency shape stereo stitching structured_light superres surface_matching text tracking video videoio videostab viz wechat_qrcode ximgproc xobjdetect xphoto
    Disabled:                    python3 world
    Disabled by dependency:      sfm
    Unavailable:                 cudaarithm cudabgsegm cudacodec cudafeatures2d cudafilters cudaimgproc cudalegacy cudaobjdetect cudaoptflow cudastereo cudawarping cudev gapi julia matlab ovis python2 ts
    Applications:                apps
    Documentation:               doxygen python javadoc
    Non-free algorithms:         NO

  GUI:                           QT5
    QT:                          YES (ver 5.15.8 )
      QT OpenGL support:         YES (Qt5::OpenGL 5.15.8)
    OpenGL support:              YES (/usr/lib/aarch64-linux-gnu/libOpenGL.so /usr/lib/aarch64-linux-gnu/libGLX.so /usr/lib/aarch64-linux-gnu/libGLU.so)
    VTK support:                 YES (ver 9.1.0)

  Media I/O: 
    ZLib:                        /usr/lib/aarch64-linux-gnu/libz.so (ver 1.2.13)
    JPEG:                        /usr/lib/aarch64-linux-gnu/libjpeg.so (ver 62)
    WEBP:                        /usr/lib/aarch64-linux-gnu/libwebp.so (ver encoder: 0x020f)
    PNG:                         /usr/lib/aarch64-linux-gnu/libpng.so (ver 1.6.39)
    TIFF:                        /usr/lib/aarch64-linux-gnu/libtiff.so (ver 42 / 4.5.0)
    JPEG 2000:                   OpenJPEG (ver 2.5.0)
    OpenEXR:                     OpenEXR::OpenEXR (ver 3.1.5)
    GDAL:                        YES (/usr/lib/aarch64-linux-gnu/libgdal.so)
    GDCM:                        YES (3.0.21)
    HDR:                         YES
    SUNRASTER:                   YES
    PXM:                         YES
    PFM:                         YES

  Video I/O:
    DC1394:                      YES (2.2.6)
    FFMPEG:                      YES
      avcodec:                   YES (59.37.100)
      avformat:                  YES (59.27.100)
      avutil:                    YES (57.28.100)
      swscale:                   YES (6.7.100)
      avresample:                NO
    GStreamer:                   YES (1.22.0)
    PvAPI:                       NO
    v4l/v4l2:                    YES (linux/videodev2.h)
    gPhoto2:                     YES

  Parallel framework:            TBB (ver 2021.8 interface 12080)

  Trace:                         YES (built-in)

  Other third-party libraries:
    Lapack:                      YES (/usr/lib/aarch64-linux-gnu/liblapack.so /usr/lib/aarch64-linux-gnu/libblas.so)
    Eigen:                       YES (ver 3.4.0)
    Custom HAL:                  NO
    Protobuf:                    /usr/lib/aarch64-linux-gnu/libprotobuf.so (3.21.12)

  OpenCL:                        YES (no extra features)
    Include path:                /usr/include/CL
    Link libraries:              Dynamic load

  Python (for build):            /usr/bin/python3

  Java:                          
    ant:                         /usr/bin/ant (ver 1.10.13)
    JNI:                         /usr/lib/jvm/default-java/include /usr/lib/jvm/default-java/include/linux /usr/lib/jvm/default-java/include
    Java wrappers:               YES
    Java tests:                  NO

  Install to:                    /usr
