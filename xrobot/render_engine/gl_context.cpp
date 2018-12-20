#include <cassert>
#include <iostream>
#include <vector>

#include "gl_context.h"

using namespace std;

namespace xrobot{
namespace render_engine {

const EGLint EGLconfigAttribs[] = {
    EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,
    EGL_BLUE_SIZE, 8,
    EGL_GREEN_SIZE, 8,
    EGL_RED_SIZE, 8,
    EGL_DEPTH_SIZE, 8,
    EGL_RENDERABLE_TYPE, EGL_OPENGL_BIT,
    EGL_NONE
};

EGLint EGLpbufferAttribs[] = {
    EGL_WIDTH, 9,
    EGL_HEIGHT, 9,
    EGL_NONE,
};

const EGLint ctxattr[] = {
    EGL_CONTEXT_MAJOR_VERSION, 3,
    EGL_CONTEXT_MINOR_VERSION, 3,
    EGL_CONTEXT_OPENGL_PROFILE_MASK_KHR,
    EGL_CONTEXT_OPENGL_CORE_PROFILE_BIT_KHR,
    EGL_NONE
};

const int GLXcontextAttribs[] = {
    GLX_CONTEXT_MAJOR_VERSION_ARB, 3,
    GLX_CONTEXT_MINOR_VERSION_ARB, 3,
    //GLX_CONTEXT_FLAGS_ARB, GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
    None
};


const int GLXpbufferAttribs[] = {
    GLX_PBUFFER_WIDTH,  9,
    GLX_PBUFFER_HEIGHT, 9,
    None
};


void GLContext::PrintInfo() {
    assert(glGetString(GL_VERSION));
    cerr << "----------- OpenGL Context Info --------------" << endl;
    cerr << "GL Version: " << glGetString(GL_VERSION) << endl;
    cerr << "GLSL Version: " << glGetString(GL_SHADING_LANGUAGE_VERSION) << endl;
    cerr << "Vendor: " << glGetString(GL_VENDOR) << endl;
    cerr << "Renderer: " << glGetString(GL_RENDERER) << endl;
    cerr << "----------------------------------------------" << endl;
}

// https://devblogs.nvidia.com/parallelforall/egl-eye-opengl-visualization-without-x-server/
EGLContext::EGLContext(int h, int w, int device): GLContext{h, w, device} {
    auto checkError = [](EGLBoolean succ) {
        EGLint err = eglGetError();
        if (err != EGL_SUCCESS) {
            fprintf(stderr,"EGL error: %d\n", err);
            fflush(stderr);
            exit(1);
        }
        if (!succ) {
            fprintf(stderr,"EGL failed\n");
            fflush(stderr);
            exit(1);
        }
    };

    // 1. Initialize EGL
    {
        static const int MAX_DEVICES = 8;
        EGLDeviceEXT eglDevs[MAX_DEVICES];
        EGLint numDevices;
        PFNEGLQUERYDEVICESEXTPROC eglQueryDevicesEXT =
          (PFNEGLQUERYDEVICESEXTPROC) eglGetProcAddress("eglQueryDevicesEXT");
        PFNEGLGETPLATFORMDISPLAYEXTPROC eglGetPlatformDisplayEXT =
          (PFNEGLGETPLATFORMDISPLAYEXTPROC) eglGetProcAddress("eglGetPlatformDisplayEXT");
        if (!eglQueryDevicesEXT or !eglGetPlatformDisplayEXT) {
            fprintf(stderr, "Failed to get function pointer of "
                            "eglQueryDevicesEXT/eglGetPlatformDisplayEXT! "
                            "Maybe EGL extensions are unsupported.");
            fflush(stderr);
            exit(1);
        }

        eglQueryDevicesEXT(MAX_DEVICES, eglDevs, &numDevices);
        vector<int> device_ids;
        cuda_visible_devices(device_ids);
        if (device_ids.size() == 0) {
            for (int i = 0; i < numDevices; ++i) {
                device_ids.push_back(i);
            }
        }
        cout << "[EGL] Detected " << numDevices;
        assert(device < (int)device_ids.size());
        device = device_ids[device];
        cout << ", using device " << device << endl;
        eglDpy_ = eglGetPlatformDisplayEXT(
                EGL_PLATFORM_DEVICE_EXT, eglDevs[device], 0);
    }

    EGLint major, minor;
    EGLBoolean succ = eglInitialize(eglDpy_, &major, &minor);

    if (!succ) {
        fprintf(stderr, "Failed to initialize EGL display!");
        fflush(stderr);
        exit(1);
    }
    checkError(succ);

    // 2. Select an appropriate configuration
    EGLint numConfigs;
    EGLConfig eglCfg;

    succ = eglChooseConfig(eglDpy_, EGLconfigAttribs, &eglCfg, 1, &numConfigs);
    checkError(succ);
    if (numConfigs != 1) {
        fprintf(stderr, "Cannot create configs for EGL! "
                        "You driver may not support EGL.");
        fflush(stderr);
        exit(1);
    }

    // 3. Create a surface
    EGLSurface eglSurf =
            eglCreatePbufferSurface(eglDpy_, eglCfg, EGLpbufferAttribs);
    checkError(succ);
    
    // 4. Bind the API
    succ = eglBindAPI(EGL_OPENGL_API);
    checkError(succ);

    // 5. Create a context and make it current
    ::EGLContext eglCtx = eglCreateContext(
            eglDpy_, eglCfg, (::EGLContext)0, ctxattr);

    checkError(succ);
    succ = eglMakeCurrent(eglDpy_, eglSurf, eglSurf, eglCtx);
    if (!succ) {
        fprintf(stderr, "Failed to make EGL context current!");
        fflush(stderr);
        exit(1);
    }
    checkError(succ);

    // Debug
    // int NumberOfExtensions;
    // glGetIntegerv(GL_NUM_EXTENSIONS, &NumberOfExtensions);

    // printf("--------------------------------------------\n");
    // printf("NumberOfExtensions: %d\n", NumberOfExtensions);

    // for(int i=0; i<NumberOfExtensions; i++) {
    //     const GLubyte *ccc=glGetStringi(GL_EXTENSIONS, i);
    //     printf("  %s\n", ccc);
    // }

    this->Init();
}

EGLContext::~EGLContext() {
    // 6. Terminate EGL when finished
    eglTerminate(eglDpy_);
}


GLXHeadlessContext::GLXHeadlessContext(int h, int w): GLContext{h, w} {
    dpy_ = XOpenDisplay(NULL);
    if (dpy_ == nullptr) {
        fprintf(stderr, "Failed to connect display");
        fflush(stderr);
        exit(1);
    }
    static int visualAttribs[] = { None };
    int numberOfFramebufferConfigurations = 0;
    GLXFBConfig* fbc = glXChooseFBConfig(dpy_,
                                         DefaultScreen(dpy_),
                                         visualAttribs,
                                         &numberOfFramebufferConfigurations);
    
    // printf("num of conf: %d\n", numberOfFramebufferConfigurations);


    // std::cout << "Getting best XVisualInfo\n";
    int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;
    for (int i = 0; i < numberOfFramebufferConfigurations; ++i) {
        XVisualInfo *vi = glXGetVisualFromFBConfig( dpy_, fbc[i] );
        if ( vi != 0) {
            int samp_buf, samples;
            glXGetFBConfigAttrib( dpy_, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
            glXGetFBConfigAttrib( dpy_, fbc[i], GLX_SAMPLES       , &samples  );
            
            if ( best_fbc < 0 || (samp_buf && samples > best_num_samp) ) {
                best_fbc = i;
                best_num_samp = samples;
            }
            if ( worst_fbc < 0 || !samp_buf || samples < worst_num_samp ) {
                worst_fbc = i;
            }
            worst_num_samp = samples;
        }
        XFree( vi );
    }
    // std::cout << "Best visual info index: " << best_fbc << "\n";
    GLXFBConfig bestFbc = fbc[ best_fbc ]; 

    // setup function pointers
    typedef GLXContext (*glXCreateContextAttribsARBProc)
                       (Display*, GLXFBConfig, GLXContext, Bool, const int*);
    static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = NULL;
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc) 
            glXGetProcAddressARB((const GLubyte *)"glXCreateContextAttribsARB");

    // printf("pass glx context\n");

    int context_attribs[] =
    {
        GLX_CONTEXT_MAJOR_VERSION_ARB, 4,
        GLX_CONTEXT_MINOR_VERSION_ARB, 3,
        GLX_CONTEXT_PROFILE_MASK_ARB, GLX_CONTEXT_CORE_PROFILE_BIT_ARB,
        //GLX_CONTEXT_FLAGS_ARB        , GLX_CONTEXT_FORWARD_COMPATIBLE_BIT_ARB,
        None
    };

    GLXContext openGLContext = glXCreateContextAttribsARB(
            dpy_, bestFbc, 0, True, context_attribs);
    
    // printf("pass gl context\n");

    GLXPbuffer pbuffer = glXCreatePbuffer(dpy_, bestFbc, GLXpbufferAttribs);
    
    // printf("pass pbuffer\n");

    XFree(fbc);
    XSync(dpy_, False);
    if (!glXMakeContextCurrent(dpy_, pbuffer, pbuffer, openGLContext)) {
        fprintf(stderr, "Failed to make GLX context current!");
        fflush(stderr);
        exit(1);
    }
    
    printf("[GLX] Headless mode\n");

    // printf("pass glx make current\n");
    this->Init();
}

GLXHeadlessContext::~GLXHeadlessContext() {
    XCloseDisplay(dpy_);
}

void GLFWContext::Terminate() {
    glfwTerminate();
}

void GLFWContext::SwapBuffer() {
    glfwSwapBuffers(window_);
}

void GLFWContext::PollEvent() {
    glfwPollEvents();
}

void GLFWContext::SetWindowShouldClose() {
    glfwSetWindowShouldClose(window_, true);
}

bool GLFWContext::GetWindowShouldClose() {
    return glfwWindowShouldClose(window_);
}

bool GLFWContext::GetKeyPress0()
{
    return glfwGetKey(window_, GLFW_KEY_0) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress9()
{
    return glfwGetKey(window_, GLFW_KEY_9) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress7()
{
    return glfwGetKey(window_, GLFW_KEY_7) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress4()
{
    return glfwGetKey(window_, GLFW_KEY_4) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress6()
{
    return glfwGetKey(window_, GLFW_KEY_6) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress8()
{
    return glfwGetKey(window_, GLFW_KEY_8) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress2()
{
    return glfwGetKey(window_, GLFW_KEY_2) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress1()
{
    return glfwGetKey(window_, GLFW_KEY_1) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress3()
{
    return glfwGetKey(window_, GLFW_KEY_3) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPress5()
{
    return glfwGetKey(window_, GLFW_KEY_5) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressUp()
{
    return glfwGetKey(window_, GLFW_KEY_UP) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressDown()
{
    return glfwGetKey(window_, GLFW_KEY_DOWN) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressLeft()
{
    return glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressRight()
{
    return glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressESC()
{
    return glfwGetKey(window_, GLFW_KEY_ESCAPE) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressW()
{
    return glfwGetKey(window_, GLFW_KEY_W) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressA()
{
    return glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressS()
{
    return glfwGetKey(window_, GLFW_KEY_S) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressD()
{
    return glfwGetKey(window_, GLFW_KEY_D) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressQ()
{
    return glfwGetKey(window_, GLFW_KEY_Q) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressE()
{
    return glfwGetKey(window_, GLFW_KEY_E) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP0()
{
    return glfwGetKey(window_, GLFW_KEY_KP_0) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP9()
{
    return glfwGetKey(window_, GLFW_KEY_KP_9) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP7()
{
    return glfwGetKey(window_, GLFW_KEY_KP_7) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP4()
{
    return glfwGetKey(window_, GLFW_KEY_KP_4) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP6()
{
    return glfwGetKey(window_, GLFW_KEY_KP_6) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP8()
{
    return glfwGetKey(window_, GLFW_KEY_KP_8) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP2()
{
    return glfwGetKey(window_, GLFW_KEY_KP_2) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP1()
{
    return glfwGetKey(window_, GLFW_KEY_KP_1) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP3()
{
    return glfwGetKey(window_, GLFW_KEY_KP_3) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressKP5()
{
    return glfwGetKey(window_, GLFW_KEY_KP_5) == GLFW_PRESS;
}

bool GLFWContext::GetKeyPressSpace() 
{
    return glfwGetKey(window_, GLFW_KEY_SPACE) == GLFW_PRESS;
}

void GLFWContext::SetTitle(const char * title)
{
    glfwSetWindowTitle (window_, title);
}

void GLFWContext::GetMouse(float &x, float &y) {
    x = GLFWContext::x_;
    y = GLFWContext::y_;
}

float GLFWContext::x_ = 0;
float GLFWContext::y_ = 0;

void GLFWContext::MouseCallback(GLFWwindow* window, double xpos, double ypos) {
    GLFWContext::x_ = (float)xpos;
    GLFWContext::y_ = (float)ypos;
}


GLFWContext::GLFWContext(int h, int w, bool core) :
        GLContext{h, w},window_(nullptr) {
    glfwInit();
    // Set all the required options for GLFW
    if (core) {
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 4);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
        glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, false);
        // glfwWindowHint(GLFW_CONTEXT_CREATION_API, GLFW_EGL_CONTEXT_API);
        // glfwWindowHint(GLFW_VISIBLE, GL_FALSE);
    }
    glfwWindowHint(GLFW_RESIZABLE, GL_FALSE);
    
    const bool isFullScreen = false;
    GLFWmonitor* pMonitor = isFullScreen ? glfwGetPrimaryMonitor() : NULL;
    
    // Create a GLFWwindow object that we can use for GLFW's functions
    window_ = glfwCreateWindow(w, h, "XRobot Debug", pMonitor, nullptr);
    if (window_ == nullptr) {
        fprintf(stderr, "Failed to make GLFW window current!");
        fflush(stderr);
        exit(1);
    }
    
    glfwMakeContextCurrent(window_);
    
    glfwSetCursorPosCallback(window_, MouseCallback);
    //glfwSetInputMode(window_, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    
    GLFWContext::x_ = w / 2;
    GLFWContext::y_ = h / 2;

    this->Init();
}

GLFWContext::~GLFWContext() {
    glfwTerminate();
}

void GLFWContext::Hide() {
    glfwHideWindow(window_);
    printf("[GLFW] Hide the window.\n");
}

bool GLXVisualizationContext::GetKeyPressUp()
{
  return e.type == KeyPress && e.xkey.keycode == 0x6f;
}

bool GLXVisualizationContext::GetKeyPressDown()
{
  return e.type == KeyPress && e.xkey.keycode == 0x74;
}

bool GLXVisualizationContext::GetKeyPressLeft()
{
  return e.type == KeyPress && e.xkey.keycode == 0x71;
}

bool GLXVisualizationContext::GetKeyPressRight()
{
  return e.type == KeyPress && e.xkey.keycode == 0x72;
}

bool GLXVisualizationContext::GetKeyPressESC()
{
  return e.type == KeyPress && e.xkey.keycode == 0x9;
}

bool GLXVisualizationContext::GetKeyPressW()
{
  return e.type == KeyPress && e.xkey.keycode == 0x19;
}

bool GLXVisualizationContext::GetKeyPressA()
{
  return e.type == KeyPress && e.xkey.keycode == 0x26;
}

bool GLXVisualizationContext::GetKeyPressS()
{
  return e.type == KeyPress && e.xkey.keycode == 0x27;
}

bool GLXVisualizationContext::GetKeyPressD()
{
  return e.type == KeyPress && e.xkey.keycode == 0x28;
}

bool GLXVisualizationContext::GetKeyPressQ()
{
  return e.type == KeyPress && e.xkey.keycode == 0x18;
}

bool GLXVisualizationContext::GetKeyPressE()
{
  return e.type == KeyPress && e.xkey.keycode == 0x1a;
}

bool GLXVisualizationContext::GetKeyPressSpace()
{
  return e.type == KeyPress && e.xkey.keycode == 0x41;
}

void GLXVisualizationContext::PollEvent()
{
  XNextEvent(dpy_, &e);
  XSync(dpy_, false);
}

void GLXVisualizationContext::SwapBuffer()
{
  XFlush(dpy_);
  glXSwapBuffers(dpy_, win_);
}

GLXVisualizationContext::GLXVisualizationContext(int h, int w) : GLContext{h, w} {

    // Open display
    dpy_ = XOpenDisplay(NULL);
    if (dpy_ == nullptr) {
        fprintf(stderr, "Failed to connect display");
        fflush(stderr);
        exit(1);
    }
    
    
    scn_ = DefaultScreenOfDisplay(dpy_);
    scnId_ = DefaultScreen(dpy_);
    
    // Check GLX version
    GLint majorGLX, minorGLX = 0;
    glXQueryVersion(dpy_, &majorGLX, &minorGLX);
    if (majorGLX <= 1 && minorGLX < 2) {
        std::cout << "GLX 1.2 or greater is required.\n";
        XCloseDisplay(dpy_);
        exit(1);
    }
    
    GLint glxAttribs[] = {
        GLX_X_RENDERABLE    , True,
        GLX_DRAWABLE_TYPE   , GLX_WINDOW_BIT,
        GLX_RENDER_TYPE     , GLX_RGBA_BIT,
        GLX_X_VISUAL_TYPE   , GLX_TRUE_COLOR,
        GLX_RED_SIZE        , 8,
        GLX_GREEN_SIZE      , 8,
        GLX_BLUE_SIZE       , 8,
        GLX_ALPHA_SIZE      , 8,
        GLX_DEPTH_SIZE      , 24,
        //GLX_STENCIL_SIZE    , 8,
        //GLX_DOUBLEBUFFER    , True,
        None
    }; // None
    
    
    int numberOfFramebufferConfigurations = 0;
    GLXFBConfig* fbc = glXChooseFBConfig(
            dpy_, scnId_, glxAttribs, &numberOfFramebufferConfigurations);
    
    
    // std::cout << "Getting best XVisualInfo\n";
    int best_fbc = -1, worst_fbc = -1, best_num_samp = -1, worst_num_samp = 999;
    for (int i = 0; i < numberOfFramebufferConfigurations; ++i) {
        XVisualInfo *vi = glXGetVisualFromFBConfig( dpy_, fbc[i] );
        if ( vi != 0) {
            int samp_buf, samples;
            glXGetFBConfigAttrib( dpy_, fbc[i], GLX_SAMPLE_BUFFERS, &samp_buf );
            glXGetFBConfigAttrib( dpy_, fbc[i], GLX_SAMPLES       , &samples  );
            
            if ( best_fbc < 0 || (samp_buf && samples > best_num_samp) ) {
                best_fbc = i;
                best_num_samp = samples;
            }
            if ( worst_fbc < 0 || !samp_buf || samples < worst_num_samp ) {
                worst_fbc = i;
            }
            worst_num_samp = samples;
        }
        XFree( vi );
    }
    // std::cout << "Best visual info index: " << best_fbc << "\n";
    GLXFBConfig bestFbc = fbc[ best_fbc ]; 
    
    XVisualInfo* visual = glXGetVisualFromFBConfig( dpy_, bestFbc );
    
    if (visual == 0) {
        std::cout << "Could not create correct visual window.\n";
        XCloseDisplay(dpy_);
        exit(1);
    }
    
    if (scnId_ != visual->screen) {
        std::cout << "screenId(" << scnId_ 
                  << ") does not match visual->screen(" 
                  << visual->screen << ").\n";
        XCloseDisplay(dpy_);
        exit(1);
    }
    
    // printf("pass conf\n");

    XSetWindowAttributes windowAttribs;
    windowAttribs.border_pixel = BlackPixel(dpy_, scnId_);
    windowAttribs.background_pixel = WhitePixel(dpy_, scnId_);
    windowAttribs.override_redirect = True;
    windowAttribs.colormap = XCreateColormap(dpy_,
                                             RootWindow(dpy_, scnId_),
                                             visual->visual,
                                             AllocNone);
    windowAttribs.event_mask = ExposureMask;

    // printf("pass color map\n");

    win_ = XCreateWindow(dpy_,
                         RootWindow(dpy_, scnId_),
                         0,
                         0,
                         w,
                         h,
                         0,
                         visual->depth,
                         InputOutput,
                         visual->visual,
                         CWBackPixel | CWColormap | CWBorderPixel | CWEventMask,
                         &windowAttribs);
    
    // printf("pass create window\n");
    
    // setup function pointers
    typedef GLXContext (*glXCreateContextAttribsARBProc)
                       (Display*, GLXFBConfig, GLXContext, Bool, const int*);
    static glXCreateContextAttribsARBProc glXCreateContextAttribsARB = NULL;
    glXCreateContextAttribsARB = (glXCreateContextAttribsARBProc)
            glXGetProcAddressARB((const GLubyte *)"glXCreateContextAttribsARB");

    GLXContext openGLContext = glXCreateContextAttribsARB(
            dpy_, bestFbc, 0, true, GLXcontextAttribs);
    
    // printf("pass gl context\n");

    XFree( fbc );
    XSync(dpy_, false);
    
    if (!glXMakeCurrent(dpy_, win_, openGLContext)) {
        std::cout << "Cannot create GLX context" << std::endl;
        exit(1);
    }
    
    XSelectInput(dpy_, win_, KeyPressMask | KeymapStateMask);
    
    // std::cout << "GL Vendor: " << glGetString(GL_VENDOR) << "\n";
    // std::cout << "GL Renderer: " << glGetString(GL_RENDERER) << "\n";
    // std::cout << "GL Version: " << glGetString(GL_VERSION) << "\n";
    // std::cout << "GL Shading Language: "
    //           << glGetString(GL_SHADING_LANGUAGE_VERSION) << "\n";
    
    XClearWindow(dpy_, win_);
    XMapRaised(dpy_, win_);
    
    printf("[GLX] Display connected\n");

    this->Init();
}

GLXVisualizationContext::~GLXVisualizationContext() {
    XCloseDisplay(dpy_);
}

} } // xrobot::render_engine
