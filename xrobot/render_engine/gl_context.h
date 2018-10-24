#ifndef RENDER_ENGINE_GL_CONTEXT_H_
#define RENDER_ENGINE_GL_CONTEXT_H_

#define INCLUDE_GL_CONTEXT_HEADERS
#include "gl_header.h"
#include "../utils.h"

namespace xrobot {
namespace render_engine {

class GLContext {
public:
    GLContext(const int h, const int w, const int device = 0) 
        : h_(h), w_(w), device_(device) {}

    ~GLContext() {}

    virtual void Terminate() {};
    virtual void PollEvent() {};
    virtual void SwapBuffer() {};
    virtual void SetWindowShouldClose() {};
    virtual void GetMouse(float &x, float &y) {};
    virtual void SetTitle(const char * title) {};
    virtual bool GetKeyPressESC() { return false; };
    virtual bool GetKeyPressW() { return false; };
    virtual bool GetKeyPressA() { return false; };
    virtual bool GetKeyPressS() { return false; };
    virtual bool GetKeyPressD() { return false; };
    virtual bool GetKeyPressQ() { return false; };
    virtual bool GetKeyPressE() { return false; };
    virtual bool GetKeyPressUp() { return false; };
    virtual bool GetKeyPressDown() { return false; };
    virtual bool GetKeyPressLeft() { return false; };
    virtual bool GetKeyPressRight() { return false; };
    virtual bool GetKeyPressSpace() { return false; };
    virtual bool GetKeyPress0() { return false; };
    virtual bool GetKeyPress1() { return false; };
    virtual bool GetKeyPress3() { return false; };
    virtual bool GetKeyPress5() { return false; };
    virtual bool GetKeyPress9() { return false; };
    virtual bool GetKeyPress7() { return false; };
    virtual bool GetKeyPress8() { return false; };
    virtual bool GetKeyPress2() { return false; };
    virtual bool GetKeyPress4() { return false; };
    virtual bool GetKeyPress6() { return false; };
    virtual bool GetKeyPressKP0() { return false; };
    virtual bool GetKeyPressKP1() { return false; };
    virtual bool GetKeyPressKP3() { return false; };
    virtual bool GetKeyPressKP5() { return false; };
    virtual bool GetKeyPressKP9() { return false; };
    virtual bool GetKeyPressKP7() { return false; };
    virtual bool GetKeyPressKP8() { return false; };
    virtual bool GetKeyPressKP2() { return false; };
    virtual bool GetKeyPressKP4() { return false; };
    virtual bool GetKeyPressKP6() { return false; };
    virtual bool GetWindowShouldClose() { return false; };

protected:
    void Init() {
        glViewport(0, 0, w_, h_);
    }

    void PrintInfo();

protected:
    int h_;
    int w_;
    int device_;
};


// Context for EGL (server-side OpenGL on some supported GPUs)
class EGLContext : public GLContext {
public:
    EGLContext(const int h, const int w, const int device = 0);
    ~EGLContext();

protected:
    EGLDisplay eglDpy_;
};

// Context for GLFW
class GLFWContext : public GLContext {
  public:
    GLFWContext(int h, int w, bool core = true);
    ~GLFWContext();

    void Terminate() override;
    void PollEvent() override;
    void SwapBuffer() override;
    void SetTitle(const char * title) override;
    bool GetWindowShouldClose() override;
    void SetWindowShouldClose() override;
    bool GetKeyPressESC() override;
    bool GetKeyPressW() override;
    bool GetKeyPressA() override;
    bool GetKeyPressS() override;
    bool GetKeyPressD() override;
    bool GetKeyPressQ() override;
    bool GetKeyPressE() override;
    bool GetKeyPressUp() override;
    bool GetKeyPressDown() override;
    bool GetKeyPressLeft() override;
    bool GetKeyPressRight() override;
    bool GetKeyPress0() override;
    bool GetKeyPress9() override;
    bool GetKeyPress7() override;
    bool GetKeyPress4() override;
    bool GetKeyPress6() override;
    bool GetKeyPress8() override;
    bool GetKeyPress2() override;
    bool GetKeyPress1() override;
    bool GetKeyPress3() override;
    bool GetKeyPress5() override;
    bool GetKeyPressKP0() override;
    bool GetKeyPressKP9() override;
    bool GetKeyPressKP7() override;
    bool GetKeyPressKP4() override;
    bool GetKeyPressKP6() override;
    bool GetKeyPressKP8() override;
    bool GetKeyPressKP2() override;
    bool GetKeyPressKP1() override;
    bool GetKeyPressKP3() override;
    bool GetKeyPressKP5() override;
    bool GetKeyPressSpace() override;
    void GetMouse(float &x, float &y) override;

    static void MouseCallback(GLFWwindow* window, double xpos, double ypos);

  protected:
    GLFWwindow* window_;

    static float x_;
    static float y_;
};

// Context for GLXHeadless
class GLXHeadlessContext : public GLContext {
public:
    GLXHeadlessContext(int h, int w);
    ~GLXHeadlessContext();

protected:
    Display* dpy_;
};

// Context for GLX
class GLXVisualizationContext : public GLContext {
public:
    GLXVisualizationContext(int h, int w);
    ~GLXVisualizationContext();
    
    void PollEvent() override;
    void SwapBuffer() override;
    bool GetKeyPressUp() override;
    bool GetKeyPressDown() override;
    bool GetKeyPressLeft() override;
    bool GetKeyPressRight() override;
    bool GetKeyPressESC() override;
    bool GetKeyPressW() override;
    bool GetKeyPressA() override;
    bool GetKeyPressS() override;
    bool GetKeyPressD() override;
    bool GetKeyPressQ() override;
    bool GetKeyPressE() override;
    bool GetKeyPressSpace() override;

protected:
    Window win_;
    Screen* scn_;
    int scnId_;
    XEvent e;
    Display* dpy_;
};

inline GLContext* CreateContext(int h, int w, int device=0) {
    #ifdef USE_GLX
        return new GLXVisualizationContext{h, w};
    #else
        return new GLFWContext{h, w};
    #endif
}

inline GLContext* CreateHeadlessContext(int h, int w, int device=0) {
    #define USE_EGL
    #ifdef USE_EGL
	   return new EGLContext{h, w, device};		
    #else
        return new GLXHeadlessContext{h, w};
    #endif
}

inline void CloseContext(GLContext * ctx) {
    if (!ctx) { return; }

    ctx->Terminate();
    delete ctx;
}

} } // xrobot::render_engine

#endif // RENDER_ENGINE_GL_CONTEXT_H_