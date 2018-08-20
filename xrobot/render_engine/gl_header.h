// Copyright 2017-present, Facebook, Inc.
// All rights reserved.
//
// This source code is licensed under the license found in the
// LICENSE file in the root directory of this source tree.

#ifndef GL_HEADER_H_
#define GL_HEADER_H_

#ifdef __linux__
	#define GL_GLEXT_PROTOTYPES
	#include <GL/gl.h>
#endif

#ifdef INCLUDE_GL_CONTEXT_HEADERS

#include <GLFW/glfw3.h>

#ifdef __linux__
#include <X11/Xutil.h>
#include <X11/Xlib.h>
#include <GL/glx.h>
#define EGL_EXT_PROTOTYPES
#include "EGL/egl.h"
#include "EGL/eglext.h"

#if EGL_EGLEXT_VERSION < 20150508
#error "Require EGLEXT >= 20150508 to compile!"
#endif

#endif  // linux

#ifdef __GNUC__
#ifndef __clang__
#if ((__GNUC__ <= 4) && (__GNUC_MINOR__ <= 8))
//#error "GCC >= 4.9 is required!"
#endif
#endif  // clang
#endif  // gnuc

#define CHECK_GL_ERROR \
{ \
    int e = glGetError(); \
    if (e!=GL_NO_ERROR) \
        fprintf(stderr, "%s:%i ERROR: 0x%x\n", __FILE__, __LINE__, e); \
    assert(e == GL_NO_ERROR); \
}

#endif // INCLUDE_GL_CONTEXT_HEADERS


#endif  // GL_HEADER_H_
