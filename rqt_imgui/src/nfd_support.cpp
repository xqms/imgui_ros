// Support methods for NFD
// Author: Max Schwarz <max.schwarz@ais.uni-bonn.de>

#include "nfd_support.h"

#include <QGuiApplication>

#include <X11/X.h>

#include <cstdio>

namespace nfd_support
{
    void setupWindowHandle(QWidget* window, nfdwindowhandle_t* handle)
    {
        if(QGuiApplication::platformName() == "xcb")
        {
            handle->handle = (void*)window->winId();
            handle->type = NFD_WINDOW_HANDLE_TYPE_X11;
        }
        else
        {
            fprintf(stderr, "Unknown platform '%s', not setting window handle\n", qPrintable(QGuiApplication::platformName()));
        }
    }
}
