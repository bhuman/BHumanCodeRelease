/****************************************************************************
**
** Copyright (C) 2016 The Qt Company Ltd.
** Contact: https://www.qt.io/licensing/
**
** This file is part of the Qt SVG module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and The Qt Company. For licensing terms
** and conditions see https://www.qt.io/terms-conditions. For further
** information use the contact form at https://www.qt.io/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 3 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL3 included in the
** packaging of this file. Please review the following information to
** ensure the GNU Lesser General Public License version 3 requirements
** will be met: https://www.gnu.org/licenses/lgpl-3.0.html.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 2.0 or (at your option) the GNU General
** Public license version 3 or any later version approved by the KDE Free
** Qt Foundation. The licenses are as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL2 and LICENSE.GPL3
** included in the packaging of this file. Please review the following
** information to ensure the GNU General Public License requirements will
** be met: https://www.gnu.org/licenses/gpl-2.0.html and
** https://www.gnu.org/licenses/gpl-3.0.html.
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef QSVGFUNCTIONS_WCE_P_H
#define QSVGFUNCTIONS_WCE_P_H

//
//  W A R N I N G
//  -------------
//
// This file is not part of the Qt API.  It exists purely as an
// implementation detail.  This header file may change from version to
// version without notice, or even be removed.
//
// We mean it.
//

#include <QtCore/qglobal.h>

#ifdef Q_OS_WINCE

// File I/O ---------------------------------------------------------

#define _O_RDONLY       0x0001
#define _O_RDWR         0x0002
#define _O_WRONLY       0x0004
#define _O_CREAT        0x0008
#define _O_TRUNC        0x0010
#define _O_APPEND       0x0020
#define _O_EXCL         0x0040

#define O_RDONLY        _O_RDONLY
#define O_RDWR          _O_RDWR
#define O_WRONLY        _O_WRONLY
#define O_CREAT         _O_CREAT
#define O_TRUNC         _O_TRUNC
#define O_APPEND        _O_APPEND
#define O_EXCL          _O_EXCL

//For zlib we need these helper functions, but they break the build when
//set globally, so just set them for zlib use
#ifdef ZLIB_H
#define open qt_wince_open
#define _wopen(a,b,c) qt_wince__wopen(a,b,c)
#define close qt_wince__close
#define lseek qt_wince__lseek
#define read qt_wince__read
#define write qt_wince__write
#endif

int qt_wince__wopen(const wchar_t *filename, int oflag, int pmode);
int qt_wince_open(const char *filename, int oflag, int pmode);
int qt_wince__close(int handle);
long qt_wince__lseek(int handle, long offset, int origin);
int qt_wince__read(int handle, void *buffer, unsigned int count);
int qt_wince__write(int handle, const void *buffer, unsigned int count);

#endif // Q_OS_WINCE
#endif // QSVGFUNCTIONS_WCE_P_H
