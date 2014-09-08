/****************************************************************************
** 
** Copyright (c) 2009 Nokia Corporation and/or its subsidiary(-ies).
** All rights reserved.
** Contact: Nokia Corporation (qt-info@nokia.com)
** 
** This file is part of a Qt Solutions component.
**
** Commercial Usage  
** Licensees holding valid Qt Commercial licenses may use this file in
** accordance with the Qt Solutions Commercial License Agreement provided
** with the Software or, alternatively, in accordance with the terms
** contained in a written agreement between you and Nokia.
** 
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
** 
** In addition, as a special exception, Nokia gives you certain
** additional rights. These rights are described in the Nokia Qt LGPL
** Exception version 1.1, included in the file LGPL_EXCEPTION.txt in this
** package.
** 
** GNU General Public License Usage 
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
** 
** Please note Third Party Software included with Qt Solutions may impose
** additional restrictions and it is the user's responsibility to ensure
** that they have met the licensing requirements of the GPL, LGPL, or Qt
** Solutions Commercial license and the relevant license of the Third
** Party Software they are using.
** 
** If you are unsure which license is appropriate for your use, please
** contact Nokia at qt-info@nokia.com.
** 
****************************************************************************/


#include "qtdotnetstyle.h"
#include <QStyleOption>
#include <QPainter>
#include <QMainWindow>
#include <QDockWidget>
#include <QDialogButtonBox>
#include <QToolBar>
#include <QDialog>
#include <QLineEdit>
#include <QComboBox>
#include <QLibrary>
#include <QStatusBar>
#include <QApplication>
#include <qt_windows.h>

typedef bool (WINAPI *PtrIsAppThemed)();
typedef HRESULT (WINAPI *PtrGetCurrentThemeName)(OUT LPWSTR pszThemeFileName, int cchMaxNameChars, OUT OPTIONAL LPWSTR pszColorBuff, int                   cchMaxColorChars, OUT OPTIONAL LPWSTR pszSizeBuff, int cchMaxSizeChars);

static PtrIsAppThemed pIsAppThemed = 0;
static PtrGetCurrentThemeName pGetCurrentThemeName = 0;

static const int windowsItemFrame       =  2; // menu item frame width
static const int windowsSepHeight       =  9; // separator item height
static const int windowsItemHMargin     =  3; // menu item hor text margin
static const int windowsItemVMargin     =  2; // menu item ver text margin
static const int windowsArrowHMargin	=  6; // arrow horizontal margin
static const int windowsRightBorder     = 15; // right border on windows

class QtDotNetStylePrivate
{
public:
    QtDotNetStylePrivate()
    {
        QLibrary themeLib("uxtheme");
        themeLib.load();
        if (themeLib.isLoaded()) {  //resolve uxtheme functions
            pIsAppThemed            = (PtrIsAppThemed)themeLib.resolve("IsAppThemed");
            pGetCurrentThemeName    = (PtrGetCurrentThemeName)themeLib.resolve("GetCurrentThemeName");
        }
    }
    ~QtDotNetStylePrivate()
    {}

    enum InternalPalette {
        Blue,
        Silver,
        Olive,
        System,
        Classic
    };

    void init();
    QColor gradientDarkColor(const QStyleOption *option) const;
    QColor gradientLightColor(const QStyleOption *option) const;
    QColor highlightOutlineColor(const QStyleOption *option, bool act) const;
    QColor shadowColor(const QStyleOption *option) const;
    QBrush highlightBrush(const QStyleOption *option, bool act) const;
    QBrush highlightMenuCheckBrush(const QStyleOption *option, bool act) const;
    QLinearGradient toolbarGradient(const QStyleOption *option, QRect rect, Qt::Orientation orientaion) const;
    QLinearGradient menuGradient(const QStyleOption *option, QRect rect, Qt::Orientation orientaion) const;
    void updatePalette();

    QColor menuSeparatorColor;
    QColor menuBackgroundColor;
    QColor menuFrameColor;
    InternalPalette internalPalette;    //internal color theme
    QtDotNetStyle::ColorTheme theme;    //external color theme
    QPalette currentPal;                //used to detect system palette changes
 };



inline QColor QtDotNetStylePrivate::gradientDarkColor(const QStyleOption *option) const {
    QColor color = option->palette.background().color();
    switch (internalPalette) {
    case Olive:
        color = QColor(217, 217, 167);
        break;
    case Blue:
        color = QColor(158, 190, 245);
        break;
    case Silver:
        color = QColor(216, 216, 229);
        break;
    default:
        break;
    }
    return color;
}

inline QColor QtDotNetStylePrivate::gradientLightColor(const QStyleOption *option) const {
    QColor color = option->palette.background().color().lighter(106);
    switch (internalPalette) {
    case Olive:
        color = QColor(241, 240, 227);
        break;
    case Blue:
        color = QColor(196, 217, 249);
        break;
    case Silver:
        color = QColor(242, 242, 247);
        break;
    case Classic:
        color = option->palette.light().color().lighter(106);
    default:
        break;
    }
    return color;
}

inline QColor QtDotNetStylePrivate::shadowColor(const QStyleOption *option) const {
    QColor color;
    switch (internalPalette) {
    case Blue:
        color = QColor(59, 97, 156);
        break;
    case Silver:
        color = QColor(124, 124, 148);
        break;
    case Olive:
        color = QColor(96, 128, 88);
        break;
    case Classic:
        color = option->palette.background().color();
        break;
    default:
        color = option->palette.dark().color();
        break;
    }
    return color;
}

inline QColor QtDotNetStylePrivate::highlightOutlineColor(const QStyleOption *option, bool act = false) const {
    QColor color;
    switch (internalPalette) {
    case Blue:
        color = QColor(0, 0, 128);
        break;
    case Silver:
        color = QColor(75, 75, 111);
        break;
    case Olive:
        color = QColor(63, 93, 56);
        break;
    default:
        if (act)
            return option->palette.highlight().color().darker(110);
        else
            return option->palette.highlight().color().lighter(110);
        break;
    }
    return color;
}

QBrush QtDotNetStylePrivate::highlightBrush(const QStyleOption *option, bool act) const {
    QBrush brush;
    QColor highlight;
    switch (internalPalette) {
    case Silver:
    case Olive:
    case Blue:
        if (const QStyleOptionMenuItem *menuitem = qstyleoption_cast<const QStyleOptionMenuItem *>(option)) {
            brush = QColor(255, 238, 194);
        } else {
            QLinearGradient gradient(option->rect.topLeft(), option->rect.bottomLeft());
            if (act) {
                gradient.setColorAt(0, QColor(254, 149, 82));
                gradient.setColorAt(1, QColor(255, 211, 142));
            } else {
                gradient.setColorAt(0, QColor(255, 252, 200));
                gradient.setColorAt(1, QColor(255, 208, 145));
            }
            brush = gradient;
        }
        break;
    default:
        highlight = option->palette.highlight().color().lighter(120);
        if (act)
            highlight.setHsv(highlight.hue(), 70, 220);
        else
            highlight.setHsv(highlight.hue(), 40, 230);
        brush = QBrush(highlight);
        break;
    }
    return brush;
}

QBrush QtDotNetStylePrivate::highlightMenuCheckBrush(const QStyleOption *option, bool act) const {
    QBrush brush;
    QColor highlight;

    switch (internalPalette) {
    case Silver:
    case Olive:
    case Blue:
        if (act)
            brush = QColor(254, 128, 62);
        else
            brush = QColor(255, 192, 111);
        break;
    default:
        highlight = option->palette.highlight().color().lighter(120);
        if (act)
            highlight.setHsv(highlight.hue(), 120, 180);
        else {
            highlight.setHsv(highlight.hue(), 30, 230);
        }
        highlight.setAlpha(170);
        brush = highlight;
        break;
    }
    return brush;
}

QLinearGradient QtDotNetStylePrivate::toolbarGradient(const QStyleOption *option, QRect rect, Qt::Orientation orientation= Qt::Vertical) const {
    QLinearGradient gradient(rect.topLeft(), orientation == Qt::Vertical ? rect.topRight() : rect.bottomLeft());
    QColor start, stop;
    switch (internalPalette)
    {
        case Blue:
            start = QColor(220, 235, 254);
            stop = QColor(129, 169, 226);
            gradient.setColorAt(0.3, start);
            break;
        case Silver:
            start = QColor(242, 242, 249);
            stop = QColor(153, 151, 181);
            gradient.setColorAt(0.3, start);
            break;
        case Olive:
            start = QColor(241, 245, 217);
            stop = QColor(183, 198, 145);
            gradient.setColorAt(0.0, start);
            break;
        default:
            start = option->palette.light().color();
            if (internalPalette == Classic)
                start = start.lighter(130);
            start.setAlpha(180);
            stop = option->palette.button().color();
            stop.setHsv(stop.hue(), qMin(stop.saturation()*2, 255), (stop.value()*2)/3);
            stop.setAlpha(128);
            if (internalPalette == Classic)
                stop.setAlpha(0);
            gradient.setColorAt(0.3, start);
            break;
    }
    gradient.setColorAt(1, stop);
    return gradient;
}


QLinearGradient QtDotNetStylePrivate::menuGradient(const QStyleOption *option, QRect rect, Qt::Orientation orientaion = Qt::Horizontal) const {

    QColor start, stop;
    QLinearGradient grad;
    if (orientaion == Qt::Vertical) {
        grad.setStart(rect.left(), rect.top());
        grad.setFinalStop(rect.left(), rect.bottom());
    } else {
        grad.setStart(QPoint(rect.left(), 0));
        grad.setFinalStop(QPoint(rect.left() + 24, 0));
    }
    switch (internalPalette)
    {
        case Blue:
            start = QColor(220, 235, 254);
            stop = QColor(129, 169, 226);
            grad.setColorAt(0.3, start);
            break;
        case Olive:
            start = QColor(246, 248, 224);
            stop = QColor(186, 201, 148);
            grad.setColorAt(0.0, start);
            break;
        case Silver:
            start = QColor(215, 215, 226);
            stop = QColor(138, 137, 166);
            grad.setColorAt(0.4, start);
            break;
        default:
            start = option->palette.brush(QPalette::Active, QPalette::Base).color();
            stop = gradientDarkColor(option);
            stop.setHsv(stop.hue(), qMin((int)(stop.saturation()*1.2), 255), (5*stop.value()/7));
            if (internalPalette == Classic)
                stop.setAlpha(120);
            grad.setColorAt(0, start);
            break;
    }
    grad.setColorAt(1, stop);
    return grad;
}



/*!
    \class QtDotNetStyle qtdotnetstyle.h
    \brief The QtDotNetStyle class provides an alternative, modern look for Qt applications on Windows.

    The GUI look is very similar to the style introduced in
    Visual Studio 2005, Office 2003 as well as in .NET Windows
    Forms (when the RenderMode is set to "Professional"
    instead of "System"). It gives applications a modern
    gradient appearance for tool bars and menus while
    retaining the native look and feel for other widgets.

    The style supports different color themes, which may be specified in
    the constructor. The default is the \c Office color theme.

    This is the textedit example running with the Standard style:
    \img dotnetstyle.png

    This is the textedit example running with the Office style:
    \img dotnetstyle2.png

    \sa QStyle, QWindowsVistaStyle, QWindowsXPStyle
*/


/*!
    \enum QtDotNetStyle::ColorTheme

    This enum describes the different color schemes supported by the style.

    \value Standard Use gradients based on the operating system palette.
    \value Office Use Microsoft Office 2003 style gradients.
*/

/*!
    Constructs a QtDotNetStyle object which uses the default color
    theme (\c Office).
*/
QtDotNetStyle::QtDotNetStyle()
: QWindowsVistaStyle(), d(new QtDotNetStylePrivate())
{
    d->theme = Office;

    // Designer does not actually do polish(QApplication*), see task 175327
    d->currentPal = qApp->palette();
    d->updatePalette();
}


/*!
    Constructs a QtDotNetStyle object which uses the color theme \a
    colortheme.
*/
QtDotNetStyle::QtDotNetStyle(ColorTheme colortheme)
    : QWindowsVistaStyle(), d(new QtDotNetStylePrivate())
{
    d->theme = colortheme;
}


/*!
    Destroys the QtDotNetStyle object.
*/
QtDotNetStyle::~QtDotNetStyle()
{
    delete d;
}


/*!
 \reimp
 */
void QtDotNetStyle::drawPrimitive(PrimitiveElement element, const QStyleOption *option,
                                    QPainter *painter, const QWidget *widget) const
{
    QColor gradientDark = d->gradientDarkColor(option);
    QColor gradientLight = d->gradientLightColor(option);
    QColor shadow = d->shadowColor(option);

    QRect rect = option->rect;
    switch (element) {
    case PE_IndicatorDockWidgetResizeHandle:
        break;
    case PE_PanelLineEdit:
        if (widget && qobject_cast<QToolBar*>(widget->parentWidget())) {
            QRect rect = option->rect.adjusted(2, 1, -2, -2);
            painter->save();
            bool active = option->state & State_Active &&
                            ((option->state & State_MouseOver)
                            || (option->state & State_HasFocus));
            QBrush editBrush = option->palette.brush(QPalette::Base);
            painter->setPen(QPen(editBrush.color()));
            painter->fillRect(rect, editBrush);

            QRect re = option->rect ;
            if (active) {
                painter->setPen(d->highlightOutlineColor(option));
                painter->drawRect(rect);
            }
            painter->restore();
        } else {
            if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
                QWindowsVistaStyle::drawPrimitive(element, option, painter, widget);
            else
                QWindowsXPStyle::drawPrimitive(element, option, painter, widget);
        }
    case PE_PanelButtonTool:
        {
            QStyleOption opt2 = *option;
            if (widget)
                if(QDockWidget *dw = qobject_cast<QDockWidget *>(widget->parentWidget())) {
                    if (!dw->isFloating())
                        opt2.palette.setBrush(QPalette::Button, Qt::transparent);
                }
            if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
                QWindowsVistaStyle::drawPrimitive(element, &opt2, painter, widget);
            else
                QWindowsXPStyle::drawPrimitive(element, &opt2, painter, widget);
        }
        break;
    case PE_FrameMenu:
        {
            QPen pen = painter->pen();
            QBrush brush = painter->brush();
            if (const QStyleOptionToolBar *toolbar = qstyleoption_cast<const QStyleOptionToolBar*>(option))
            {
                Qt::Orientation orientation =(toolbar->toolBarArea == Qt::RightToolBarArea ||
                                              toolbar->toolBarArea == Qt::LeftToolBarArea) ?
                                              Qt::Vertical : Qt::Horizontal;
                painter->setBrush(d->toolbarGradient(toolbar, toolbar->rect, orientation));
            } else {
                painter->setBrush(d->menuBackgroundColor);
            }
            painter->setPen(d->menuFrameColor);
            painter->drawRect(option->rect.adjusted(0, 0, -1, -1));
            painter->setPen(pen);
            painter->setBrush(brush);
        }
        break;
    case PE_Widget:
        if (const QMainWindow *mw = qobject_cast<const QMainWindow *>(widget)) {
            if (d->currentPal != qApp->palette()) //workaround for style polish on theme color change
            {
                d->currentPal = qApp->palette();
                d->updatePalette();
            }
            painter->save();
            QPoint begin = widget->geometry().topRight();
            QPoint end = widget->geometry().topLeft() + QPoint(0, widget->geometry().height()/2);
            begin = widget->mapFromGlobal(begin);
            end = widget->mapFromGlobal(end);
            QLinearGradient menubargradient(begin, end);
            menubargradient.setColorAt(0, gradientLight);
            if (d->internalPalette == QtDotNetStylePrivate::System)
                menubargradient.setColorAt(0.3, gradientDark);
            else
                menubargradient.setColorAt(0.8, gradientDark);
            menubargradient.setColorAt(1, gradientDark);
            QRect windowRect = option->rect;
            if (QStatusBar *statusBar = mw->findChild<QStatusBar*>()) {
                if (statusBar->isVisible()) {
                    windowRect.adjust(0, 0, 0, -statusBar->height());
                    painter->setPen(option->palette.background().color().lighter(106));
                    painter->drawLine(windowRect.bottomLeft() + QPoint(0, 1),
                                        windowRect.bottomRight() + QPoint(0, 1));
                }
            }
            painter->fillRect(windowRect, menubargradient);
            painter->restore();
        }
        break;
    case PE_IndicatorToolBarSeparator:
        {
            QRect rect = option->rect;
            shadow.setAlpha(180);
            painter->setPen(shadow);
            const int margin = 3;
            if (option->state & State_Horizontal) {
                const int offset = rect.width()/2;
                painter->drawLine(rect.bottomLeft().x() + offset,
                            rect.bottomLeft().y() - margin,
                            rect.topLeft().x() + offset,
                            rect.topLeft().y() + margin);
                painter->setPen(QPen(option->palette.background().color().light(110)));
                painter->drawLine(rect.bottomLeft().x() + offset + 1,
                            rect.bottomLeft().y() - margin,
                            rect.topLeft().x() + offset + 1,
                            rect.topLeft().y() + margin);
            } else { //Draw vertical separator
                const int offset = rect.height()/2;
                painter->setPen(QPen(option->palette.background().color().dark(110)));
                painter->drawLine(rect.topLeft().x() + margin ,
                            rect.topLeft().y() + offset,
                            rect.topRight().x() - margin,
                            rect.topRight().y() + offset);
                painter->setPen(QPen(option->palette.background().color().light(110)));
                painter->drawLine(rect.topLeft().x() + margin ,
                            rect.topLeft().y() + offset + 1,
                            rect.topRight().x() - margin,
                            rect.topRight().y() + offset + 1);
            }
        }
        break;
    case PE_IndicatorToolBarHandle:
        painter->save();
        {
            QColor gripColor = shadow;
            if (d->internalPalette != QtDotNetStylePrivate::System)
                gripColor = shadow.darker(120);
            if (option->state & State_Horizontal) {
                for (int i = rect.height()/5; i <= 4*(rect.height()/5) ; i+=4) {
                    int y = rect.topLeft().y() + i + 1;
                    int x1 = rect.topLeft().x() + 3;
                    painter->fillRect(x1 + 1, y, 2, 2, Qt::white);
                    painter->fillRect(x1, y - 1, 2, 2, gripColor);
                }
            }
            else { //vertical toolbar
                for (int i = rect.width()/5; i <= 4*(rect.width()/5) ; i+=4) {
                    int x = rect.topLeft().x() + i + 1;
                    int y1 = rect.topLeft().y() + 3;
                    painter->fillRect(x , y1 + 1, 2, 2, Qt::white);
                    painter->fillRect(x - 1, y1, 2, 2, gripColor);
                }
            }
        }
        painter->restore();
        break;
    case PE_PanelMenuBar:
        break;
    default:
        if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
            QWindowsVistaStyle::drawPrimitive(element, option, painter, widget);
        else
            QWindowsXPStyle::drawPrimitive(element, option, painter, widget);
        break;
    }
}

/*!
 \reimp
 */
void QtDotNetStyle::drawControl(ControlElement element, const QStyleOption *option,
                                  QPainter *painter, const QWidget *widget) const
{

    QColor gradientDark = d->gradientDarkColor(option);
    QColor gradientLight = d->gradientLightColor(option);
    QColor shadow = d->shadowColor(option);
    QRect rect = option->rect;

    switch (element) {
#ifndef QT_NO_MENU
    case CE_MenuItem:
        painter->save();
        if (const QStyleOptionMenuItem *menuitem = qstyleoption_cast<const QStyleOptionMenuItem *>(option)) {
            int x, y, w, h;
            QStyleOptionMenuItem mbiCopy = *menuitem;
            painter->fillRect(rect, d->menuBackgroundColor);
            painter->fillRect(QRect(0, rect.top(), 25, rect.bottom()), d->menuGradient(option, rect));
            menuitem->rect.getRect(&x, &y, &w, &h);

            int tab = menuitem->tabWidth;
            bool dis = !(menuitem->state & State_Enabled);
            bool checked = menuitem->checkType != QStyleOptionMenuItem::NotCheckable
                            ? menuitem->checked : false;
            bool act = menuitem->state & State_Selected;

            // windows always has a check column, regardless whether we have an icon or not
            int checkcol = qMax(menuitem->maxIconWidth, 20);
            if (menuitem->menuItemType == QStyleOptionMenuItem::Separator) {
                int yoff = y-1 + h / 2;
                painter->setPen(d->menuSeparatorColor);
                painter->drawLine(x + 32, yoff, x + w + 6, yoff);
                painter->restore();
                return;
            }

            QRect vCheckRect = visualRect(option->direction, menuitem->rect,
                                QRect(menuitem->rect.x(), menuitem->rect.y(), checkcol, menuitem->rect.height()));
            vCheckRect.adjust(2, 0, 0, 0);

            if (act) {
                painter->setPen(d->highlightOutlineColor(option));
                painter->setBrush(d->highlightBrush(option, false));
                painter->drawRect(option->rect.adjusted(0, 0, -2, -2));
            }
            if (checked) {
                painter->save();
                painter->setPen(d->highlightOutlineColor(option, act));
                painter->setBrush(d->highlightMenuCheckBrush(option, act));
                painter->drawRect(vCheckRect.adjusted(-1, 1, 0, -3));
                painter->restore();
            }
            if (!menuitem->icon.isNull()) {
                QIcon::Mode mode = dis ? QIcon::Disabled : QIcon::Normal;
                if (act && !dis)
                    mode = QIcon::Active;
                QPixmap pixmap;
                if (checked)
                    pixmap = menuitem->icon.pixmap(pixelMetric(PM_SmallIconSize), mode, QIcon::On);
                else
                    pixmap = menuitem->icon.pixmap(pixelMetric(PM_SmallIconSize), mode);
                int pixw = pixmap.width();
                int pixh = pixmap.height();
                QRect pmr(0, 0, pixw, pixh);
                pmr.moveCenter(vCheckRect.center());
                painter->setPen(menuitem->palette.text().color());
                painter->drawPixmap(pmr.topLeft(), pixmap);
            } else if (checked) {
                QStyleOptionMenuItem newMi = *menuitem;
                newMi.state = State_None;
                if (!dis)
                    newMi.state |= State_Enabled;
                if (act)
                    newMi.state |= State_On;
                newMi.rect = visualRect(option->direction, menuitem->rect,
                                        QRect(menuitem->rect.x() + windowsItemFrame, menuitem->rect.y() + windowsItemFrame,
                                        checkcol - 2 * windowsItemFrame, menuitem->rect.height() - 2*windowsItemFrame));
                drawPrimitive(PE_IndicatorMenuCheckMark, &newMi, painter, widget);
            }
            painter->setPen(menuitem->palette.buttonText().color());

            QColor discol;
            if (dis) {
                discol = menuitem->palette.text().color();
                painter->setPen(discol);
            }

            int xm = windowsItemFrame + checkcol + windowsItemHMargin;
            int xpos = menuitem->rect.x() + xm;
            QRect textRect(xpos + 5, y + windowsItemVMargin, w - xm - windowsRightBorder - tab + 1, h - 2 * windowsItemVMargin);
            QRect vTextRect = visualRect(option->direction, menuitem->rect, textRect);
            QString s = menuitem->text;

            if (!s.isEmpty()) { // draw text
                painter->save();
                int t = s.indexOf(QLatin1Char('\t'));
                int text_flags = Qt::AlignVCenter | Qt::TextShowMnemonic | Qt::TextDontClip | Qt::TextSingleLine;
                if (!styleHint(SH_UnderlineShortcut, menuitem, widget))
                    text_flags |= Qt::TextHideMnemonic;
                text_flags |= Qt::AlignLeft;
                if (t >= 0) {
                    QRect vShortcutRect = visualRect(option->direction, menuitem->rect,
                    QRect(textRect.topRight(), QPoint(menuitem->rect.right(), textRect.bottom())));
                    if (dis && !act) {
                        painter->setPen(discol);
                    }
                    painter->drawText(vShortcutRect, text_flags, s.mid(t + 1));
                    s = s.left(t);
                }
                QFont font = menuitem->font;
                if (menuitem->menuItemType == QStyleOptionMenuItem::DefaultItem)
                    font.setBold(true);
                painter->setFont(font);
                if (dis && !act) {
                    painter->setPen(discol);
                }
                painter->drawText(vTextRect, text_flags, s.left(t));
                painter->restore();
            }
            if (menuitem->menuItemType == QStyleOptionMenuItem::SubMenu) {// draw sub menu arrow
                int dim = (h - 2 * windowsItemFrame) / 2;
                PrimitiveElement arrow;
                arrow = (option->direction == Qt::RightToLeft) ? PE_IndicatorArrowLeft : PE_IndicatorArrowRight;
                xpos = x + w - windowsArrowHMargin - windowsItemFrame - dim;
                QRect  vSubMenuRect = visualRect(option->direction, menuitem->rect, QRect(xpos, y + h / 2 - dim / 2, dim, dim));
                QStyleOptionMenuItem newMI = *menuitem;
                newMI.rect = vSubMenuRect;
                newMI.state = dis ? State_None : State_Enabled;
                drawPrimitive(arrow, &newMI, painter, widget);
            }
        }
        painter->restore();
        break;
#endif // QT_NO_MENU

        case CE_MenuEmptyArea:
            {
                painter->fillRect(option->rect, option->palette.base());
                QLinearGradient grad(QPoint(rect.left() , 0), QPoint(rect.left() + 23, 0));
                grad.setColorAt(0, gradientLight.light(110));
                grad.setColorAt(1, gradientDark);
                painter->fillRect(QRect(0, rect.top() + 2, 23, rect.bottom() - 3), grad);
            }
            break;
        case CE_MenuBarItem:
            if (const QStyleOptionMenuItem *mbi = qstyleoption_cast<const QStyleOptionMenuItem *>(option)) {
                QRect rect(QPoint(0, 0), widget->window()->size());
                {
                    if (widget->window())
                        drawPrimitive(PE_Widget, option, painter, widget);
                }
                if (option->state & (QStyle::State_Sunken | QStyle::State_Selected))
                {
                    QColor highlight = shadow;
                    painter->setPen(d->menuFrameColor);
                    if (option->state & QStyle::State_Sunken) {

                        if (d->internalPalette == QtDotNetStylePrivate::System)
                            painter->setBrush(Qt::white);
                        else
                            painter->setBrush(d->menuGradient(option, option->rect, Qt::Vertical));

                        painter->drawRect(option->rect.adjusted(0, 2, -1, 0));
                        //draw shadow behind:
                        QColor shade = Qt::black;
                        shade.setAlpha(50);
                        painter->setBrush(shade);
                        painter->setPen(Qt::transparent);
                        QRect shadowRect(option->rect.topRight() + QPoint(0, 4), option->rect.bottomRight() + QPoint(1,0));
                        painter->drawRect(shadowRect);
                    }
                    else {
                        painter->setPen(d->highlightOutlineColor(option));
                        painter->setBrush(d->highlightBrush(option, false));
                        painter->drawRect(option->rect.adjusted(0, 2, -2, -2));
                    }
                }
                QStyleOptionMenuItem mbiCopy = *mbi;
                QPalette pal = mbi->palette;
                pal.setBrush(QPalette::All, QPalette::ButtonText, mbi->palette.text());
                mbiCopy.palette = pal;
                QCommonStyle::drawControl(element, &mbiCopy, painter, widget);
        }
            break;
        case CE_MenuBarEmptyArea:
            if (widget->window())
                drawPrimitive(PE_Widget, option, painter, widget);
            break;
        case CE_DockWidgetTitle:
            if (const QStyleOptionDockWidget *dockWidget = qstyleoption_cast<const QStyleOptionDockWidget *>(option)) {
                painter->save();
                /*
                if (d->internalPalette == QtDotNetStylePrivate::System) {
                    painter->setBrush(option->palette.dark().color().light(130));
                    painter->setPen(option->palette.dark().color());
                } else if (d->internalPalette == QtDotNetStylePrivate::Classic) {
                    painter->setBrush(option->palette.background().color().dark(108));
                    painter->setPen(option->palette.dark().color());
                } else {
                    painter->setBrush(d->toolbarGradient(option, option->rect, Qt::Horizontal));
                    QColor color = d->gradientDarkColor(option).darker(120);
                    painter->setPen(color);
                }
                */

                const QStyleOptionDockWidgetV2 *v2
                    = qstyleoption_cast<const QStyleOptionDockWidgetV2*>(option);
                bool verticalTitleBar = v2 == 0 ? false : v2->verticalTitleBar;

                QRect rect = dockWidget->rect;
                QRect r = rect;

                if (verticalTitleBar) {
                    QSize s = r.size();
                    s.transpose();
                    r.setSize(s);

                    painter->translate(r.left() - 1, r.top() + r.width());
                    painter->rotate(-90);
                    painter->translate(-r.left() + 1, -r.top());
                }

                
                QLinearGradient gradient = d->toolbarGradient(option, r, Qt::Horizontal);
                //if(painter->font().bold())
                  //gradient.setColorAt(0.3, d->highlightMenuCheckBrush(option, false).color());
                painter->setBrush(gradient);
                painter->setPen(Qt::NoPen);

                //painter->drawRect(r.adjusted(0, 2, -1, -3));
                //if (roundEdges) 
                {
                  QRect rect = r.adjusted(0, 1, 0, -2).adjusted(0, 0, 0, 2); //.adjusted(1, 1, -1, 0);
                  QRegion region = rect;
                  region -= QRect(rect.left(), rect.top(), 2, 1);
                  region -= QRect(rect.right() - 1, rect.top(), 2, 1);
                  region -= QRect(rect.left(),  rect.top() + 1, 1, 1);
                  region -= QRect(rect.right(), rect.top() + 1, 1, 1);
                  painter->setClipRegion(region);

                  painter->fillRect(rect, gradient);
                  
                  painter->setPen(QPen(shadow, 0));
                  painter->drawLine(rect.bottomLeft(), rect.bottomRight());
                  
                  QColor alphashadow = shadow;
                  alphashadow.setAlpha(128);
                  painter->setPen(QPen(alphashadow, 0));
                  painter->drawLine(rect.topRight() + QPoint(0, 2), rect.bottomRight() - QPoint(0, 1));
                }
                //else
                  //painter->fillRect(r.adjusted(0, 1, 0, -2), gradient);

                if (!dockWidget->title.isEmpty()) {
                    QPalette palette = dockWidget->palette;
                    QRect titleRect = subElementRect(SE_DockWidgetTitleBarText, option, widget);
                    if (verticalTitleBar) {
                        titleRect = QRect(r.left() + rect.bottom()
                                            - titleRect.bottom(),
                                        r.top() + titleRect.left() - rect.left(),
                                        titleRect.height(), titleRect.width());
                    }
                    QString titleText = painter->fontMetrics().elidedText(dockWidget->title, Qt::ElideRight, titleRect.width());
                    drawItemText(painter, titleRect,
                                Qt::AlignLeft | Qt::AlignVCenter | Qt::TextShowMnemonic, palette,
                                dockWidget->state & State_Enabled, titleText, QPalette::WindowText);
                }
                painter->restore();
            }
            break;
        case CE_ToolBar:
            if (const QStyleOptionToolBar *toolbar = qstyleoption_cast<const QStyleOptionToolBar*>(option)) {
                painter->save();
                rect.adjust(1, 0, 0, 0);

                if (const QToolBar *tb =  qobject_cast<const QToolBar *>(widget)) {
                    if (option->direction == Qt::LeftToRight)
                        rect.setRight(tb->childrenRect().right() + 2);
                    else
                        rect.setLeft(tb->childrenRect().left() - 2);
                }
                Qt::Orientation gradientOrientation =(toolbar->toolBarArea == Qt::RightToolBarArea ||
                                                       toolbar->toolBarArea == Qt::LeftToolBarArea) ? Qt::Vertical : Qt::Horizontal;
                QLinearGradient gradient = d->toolbarGradient(toolbar, rect, gradientOrientation);
                painter->setBrush(gradient);
                painter->setPen(Qt::NoPen);

                bool roundEdges = true;
                if (widget && widget->parentWidget()) {
                    if (const QToolBar *tb =  qobject_cast<const QToolBar *>(widget)) {
                        if (!(tb->childrenRect().width() + 32 < widget->parentWidget()->geometry().width())) {
                            roundEdges = false;
                            rect.adjust(-1, 0, 0, 0);
                        }
                    }
                }
                if (roundEdges) {
                    rect = rect.adjusted(1, 1, -1, 0);
                    QRegion region = rect;
                    region -= QRect(rect.left(), rect.bottom(), 2, 1);
                    region -= QRect(rect.right() - 1, rect.bottom(), 2, 1);
                    region -= QRect(rect.left(),  rect.bottom() - 1, 1, 1);
                    region -= QRect(rect.right(), rect.bottom() - 1, 1, 1);
                    region -= QRect(rect.left(), rect.top(), 2, 1);
                    region -= QRect(rect.right() - 1, rect.top(), 2, 1);
                    region -= QRect(rect.left(),  rect.top() + 1, 1, 1);
                    region -= QRect(rect.right(), rect.top() + 1, 1, 1);
                    painter->setClipRegion(region);

                    painter->fillRect(rect, gradient);

                    painter->setPen(QPen(shadow, 0));
                    painter->drawLine(rect.bottomLeft() + QPoint(3,0), rect.bottomRight() - QPoint(2,0));
                    painter->drawPoint(rect.bottomRight() - QPoint(1, 1));
                    QColor alphashadow = shadow;
                    alphashadow.setAlpha(128);
                    painter->setPen(QPen(alphashadow, 0));
                    painter->drawLine(rect.topRight() + QPoint(0, 2), rect.bottomRight() - QPoint(0, 2));
                } else {
                    painter->fillRect(rect, gradient);
                }
                painter->restore();
            }
            break;

        case CE_Splitter:
            break;

        default:
            if (painter)
                if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
                    QWindowsVistaStyle::drawControl(element, option, painter, widget);
                else
                    QWindowsXPStyle::drawControl(element, option, painter, widget);
            break;
    }
}

/*!
 \reimp
 */
void QtDotNetStyle::drawComplexControl(ComplexControl control, const QStyleOptionComplex *option,
                                         QPainter *painter, const QWidget *widget) const
{
    switch (control) {
        case CC_ComboBox:
            if (widget && qobject_cast<QToolBar *>(widget->parentWidget())) {
                QColor highlightedOutlineColor = d->highlightOutlineColor(option);
                if (const QStyleOptionComboBox *cmb = qstyleoption_cast<const QStyleOptionComboBox *>(option))
                {
                    bool active = cmb->state & State_Active &&
                                    ((cmb->state & State_MouseOver)
                                        || (cmb->state & State_On)
                                        || (cmb->state & State_HasFocus));
                    QRect editRect = QWindowsStyle::subControlRect(CC_ComboBox, cmb, SC_ComboBoxEditField, widget).adjusted(0, 0, 3, 1);
                    QRect rect = option->rect.adjusted(1, 1, -3, -2);
                    editRect.setLeft(rect.left());

                    if (cmb->subControls & SC_ComboBoxEditField) {
                        painter->save();
                        QBrush editBrush = cmb->palette.brush(QPalette::Base);
                        painter->setPen(QPen(editBrush.color()));
                        painter->drawRect(rect);
                        painter->fillRect(editRect, editBrush);

                        if (active) {
                            QColor highlight = highlightedOutlineColor;
                            painter->setPen(highlight);
                            painter->drawRect(rect);
                        }
                        painter->restore();
                    }

                    if (cmb->subControls & SC_ComboBoxArrow) {
                        State flags = State_None;

                        QRect ar = QWindowsStyle::subControlRect(CC_ComboBox, cmb, SC_ComboBoxArrow, widget).adjusted(1, 0, -1, 0);
                        if (active) {
                            painter->save();
                            painter->fillRect(ar, d->highlightBrush(option, cmb->state & State_Sunken));
                            painter->setPen(highlightedOutlineColor);
                            painter->drawLine(ar.topLeft(), ar.bottomLeft());
                            painter->restore();
                        }
                        ar.adjust(5, 5, -2, -2);
                        if (option->state & State_Enabled)
                            flags |= State_Enabled;
                        QStyleOption arrowOpt(0);
                        arrowOpt.rect = ar;
                        arrowOpt.palette = cmb->palette;
                        arrowOpt.state = flags;
                        QWindowsStyle::drawPrimitive(PE_IndicatorArrowDown, &arrowOpt, painter, widget);
                    }
                }
            } else {
                if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
                    QWindowsVistaStyle::drawComplexControl(control, option, painter, widget);
                else
                    QWindowsXPStyle::drawComplexControl(control, option, painter, widget);
            }
            break;

    case CC_ToolButton:
        if (const QStyleOptionToolButton *toolbutton = qstyleoption_cast<const QStyleOptionToolButton *>(option))
        {
            QRect button;
            button = subControlRect(control, toolbutton, SC_ToolButton, widget);
            if ((widget && qobject_cast<QToolBar*>(widget->parentWidget())) &&
                ((toolbutton->state & State_MouseOver && toolbutton->state & State_Enabled)
                || toolbutton->state & State_On)) {
                QLinearGradient menubargradient2(button.topLeft(), button.bottomLeft());

                bool act = (toolbutton->state & State_Sunken) || (toolbutton->state & State_On);
                painter->setPen(d->highlightOutlineColor(option, act));
                painter->setBrush(d->highlightBrush(option, act));
                painter->drawRect(button.adjusted(0, 0, -2, -1));

                QStyleOptionComplex comp;
                comp.init(widget);
                comp.rect = button;
                QStyleOptionToolButton label = *toolbutton;
                int fw = pixelMetric(PM_DefaultFrameWidth, option, widget);
                label.rect = button.adjusted(fw, fw, -fw, -fw);
                label.features &= ~QStyleOptionToolButton::Arrow;
                drawControl(CE_ToolButtonLabel, &label, painter, widget);

                if (toolbutton->subControls & SC_ToolButtonMenu) {
                    QStyleOption tool = *toolbutton;
                    tool.rect = QWindowsStyle::subControlRect(control, toolbutton, SC_ToolButtonMenu, widget);
                    painter->setPen(d->highlightOutlineColor(option, act));
                    painter->setBrush(d->highlightBrush(option, toolbutton->activeSubControls & SC_ToolButtonMenu));
                    painter->drawRect(tool.rect.adjusted(-2, 0, -1, -1));
                    tool.rect.adjust(2, 3, -2, -2);
                    drawPrimitive(PE_IndicatorArrowDown, &tool, painter, widget);
                }
            } else
                if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
                    QWindowsVistaStyle::drawComplexControl(control, option, painter, widget);
                else
                    QWindowsXPStyle::drawComplexControl(control, option, painter, widget);
        }
        break;

    default:
        if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
            QWindowsVistaStyle::drawComplexControl(control, option, painter, widget);
        else
            QWindowsXPStyle::drawComplexControl(control, option, painter, widget);
        break;
    }
}

/*!
 \reimp
 */
QSize QtDotNetStyle::sizeFromContents(ContentsType type, const QStyleOption *option,
                                        const QSize &size, const QWidget *widget) const
{
    QSize newSize;
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        newSize = QWindowsVistaStyle::sizeFromContents(type, option, size, widget);
    else
        newSize = QWindowsXPStyle::sizeFromContents(type, option, size, widget);

    switch (type) {
    case CT_LineEdit:
        if (widget && qobject_cast<QToolBar*>(widget->parentWidget())) {
            newSize = QWindowsStyle::sizeFromContents(type, option, size, widget);
            newSize += QSize(0, 2);
        }
        break;
    case CT_ComboBox:
        if (widget && qobject_cast<QToolBar*>(widget->parentWidget())) {
            newSize = QWindowsStyle::sizeFromContents(type, option, size, widget);
            newSize += QSize(0, 2);
        }
        break;
    case CT_MenuBarItem:
        newSize += QSize(0, 1);
        break;
    case CT_MenuItem:
        if (const QStyleOptionMenuItem *menuitem = qstyleoption_cast<const QStyleOptionMenuItem *>(option)) {
            newSize = QWindowsStyle::sizeFromContents(type, option, size, widget);
            if (menuitem->menuItemType == QStyleOptionMenuItem::Separator){
                newSize.setHeight(2);
            } else {
                newSize.rwidth() += 10;
                //ensure menu items have uniform height
                if (newSize.height() < 22)
                    newSize.setHeight(22);
            }
        }
        break;
    case CT_Menu:
        //work around for a menu frame issue (same as xp style)
        newSize = size;
        newSize -= QSize(1,2);
        break;
    case CT_ToolButton:
        if (const QStyleOptionToolButton *toolbutton = qstyleoption_cast<const QStyleOptionToolButton *>(option)) {
            // This is somewhat of a workaround for task (183347) but it prevents us from having to
            //copy toolbuttonlabel from common style
            if (toolbutton->toolButtonStyle == Qt::ToolButtonTextUnderIcon)
                newSize += QSize(0, 2);
        }
        break;
    default:
        break;
    }
    return newSize;
}

/*!
 \reimp
 */
QRect QtDotNetStyle::subElementRect(SubElement element, const QStyleOption *option, const QWidget *widget) const
{
    QRect rect;
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        rect = QWindowsVistaStyle::subElementRect(element, option, widget);
    else
        rect = QWindowsXPStyle::subElementRect(element, option, widget);
    if (element == SE_DockWidgetCloseButton || element == SE_DockWidgetFloatButton)
        rect.translate(0, 1);
    return rect;
}

/*!
 \reimp
 */
QRect QtDotNetStyle::subControlRect(ComplexControl control, const QStyleOptionComplex *option,
                                      SubControl subControl, const QWidget *widget) const
{
    QRect rect;
    if (control == CC_ComboBox && widget && qobject_cast<QToolBar*>(widget->parentWidget()))
        rect = QWindowsStyle::subControlRect(control, option, subControl, widget);
    else if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        rect = QWindowsVistaStyle::subControlRect(control, option, subControl, widget);
    else
        rect = QWindowsXPStyle::subControlRect(control, option, subControl, widget);

    return rect;
}

/*!
 \reimp
 */
QStyle::SubControl QtDotNetStyle::hitTestComplexControl(ComplexControl control, const QStyleOptionComplex *option,
                                                          const QPoint &pos, const QWidget *widget) const
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        return QWindowsVistaStyle::hitTestComplexControl(control, option, pos, widget);
    else
        return QWindowsXPStyle::hitTestComplexControl(control, option, pos, widget);
}

/*!
 \reimp
 */
int QtDotNetStyle::pixelMetric(PixelMetric metric, const QStyleOption *option, const QWidget *widget) const
{
    int retval = 0;

    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        retval = QWindowsVistaStyle::pixelMetric(metric, option, widget);
    else
        retval = QWindowsXPStyle::pixelMetric(metric, option, widget);

    switch (metric) {
    case PM_ToolBarItemSpacing:
        retval = 2;
        break;
    case PM_ToolBarIconSize:
        retval = 16;
        break;
    case PM_MenuHMargin:
        retval = 1;
        break;
    case PM_MenuVMargin:
        retval = 1;
        break;
    case PM_MenuBarPanelWidth:
        retval = 0;
        break;
    case PM_MenuPanelWidth:
        retval = 1;
        break;
    case PM_DockWidgetTitleBarButtonMargin:
        retval = 6;
        break;
    case PM_DefaultFrameWidth:
    case PM_SpinBoxFrameWidth:
        if (widget && qobject_cast<QToolBar*>(widget->parentWidget()))
            retval = 2;
        break;
    case PM_ButtonShiftVertical:
    case PM_ButtonShiftHorizontal:
        if (widget && qobject_cast<QToolBar*>(widget->parentWidget()))
            retval = 0;
        break;
    default:
        break;
    }
    return retval;
}

/*!
 \reimp
 */
QPalette QtDotNetStyle::standardPalette() const
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        return QWindowsVistaStyle::standardPalette();
    else
        return QWindowsXPStyle::standardPalette();
}


void QtDotNetStylePrivate::updatePalette()
{
    //Detect color palette
    const int maxlength = 256;
    WCHAR themeFileName[maxlength];
    WCHAR themeColor[maxlength];
    internalPalette = Classic;
    if (pIsAppThemed && pIsAppThemed() && pGetCurrentThemeName(themeFileName, maxlength, themeColor, maxlength, NULL, 0) == S_OK) {
        QString name = QString::fromWCharArray(themeFileName);
        QString color = QString::fromWCharArray(themeColor);
        if (theme == QtDotNetStyle::Standard) {
            if (name.endsWith("Luna.msstyles")) {
                if (color == "Metallic")
                    internalPalette = Silver;
                else
                    internalPalette = System;
            }
        } else { //Office style
            if (name.endsWith("Luna.msstyles")) {
                if (color == "HomeStead")
                    internalPalette = Olive;
                else if (color == "Metallic")
                    internalPalette = Silver;
                else
                    internalPalette = Blue;
            } else if (name.endsWith("Aero.msstyles")) {
                    internalPalette = Blue;
            }
        }
    }

    switch (internalPalette) {
    case Blue:
        menuSeparatorColor = QColor(106, 140, 203);
        menuBackgroundColor = QColor(246, 246, 246);
        menuFrameColor = QColor(0, 45, 150);
        break;
    case Silver:
        menuSeparatorColor = QColor(110, 109, 143);
        menuBackgroundColor = QColor(254, 250, 255);
        menuFrameColor = QColor(124, 124, 148);
        break;
    case Olive:
        menuSeparatorColor = QColor(96, 128, 88);
        menuBackgroundColor = QColor(244, 244, 238);
        menuFrameColor = QColor(117, 141, 94);
        break;
    default:
        menuFrameColor = currentPal.dark().color().lighter(108);
        menuSeparatorColor = currentPal.background().color().darker(110);
        menuBackgroundColor = currentPal.light().color();
        menuBackgroundColor.setHsv(
            menuBackgroundColor.hue(),
            3 * menuBackgroundColor.saturation()/8 ,
            250);
        break;
    }
}

/*!
 \reimp
 */
void QtDotNetStyle::polish(QApplication *app)
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        QWindowsVistaStyle::polish(app);
    else
        QWindowsXPStyle::polish(app);
    d->currentPal = app->palette();
    d->updatePalette();
}

/*!
 \reimp
 */
void QtDotNetStyle::unpolish(QApplication *app)
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        QWindowsVistaStyle::unpolish(app);
    else
        QWindowsXPStyle::unpolish(app);
}

// Work around a missing update in QToolbar (see task 192274)
/*!
 \reimp
 */
bool QtDotNetStyle::eventFilter(QObject *watched, QEvent *event)
{
    switch (event->type()) {
    case QEvent::Resize:
        if (QComboBox *combo = qobject_cast<QComboBox*>(watched)) {
            if (combo->sizeAdjustPolicy() == QComboBox::AdjustToContents) {
                if (QToolBar* bar = qobject_cast<QToolBar*>(watched->parent()))
                    bar->update();
            }
        }
        break;
    default:
        break;
    }
    return QObject::eventFilter(watched, event);
}

/*!
 \reimp
 */
void QtDotNetStyle::polish(QWidget *widget)
{
    if (qobject_cast<QMainWindow *>(widget))
        widget->setAttribute(Qt::WA_StyledBackground);
    else if (qobject_cast<QLineEdit*>(widget))
        widget->setAttribute(Qt::WA_Hover);
    else if (qobject_cast<QComboBox*>(widget)) {
        widget->setAttribute(Qt::WA_Hover);
        widget->installEventFilter(this); // see task 192274
    }
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        QWindowsVistaStyle::polish(widget);
    else
        QWindowsXPStyle::polish(widget);
}

/*!
 \reimp
 */
void QtDotNetStyle::unpolish(QWidget *widget)
{
    if (qobject_cast<QMainWindow *>(widget))
        widget->setAttribute(Qt::WA_StyledBackground, false);
    else if (qobject_cast<QLineEdit*>(widget))
        widget->setAttribute(Qt::WA_Hover, false);
    else if (qobject_cast<QComboBox*>(widget)) {
        widget->setAttribute(Qt::WA_Hover, false);
        widget->removeEventFilter(this);
    }

    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        QWindowsVistaStyle::unpolish(widget);
    else
        QWindowsXPStyle::unpolish(widget);
}

/*!
 \reimp
 */
void QtDotNetStyle::polish(QPalette &pal)
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        QWindowsVistaStyle::polish(pal);
    else
        QWindowsXPStyle::polish(pal);
}

/*!
 \reimp
 */
QPixmap QtDotNetStyle::standardPixmap(StandardPixmap standardPixmap, const QStyleOption *opt,
                                      const QWidget *widget) const
{
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        return QWindowsVistaStyle::standardPixmap(standardPixmap, opt, widget);
    else
        return QWindowsXPStyle::standardPixmap(standardPixmap, opt, widget);
}

/*!
  \reimp
*/
int QtDotNetStyle::styleHint(StyleHint hint, const QStyleOption *option, const QWidget *widget,
                               QStyleHintReturn *returnData) const
{
    int ret = 0;
    if (QSysInfo::WindowsVersion >= QSysInfo::WV_VISTA && QSysInfo::WindowsVersion < QSysInfo::WV_NT_based)
        ret =  QWindowsVistaStyle::styleHint(hint, option, widget, returnData);
    else
        ret = QWindowsXPStyle::styleHint(hint, option, widget, returnData);
    return ret;
}
