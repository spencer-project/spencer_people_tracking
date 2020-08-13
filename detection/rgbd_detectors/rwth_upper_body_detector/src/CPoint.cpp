//************************************************************************
//  T h e   O p e n   F o u n d a t i o n    C l a s s e s
// ------------------------------------------------------------------------
//  Filename   : CPoint.CPP
//  Version    : 0.50
//  Author(s)  : Carsten Breuer

// --[ Description ]-------------------------------------------------------

//  This file is part of the OFC - Open Foundation Classes -.
//  It is MFC compatible.
//  Implementation of the class CPoint.

// --[ History ] ----------------------------------------------------------

//mm-dd-yy  who  ver   What
//??-??-??  cb   0.10  Original file by Carsten Breuer. First beta version.
//??-??-??  gv   0.20  Changes & fixes by Geurt Vos
//??-??-??  cb   0.30  Finished (hopefully) by Carsten Breuer
//??-??-??  wdh  0.40  Fixed constructors and Offset(), added operator= due
//    to compiler bug.
//05-30-04  wdh  0.50  Exchanged many includes for one afxwin.h. Changed history
//    to be like other source files.

// --[ Developers ]--------------------------------------------------------

//cb  = Carsten Breuer     (Carsten.Breuer@breuer-software.de)
//gv  = Geurt Vos          (geurt@users.sourceforge.net)
//id  = Ivan Deras         (ideras@users.sourceforge.net)
//tm  = Tim Musschoot      (timussch@users.sourceforge.net)
//wdh = William D. Herndon (shadowdog@users.sourceforge.net)

// --[ How to compile ]----------------------------------------------------

//  This file was developed under DJGPP and Rhide. If you are familiar with
//  Borland C++ 3.1, you will feel like at home ;-)).
//  Both tools are free software. Downloads at: http://www.delorie.com/djgpp


// --[ Where to get help/information ]-------------------------------------

//  The author              : Carsten.Breuer@breuer-software.de

// --[ License ] ----------------------------------------------------------

//  This program is free software; you can redistribute it and/or
//  modify it under the terms of the GNU General Public License
//  as published by the Free Software Foundation; either version 2
//  of the License, or (at your option) any later version.

//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.

//  You should have received a copy of the GNU General Public License
//  along with this program; if not, write to the Free Software
//  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.

// ------------------------------------------------------------------------
//  Copyright (c) 2000-04 The Open Foundation Classes
//  Copyright (c) 2000 Carsten Breuer
//************************************************************************
#include "CPoint.h"

CPoint::CPoint()
{
   x = 0;
   y = 0;
}

CPoint::CPoint(int xi, int yi)
{
   x = xi;
   y = yi;
}

CPoint::CPoint(const CPoint& pt)
{
   x = pt.x;
   y = pt.y;
}

void CPoint::Offset (int X, int Y)
{
   x += X;
   y += Y;
}

void CPoint::Offset (CPoint point)
{
   *this += point;
}

// overloaded operators
bool CPoint::operator == (CPoint& Point) const
{
   if ((x == Point.x) && (y == Point.y))
   {
      return true;
   }
   return false;
}

bool CPoint::operator != (CPoint& Point) const
{
   if ((x != Point.x) || (y != Point.y))
   {
      return true;
   }
   return false;
}

void CPoint::operator += (CPoint Point)
{
   x += Point.x;
   y += Point.y;
}

void CPoint::operator -= (CPoint Point)
{
   x -= Point.x;
   y -= Point.y;
}

CPoint CPoint::operator + (CPoint Point) const
{
   CPoint sPoint(*this);
   sPoint.x += Point.x;
   sPoint.y += Point.y;
   return sPoint;
}


CPoint CPoint::operator - (CPoint Point) const
{
   CPoint sPoint(*this);
   sPoint.x -= Point.x;
   sPoint.y -= Point.y;
   return sPoint;
}


CPoint CPoint::operator - () const
{
   CPoint Point (*this);
   Point.x = 0 - Point.x;
   Point.y = 0 - Point.y;
   return Point;
}

// Without this, gcc creates code which crashes on POINT assignment
const CPoint& CPoint::operator = (CPoint Point)
{
   x = Point.x;
   y = Point.y;
   return( *this );
}

      const CPoint& CPoint::operator = (int v)
      {
   x = v;
   y = v;
   return( *this );
}


