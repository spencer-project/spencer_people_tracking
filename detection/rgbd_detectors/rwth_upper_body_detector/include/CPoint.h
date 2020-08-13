//************************************************************************
//   T h e   O p e n   F o u n d a t i o n   C l a s s e s
// ------------------------------------------------------------------------
//   Filename   : CPoint.h
//   Author(s)  : Carsten Breuer

// --[ History ] ----------------------------------------------------------

//cb  = Carsten Breuer     (Carsten.Breuer@breuer-software.de)
//gv  = Geurt Vos          (geurt@users.sourceforge.net)
//id  = Ivan Deras         (ideras@users.sourceforge.net)
//tm  = Tim Musschoot      (timussch@users.sourceforge.net)
//wdh = William D. Herndon (shadowdog@users.sourceforge.net)

//mm-dd-yy  ver   who  what
//??-??-00  0.10  cb   Created.
//10-05-03  0.10  wdh  Added this history, changed to use windef.h
//                     instead of OfcTypes.h.

// ------------------------------------------------------------------------
// Copyright (c) 2000-03 by The Open Foundation Classes
// Copyright (c) 2000 by Carsten Breuer
//************************************************************************

#ifndef CPOINT_H
#define CPOINT_H

class CSize;
class CRect;

class CPoint
{
   public:
      CPoint ();
      CPoint (int x, int y); // OK, this is an enhancement
      CPoint (const CPoint& pt);

      void Offset (int X, int Y);
      void Offset (CPoint point);

      // Operators:
      bool    operator == (CPoint& Point) const;
      bool    operator != (CPoint& Point) const;
      void    operator += (CPoint  Point);

      void    operator -= (CPoint  Point);

      CPoint  operator +  (CPoint  Point) const;


      CPoint  operator -  (CPoint  Point) const;

      CPoint  operator -  () const;

      // Without this, gcc creates code which crashes on POINT assignment
      const CPoint& operator = (CPoint Point);
      const CPoint& operator = (int v);

      int x;
      int y;

private:


};

#endif // CPOINT_H

