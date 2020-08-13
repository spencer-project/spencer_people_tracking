/*********************************************************************************
This piece of program contained can be used, copied, modified, merged, published, 
and/or have copies distributed for academic or research purposes only without 
restriction under the following conditions:

1. The above header and this permission notice shall be included in all copies or 
substantial portions of the program

2. The software is provided "as is", without warranty of any kind, express or implied, 
including but not limited to the warranties of merchantability, fitness for a particular 
purpose and non-infringement. In no event shall the author(s) be liable for any claim, 
damages or liability, whether in an action of contract, tort or otherwise, arising from, 
out of or in connection with this program.

3. If you use this piece of code for research purposes, refer to 

Tola, Engin. 2006 June 12. Homepage. <http://cvlab.epfl.ch/~tola/index.htm>

4. An acknowledgement note should be included as: 

     "The software used here was originally created by Tola, Engin. 2006 June 12. 
	 Homepage. <http://cvlab.epfl.ch/~tola/index.htm>"

**************************************************************************************/

// ConnectedComponentLabeler.h: interface for the CConnectedComponentLabeler class.
//
//////////////////////////////////////////////////////////////////////

#if !defined(AFX_CONNECTEDCOMPONENTLABELER_H__E4CEAEC2_ABC6_4A4E_AE25_68AC93377186__INCLUDED_)
#define AFX_CONNECTEDCOMPONENTLABELER_H__E4CEAEC2_ABC6_4A4E_AE25_68AC93377186__INCLUDED_

#if _MSC_VER > 1000
#pragma once
#endif // _MSC_VER > 1000

#include <string>
#include <vector>
#include "CPoint.h"


using namespace std;
class KBox
{
public:
        KBox();
        virtual ~KBox();
        CPoint topLeft;
        CPoint bottomRight;
        int	   ID;
};

class KConnectedComponentLabeler
{

private:
	class KNode  
	{
	public:
		KNode();
		virtual ~KNode();
		KNode*	ngNext;
		KNode*	sgNext;
		int data;
	};


private:
	class KLinkedList
	{
	public:
		void printTable();

		KNode * header;
		int  regionCount;

		void Search(int data, KNode* &p);

		void InsertData(int data);
		void InsertData(int addGroup,int searchGroup);
		
		KLinkedList();
		~KLinkedList();
	};

public:
	int*	GetOutput();
	
	int		m_ObjectNumber;
    int*	m_MaskArray;
		
	int		m_nAreaThreshold;
	int		m_height;
	int		m_width;

	vector<KBox> m_Components;

	void	Process();
	
    KConnectedComponentLabeler(int nAreaThreshold, int *mask, int width, int height);
	virtual ~KConnectedComponentLabeler();
};

#endif // !defined(AFX_CONNECTEDCOMPONENTLABELER_H__E4CEAEC2_ABC6_4A4E_AE25_68AC93377186__INCLUDED_)
