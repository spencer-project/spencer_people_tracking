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

//////////////////////////////////////////////////////////////////////
// Changes - (Omid):
//      Default constructor     ->  Removed
//      SetMask                 ->  Removed
//      Binarize                ->  Removed
//      InitConfig              ->  Removed
//      Clear                   ->  Removed
//
//      Process     ->  Changed (removing clear and binarize calls)
//      Destructor  ->  Changed (don't delete m_MaskArray)
//////////////////////////////////////////////////////////////////////

// ConnectedComponentLabeler.cpp: implementation of the CConnectedComponentLabeler class.
//
//////////////////////////////////////////////////////////////////////


#include "KConnectedComponentLabeler.h"
#include <iostream>
#include <string>
#include <cstdlib>

//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

KConnectedComponentLabeler::KConnectedComponentLabeler(int nAreaThreshold, int *mask, int width, int height)
{
    m_MaskArray = mask;
    m_height = height;
    m_width  = width;
    m_nAreaThreshold = nAreaThreshold;
    m_ObjectNumber = 0;
}

KConnectedComponentLabeler::~KConnectedComponentLabeler()
{
    m_Components.clear();
}

int* KConnectedComponentLabeler::GetOutput()
{
    return m_MaskArray;
}

void KConnectedComponentLabeler::Process()
{
	if(m_MaskArray == NULL) return;
	
	KLinkedList eqTable;

	KNode * tmp = NULL;
	KNode * p	= NULL;
	KNode * p2	= NULL;

	int i;
	int	label = 2;
	int	index = 1;
	int	north = 0;
	int	west  = 0;
	int	nWest = 0;
	int	nEast = 0;
	
	int		 * regionLabel = NULL;
	int		 * lookUpTable = NULL;
	long int * regionArea  = NULL;
	
	int	regionNumber;

	int	maxArea;
	int	maxIndex;
	
	int	data=0;

	for(i=0; i<m_height; i++)
	{
		for(int j=0; j<m_width; j++)
		{			
			index = i*m_width+j;
			
			if(m_MaskArray[index] == 1)
			{
				north = 0;
				west  = 0;
				nWest = 0;
				nEast = 0;

				if( i==0 && j != 0  )
				{
                    west = m_MaskArray[index-1];
				}
				else if( i!=0 && j == 0 )
				{
					north = m_MaskArray[index-m_width];
					nEast = m_MaskArray[index-m_width+1];
				}
				else if( i!= 0 && j != 0 )
				{
					north = m_MaskArray[index-m_width];
					west  = m_MaskArray[index-1];
					nWest = m_MaskArray[index-m_width-1];

					if( j != m_width-1 )
						nEast= m_MaskArray[index-m_width+1];
				}

				// after finding the neighbour labels
				if ( west > 1 ) {

					m_MaskArray[index] = west;

					if( nWest>1 )
						eqTable.InsertData(west,nWest);

					if( north>1 && nWest<=1 )
						eqTable.InsertData(west,north);

					if( nEast>1 && north<=1 )
						eqTable.InsertData(west,nEast);
				
				}else if( nWest > 1) {
					
					m_MaskArray[index] = nWest;

					if ( north<=1 && nEast>1 )
						eqTable.InsertData(nWest,nEast);
				}else if( north > 1 ) {
					m_MaskArray[index] = north;
				}else if( nEast > 1 ) 
					m_MaskArray[index] = nEast;
				else {
					m_MaskArray[index] = label;
					eqTable.InsertData(label);
					label++;
				}
			}
		}
	}

	regionNumber = eqTable.regionCount;

	if( regionNumber > 0 ) 
	{
		regionLabel  = new int[regionNumber];
		regionArea   = new long int[label];

		for(i=0; i<label; i++ )
			regionArea[i]=0;

		tmp = eqTable.header;
		i=0;
		do
		{
			regionLabel[i]=tmp->data;
			tmp=tmp->ngNext;
			i=i+1;
		}
		while(tmp!=NULL);

		lookUpTable = new int[label];
	
		p  = eqTable.header;
		p2 = p;
		
		do
		{   
			data=p->data;
			do
			{   
				lookUpTable[p2->data] = data;
				p2 = p2->sgNext;
			}
			while(p2 != NULL);
			p  = p->ngNext;
			p2 = p;
		}
		while(p != NULL);		

		for (i=0;i<m_height; i++  ) {
			for (int j=0; j<m_width; j++ )
			{
				index=i*m_width+j;

				if( m_MaskArray[index]>1 ) 
				{
					data=lookUpTable[ m_MaskArray[index] ];
					m_MaskArray[index]=data;

					regionArea[ data ]++;
				}
				else
					m_MaskArray[index]=0;
			}
		}

		maxArea = regionArea[0];
		maxIndex=0;
		for(i=1; i<label; i++)
		{
			if(regionArea[i]>maxArea)
			{
				maxIndex = i;
				maxArea = regionArea[i];
			}
		}

		int* trueLabelArray = new int[label];
		for(i=0; i<label; i++)
		{
			trueLabelArray[i] = 0;

			KBox newComponent;
			newComponent.ID = i+1;
                        newComponent.bottomRight_x = RAND_MAX;
                        newComponent.bottomRight_y = RAND_MAX;
                        newComponent.topLeft_x     = -RAND_MAX;
                        newComponent.topLeft_y     = -RAND_MAX;
			m_Components.push_back(newComponent);
		}

		m_ObjectNumber = 0;
		
		int nImSize = m_height*m_width;
		int x, y;

		for(i=0; i<nImSize; i++)
		{
			x = i%m_width;
			y = m_height-i/m_width-1;

			if( regionArea[ m_MaskArray[i] ] < m_nAreaThreshold ) 
				m_MaskArray[i] = 0;
			else
			{ 
				if( trueLabelArray[ m_MaskArray[i] ] == 0 )
				{
					m_ObjectNumber++;

					m_Components[ m_ObjectNumber-1 ].topLeft_x     = x;
					m_Components[ m_ObjectNumber-1 ].topLeft_y     = y;
					m_Components[ m_ObjectNumber-1 ].bottomRight_x = x;
					m_Components[ m_ObjectNumber-1 ].bottomRight_y = y;

					trueLabelArray[ m_MaskArray[i] ] = m_ObjectNumber;
					m_MaskArray[i] = m_ObjectNumber;
				}
				else
				{
					m_MaskArray[i] = trueLabelArray[ m_MaskArray[i] ];

					if( x > m_Components[ m_MaskArray[i]-1 ].bottomRight_x )
						m_Components[ m_MaskArray[i]-1 ].bottomRight_x = x;

					if( x < m_Components[ m_MaskArray[i]-1 ].topLeft_x     )
						m_Components[ m_MaskArray[i]-1 ].topLeft_x     = x;

					if( y > m_Components[ m_MaskArray[i]-1 ].bottomRight_y )
						m_Components[ m_MaskArray[i]-1 ].bottomRight_y = y;
					
					if( y < m_Components[ m_MaskArray[i]-1 ].topLeft_y     )
						m_Components[ m_MaskArray[i]-1 ].topLeft_y     = y;
				}
			}
		}

        while( m_Components.size() != m_ObjectNumber )
            m_Components.pop_back();


		delete trueLabelArray; trueLabelArray = NULL;
	}
	
	delete []lookUpTable;
	lookUpTable = NULL;

	delete []regionArea;
	regionArea = NULL;

	delete []regionLabel;
	regionLabel = NULL;
}

//////// private KNode implementations ////////////////////

KConnectedComponentLabeler::KNode::KNode()
{
	this->data=0;
	this->sgNext=NULL;
	this->ngNext=NULL;
}

KConnectedComponentLabeler::KNode::~KNode()
{

}


KBox::KBox()
{
	ID = 0;
	bottomRight_x = 0;
	bottomRight_y = 0;
	topLeft_x = 0;
	topLeft_y = 0;
}

KBox::~KBox()
{
}

///////////////////////////////////////////////////////////

///////// private KLinkedList implementations ///////////////////////////////////////////////

KConnectedComponentLabeler::KLinkedList::KLinkedList()
{
	this->header = NULL;
	this->regionCount = 0;
}

KConnectedComponentLabeler::KLinkedList::~KLinkedList()
{
	KNode* ptr1 = header;
	KNode* ptr2 = header;
	KNode* ptr3 = header;
	
	if( header != NULL ) {
		do 
		{
			do 
			{
				if (ptr2->sgNext != NULL){
					ptr3 = ptr2;
					ptr2 = ptr2->sgNext;
				} else if( ptr1->sgNext != NULL ) {
					delete ptr2;
					if( ptr3 != NULL )
						ptr3->sgNext=NULL;
					ptr2 = ptr1;
					ptr3 = ptr1;
				}
			}
			while(ptr1->sgNext !=NULL);
			
			ptr1=ptr1->ngNext;
			delete ptr2;
			ptr2=ptr1;
			ptr3=ptr1;
		}
		while(ptr1!=NULL);
	}
}

void KConnectedComponentLabeler::KLinkedList::InsertData(int data)
{
	KNode *	ptrTemp = new KNode;
	ptrTemp->data=data;
	ptrTemp->ngNext=header;
	header=ptrTemp;
	regionCount++;
}

void KConnectedComponentLabeler::KLinkedList::InsertData(int addGroup, int searchGroup)
{
	if ( addGroup != searchGroup ) {
		
		KNode* tmp1 = header;
		KNode* ptrAdd ;
		KNode* ptrSearch ; 

		Search(addGroup,ptrAdd);
		Search(searchGroup,ptrSearch);

		if ( (ptrSearch != NULL) && (ptrAdd != NULL) && (ptrSearch!=ptrAdd) ) {
			
			if ( ptrSearch != header ) {
				
				while( tmp1->ngNext != ptrSearch )
					tmp1=tmp1->ngNext;
				
				tmp1->ngNext=ptrSearch->ngNext;
			}
			else{
				header=ptrSearch->ngNext;
			}
			
			while( ptrAdd->sgNext != NULL )
				ptrAdd=ptrAdd->sgNext;
			
			ptrAdd->sgNext=ptrSearch;	
		}
	}

}



void KConnectedComponentLabeler:: KLinkedList::Search(int data, KConnectedComponentLabeler::KNode* &p )
{
	KNode* ptr1 = header;
	KNode* ptr2 = header;
	
	do 
	{
		do 
		{
			if (ptr2->data==data){
				p=ptr1;
				return;
			}
			ptr2=ptr2->sgNext;
		}
		while(ptr2!=NULL);
		
		ptr1=ptr1->ngNext;
		ptr2=ptr1;
	}
	while(ptr1!=NULL);
	
	p=ptr1;
}


/////////////////////////////////////////////////////////////////////////////////////////////
