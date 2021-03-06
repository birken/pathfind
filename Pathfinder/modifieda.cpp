
#include <cstring>
#include <vector>
#include <stdlib.h>
#include <algorithm>
#include <memory>

static inline int CoordinateToIndex(const int nX, const int nY, const int nWidth)
{
	return nX + nY * nWidth;
}

static inline void IndexToCoordinate(const int nIndex, const int nWidth, int& nOutX, int& nOutY)
{
	nOutX = nIndex % nWidth;
	nOutY = nIndex / nWidth;
}

int BuildPathMA(const unsigned char* Previous, const int nStartIndex, const int nTargetIndex, const int nWidth, const int* XDirections, const int* YDirections, int* pOutBuffer, const int nOutBufferSize)
{
	int* pOutEnd = pOutBuffer + nOutBufferSize;
	int* pOutIterator = pOutBuffer;
	int nCurrent = nStartIndex;
	int nSteps = 0;
	while (nCurrent != nTargetIndex)
	{
		int Direction = Previous[nCurrent];
		nCurrent -= XDirections[Direction];
		nCurrent -= nWidth * YDirections[Direction];
		++nSteps;
		if (pOutIterator < pOutEnd)
		{
			*pOutIterator = nCurrent;
			++pOutIterator;
		}
	}
	return nSteps;
}
template<typename T>
struct ArrayDeleter
{
	~ArrayDeleter()
	{
		delete[] _pArray;
	}
	T* _pArray = nullptr;
};

static inline unsigned int EstimateCost(const int x0, const int y0, const int x1, const int y1)
{
	return std::abs(x0 - x1) + std::abs(y0 - y1);
}
struct QueueElement
{
	QueueElement(int nIndex, unsigned int nCost, unsigned int nEstimatedTotalCost)
		: _nIndex(nIndex), _nCost(nCost), _nEstimatedTotalCost(nEstimatedTotalCost)
	{

	}
	int _nIndex;
	unsigned int _nCost;
	unsigned int _nEstimatedTotalCost;

	bool operator<(const QueueElement& other) const
	{
		return _nEstimatedTotalCost > other._nEstimatedTotalCost;
	}
};
int FindPath(const int nStartX, const int nStartY,
	const int nTargetX, const int nTargetY,
	const unsigned char* pMap, const int nMapWidth, const int nMapHeight,
	int* pOutBuffer, const int nOutBufferSize)
{

	const int nStartIndex = CoordinateToIndex(nStartX, nStartY, nMapWidth);
	const int nTargetIndex = CoordinateToIndex(nTargetX, nTargetY, nMapWidth);
	if (pMap[nStartIndex] != 1)
	{
		return -1;
	}
	if (pMap[nTargetIndex] != 1)
	{
		return -1;
	}
	if (nStartIndex == nTargetIndex)
	{
		if (nOutBufferSize > 0)
		{
			*pOutBuffer = nTargetIndex;
		}
		return 0;
	}

	const int nTotalMapSize = nMapHeight * nMapWidth;
//	const int nTotalNeededMemory = (nTotalMapSize) * sizeof(QueueElement) + nTotalMapSize * sizeof(unsigned char) + nTotalMapSize * sizeof(int);
//	unsigned char* pMemoryBuffer = (unsigned char*)malloc(nTotalNeededMemory);
//	unsigned char* pMemoryBufferStack = pMemoryBuffer;
	QueueElement* Queue = (QueueElement*)new unsigned char[nTotalMapSize*sizeof(QueueElement)]; // (QueueElement*)pMemoryBufferStack;
	ArrayDeleter<unsigned char> QueueDeleter;
	QueueDeleter._pArray = (unsigned char*)Queue;
																								//	pMemoryBufferStack += nTotalMapSize * sizeof(QueueElement);
	unsigned char* pWorkingMap = new unsigned char[nTotalMapSize]; // pMemoryBufferStack;
	ArrayDeleter<unsigned char> WorkingDeleter;
	WorkingDeleter._pArray = pWorkingMap;

//	pMemoryBufferStack += nTotalMapSize * sizeof(unsigned char);
	unsigned int* pCostMap = new unsigned int[nTotalMapSize]; // (unsigned int*)pMemoryBufferStack;
	ArrayDeleter<unsigned int> CostDeleter;
	CostDeleter._pArray = pCostMap;

	const unsigned char* pMapIt = pMap;
	const unsigned char* pMapEnd = pMap + nTotalMapSize;
	unsigned int* pCostMapIt = pCostMap;
	while (pMapIt != pMapEnd)
	{
		(*pCostMapIt++) = -(*pMapIt++);
	}
	// Pathfinding from target to start to avoid the need to reverse the out path
	pWorkingMap[nTargetIndex] = 0;
	QueueElement* pQueueBack = Queue + (nTotalMapSize / 2) ;
	QueueElement* pQueueFront = pQueueBack;
	*pQueueBack = QueueElement(nTargetIndex, 0, EstimateCost(nStartX, nStartY, nTargetX, nTargetY));
	++pQueueBack;
	int XDirections[] = { -1, 1, 0, 0 };
	int YDirections[] = { 0, 0, -1, 1 };
	int Directions[] = { -1, 1, -nMapWidth, nMapWidth };
	//int nProcessed = 0;
	//int nInQueue = 1;
	//int nMaxQueue = 1;
	while (pQueueFront < pQueueBack)
	{
		//++nProcessed;
		QueueElement Current = *pQueueFront;
		++pQueueFront;
		//--nInQueue;
		int nCurrent = Current._nIndex;

		if (Current._nCost > pCostMap[nCurrent])
			continue;

		unsigned int nCost = Current._nCost + 1;

		int nCurrentX;
		int nCurrentY;
		IndexToCoordinate(nCurrent, nMapWidth, nCurrentX, nCurrentY);

		// Search all four neighbors and add to queue
		for (int nDirection = 0; nDirection < 4; ++nDirection)
		{
			int nNeighborX = nCurrentX + XDirections[nDirection];
			if (nNeighborX < 0 || nNeighborX >= nMapWidth)
			{
				continue;
			}
			int nNeighborY = nCurrentY + YDirections[nDirection];
			if (nNeighborY < 0 || nNeighborY >= nMapHeight)
			{
				continue;
			}
			const int nNeighborIndex = nCurrent + Directions[nDirection];
			//if (pMap[nNeighborIndex] == 1) // not wall
			{
				if (nCost < pCostMap[nNeighborIndex])
				{
					pCostMap[nNeighborIndex] = nCost;
					pWorkingMap[nNeighborIndex] = nDirection;	// Save direction
					unsigned int nEstimatedTotalCost = nCost + EstimateCost(nStartX, nStartY, nNeighborX, nNeighborY);
					if (nEstimatedTotalCost <= Current._nEstimatedTotalCost)
					{
						// Add directly to front
						--pQueueFront;
						*pQueueFront = QueueElement(nNeighborIndex, nCost, nEstimatedTotalCost );
					}
					else
					{
						*pQueueBack = QueueElement(nNeighborIndex, nCost, nEstimatedTotalCost);
						++pQueueBack;
					}
					//++nInQueue;
					//nMaxQueue = std::max(nMaxQueue, nInQueue);
					if (nNeighborIndex == nStartIndex)	// Did we find our target?
					{
						int nOut = BuildPathMA(pWorkingMap, nStartIndex, nTargetIndex, nMapWidth, XDirections, YDirections, pOutBuffer, nOutBufferSize);
						return nOut;
					}
				}
			}
		}
	}

	// Queue empty and path not found
	return -1;
}