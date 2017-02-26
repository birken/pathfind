
#include <cstring>
#include <vector>
#include <stdlib.h>
#include <algorithm>

static inline int CoordinateToIndex(const int nX, const int nY, const int nWidth)
{
	return nX + nY * nWidth;
}

static inline void IndexToCoordinate(const int nIndex, const int nWidth, int& nOutX, int& nOutY)
{
	nOutX = nIndex % nWidth;
	nOutY = nIndex / nWidth;
}

int BuildPathTwoWay( const int nFromStart, const int nFromTarget, const char* Previous, const int nStartIndex, const int nTargetIndex, const int nWidth, const int* XDirections, const int* YDirections, int* pOutBuffer, const int nOutBufferSize)
{
	int* pOutEnd = pOutBuffer + nOutBufferSize;
	int* pOutIterator = pOutBuffer;
	// start from nFromStart and go to nStartIndex
	int nCurrent = nFromStart;
	int nSteps = 0;
	bool bOutBufferOverrun = false;
	while (nCurrent != nStartIndex)
	{
		++nSteps;
		if (pOutIterator < pOutEnd)
		{
			*pOutIterator = nCurrent;
			++pOutIterator;
		}
		else
		{
			bOutBufferOverrun = true;
		}
		int Direction = -( Previous[nCurrent] ) - 2;
		nCurrent -= XDirections[Direction];
		nCurrent -= nWidth * YDirections[Direction];
	}
	// reverse path
	if (!bOutBufferOverrun)
	{
		std::reverse(pOutBuffer, pOutIterator);
	}
	// start from nFromTarget and go to nTargetIndex, append directly to path
	nCurrent = nFromTarget;
	++nSteps;
	if (pOutIterator < pOutEnd)
	{
		*pOutIterator = nCurrent;
		++pOutIterator;
	}
	while (nCurrent != nTargetIndex )
	{
		int Direction = Previous[nCurrent] - 2;
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

int FindPath2W(const int nStartX, const int nStartY,
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
	const int nTotalNeededMemory = (nTotalMapSize) * sizeof(int) + nTotalMapSize * sizeof(unsigned char);
	unsigned char* pMemoryBuffer = (unsigned char*)malloc(nTotalNeededMemory);
	unsigned char* pMemoryBufferStack = pMemoryBuffer;
	int* Queue = (int*)pMemoryBufferStack;
	pMemoryBufferStack += nTotalMapSize * sizeof(int);
	char* pWorkingMap = (char*)pMemoryBufferStack;
	std::memcpy(pWorkingMap, pMap, nTotalMapSize * sizeof(char));

	// Pathfinding from target to start to avoid the need to reverse the out path
	pWorkingMap[nTargetIndex] = 2;
	pWorkingMap[nStartIndex] = -2;
	int* pQueueBack = Queue;
	int* pQueueFront = Queue;
	*pQueueBack++ = nTargetIndex;
	*pQueueBack++ = nStartIndex;
	int XDirections[] = { -1, 1, 0, 0 };
	int YDirections[] = { 0, 0, -1, 1 };
	while (pQueueFront < pQueueBack)
	{
		const int nCurrent = *pQueueFront;
		++pQueueFront;
		int nCurrentX;
		int nCurrentY;
		IndexToCoordinate(nCurrent, nMapWidth, nCurrentX, nCurrentY);

		bool bCurrentFromTarget = pWorkingMap[nCurrent] > 0;
		int nCurrentSign = bCurrentFromTarget ? 1 : -1;
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
			const int nNeighborIndex = CoordinateToIndex(nNeighborX, nNeighborY, nMapWidth);

			char NeighborDirection = pWorkingMap[nNeighborIndex];
			if (NeighborDirection == 1)
			{
				pWorkingMap[nNeighborIndex] = (nDirection + 2) * nCurrentSign;	// Save direction negative direction if search started from start
				*pQueueBack = nNeighborIndex;
				++pQueueBack;
			}
			else if ( NeighborDirection != 0 && (NeighborDirection > 0) != bCurrentFromTarget)
			{
				int nFromStart = bCurrentFromTarget ? nNeighborIndex : nCurrent;
				int nFromTarget = bCurrentFromTarget ? nCurrent : nNeighborIndex;
				// Path found the two searched have met
				int nOut = BuildPathTwoWay(nFromStart, nFromTarget, pWorkingMap, nStartIndex, nTargetIndex, nMapWidth, XDirections, YDirections, pOutBuffer, nOutBufferSize);
				free(pMemoryBuffer);
				return nOut;
			}

		}
	}

	// Queue empty and path not found
	free(pMemoryBuffer);
	return -1;
}