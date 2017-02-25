
#include <cstring>
#include <vector>
inline int CoordinateToIndex(const int nX, const int nY, const int nWidth)
{
	return nX + nY * nWidth;
}

inline void IndexToCoordinate(const int nIndex, const int nWidth, int& nOutX, int& nOutY)
{
	nOutX = nIndex % nWidth;
	nOutY = nIndex / nWidth;
}

int BuildPath(const int* Previous, const int nStartIndex, const int nTargetIndex, int* pOutBuffer, const int nOutBufferSize)
{
	int* pOutEnd = pOutBuffer + nOutBufferSize;
	int* pOutIterator = pOutBuffer;
	int nCurrent = nStartIndex;
	int nSteps = 0;
	while (nCurrent != nTargetIndex)
	{
		nCurrent = Previous[nCurrent];
		++nSteps;
		if (pOutIterator < pOutEnd)
		{
			*pOutIterator = nCurrent;
			++pOutIterator;
		}
	}
	return nSteps;
}

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
	int* pMemoryBuffer = new int[nTotalMapSize * 2];
	int* Previous = pMemoryBuffer;
	int* Queue = pMemoryBuffer + nTotalMapSize;
	unsigned char* pWorkingMap = new unsigned char[nTotalMapSize];
	std::memcpy(pWorkingMap, pMap, nTotalMapSize * sizeof(unsigned char));

	// Pathfinding from target to start to avoid the need to reverse the out path
	Previous[nTargetIndex] = nTargetIndex;
	int* pQueueBack = Queue;
	int* pQueueFront = Queue;
	*pQueueBack = nTargetIndex;
	++pQueueBack;
	int XDirections[] = { -1, 1, 0, 0 };
	int YDirections[] = { 0, 0, -1, 1 };

	while (pQueueFront < pQueueBack)
	{
		const int nCurrent = *pQueueFront;
		++pQueueFront;
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
			const int nNeighborIndex = CoordinateToIndex(nNeighborX, nNeighborY, nMapWidth);
			if (pWorkingMap[nNeighborIndex] == 1 ) // Not visited or in queue
			{
				pWorkingMap[nNeighborIndex] = 0;
				Previous[nNeighborIndex] = nCurrent;
				*pQueueBack = nNeighborIndex;
				++pQueueBack;
				if (nNeighborIndex == nStartIndex)	// Did we find our target?
				{
					int nOut = BuildPath(Previous, nStartIndex, nTargetIndex, pOutBuffer, nOutBufferSize);
					delete[] pMemoryBuffer;
					delete[] pWorkingMap;
					return nOut;
				}
			}
		}
	}

	// Queue empty and path not found
	delete[] pWorkingMap;
	delete[] pMemoryBuffer;
	return -1;
}