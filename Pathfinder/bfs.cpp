

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

int BuildPath(const std::vector<int>& Previous, const int nStartIndex, const int nTargetIndex, int* pOutBuffer, const int nOutBufferSize)
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
	std::vector<int> Previous;
	Previous.reserve(nTotalMapSize);
	std::vector<int> Queue;
	Queue.reserve(nTotalMapSize);
	// Mark walls directly in Previous buffer
	for (int i = 0; i < nTotalMapSize; ++i)
	{
		if (pMap[i] == 1)
		{
			Previous.push_back( -1 );
		}
		else
		{
			Previous.push_back(-2);
		}
	}
	// Pathfinding from target to start to avoid the need to reverse the out path
	Previous[nTargetIndex] = nTargetIndex;
	Queue.push_back(nTargetIndex);
	unsigned int nQueueFront = 0;
	int XDirections[] = { 0, -1, 1, 0 };
	int YDirections[] = { -1, 0, 0, 1 };

	while (nQueueFront < Queue.size())
	{
		const int nCurrent = Queue[nQueueFront++];
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
			const int nNeighborPrevious = Previous[nNeighborIndex];
			if (Previous[nNeighborIndex] == -1) // Not visited or in queue
			{
				Previous[nNeighborIndex] = nCurrent;
				Queue.push_back(nNeighborIndex);
				if (nNeighborIndex == nStartIndex)	// Did we find our target?
				{
					return BuildPath(Previous, nStartIndex, nTargetIndex, pOutBuffer, nOutBufferSize);
				}
			}
		}
	}

	// Queue empty and path not found
	return -1;
}