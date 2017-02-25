#include "windows.h"

#include "main.h"
#include "bfs.h"
#include <string>
#include <iostream>
#include <fstream>
#include <vector>

int main()
{
	char PrintBuffer[256];
	{
		OutputDebugString("\n--- START ---\n");
		unsigned char pMap[] = { 1, 1, 1, 1, 0, 1, 0, 1, 0, 1, 1, 1 };
		int pOutBuffer[12];
		int nSteps = FindPath(0, 0, 1, 2, pMap, 4, 3, pOutBuffer, 12);
		sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
		OutputDebugString( PrintBuffer );
		for (int i = 0; i < nSteps; ++i)
		{
			sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
			OutputDebugString(PrintBuffer);
		}
		OutputDebugString("\n--- END ---\n");
	}

	{
		OutputDebugString("\n--- START ---\n");
		unsigned char pMap[] = { 0, 0, 1, 0, 1, 1, 1, 0, 1 };
		int pOutBuffer[7];
		int nSteps = FindPath(2, 0, 0, 2, pMap, 3, 3, pOutBuffer, 7);
		sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
		OutputDebugString(PrintBuffer);
		for (int i = 0; i < nSteps; ++i)
		{
			sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
			OutputDebugString(PrintBuffer);
		}
		OutputDebugString("\n--- END ---\n");
	}

	{
		OutputDebugString("\n--- START ---\n");

		using namespace std;

		streampos size;
	
		ifstream file("maze.txt");
		if (file.is_open())
		{
			size = file.tellg();
			std::vector<unsigned char> Map;
			Map.reserve(1024 * 1024);
			while (!file.eof())
			{
				int nGet = file.get();
				if (nGet == '#')
				{
					Map.push_back(0);
				}
				else if (nGet == '.')
				{
					Map.push_back(1);
				}
				else
				{
					// do nothing
				}
			}
			file.close();
			int nMapSize = Map.size();
			if (nMapSize == 983 * 991)
			{
				int* pOutBuffer = new int[10000];
				int nSteps;
				for (int i = 0; i < 50; ++i)
				{
					nSteps = FindPath(900, 902, 210, 310, &Map[0], 983, 991, pOutBuffer, 10000);
					nSteps = FindPath(210, 310, 900, 902, &Map[0], 983, 991, pOutBuffer, 10000);
				}
				sprintf_s(PrintBuffer, "Steps: %d\n", nSteps);
				OutputDebugString(PrintBuffer);
				for (int i = 0; i < nSteps; ++i)
				{
					sprintf_s(PrintBuffer, "%d ", pOutBuffer[i]);
					OutputDebugString(PrintBuffer);
				}
				OutputDebugString("\n--- END ---\n");
				delete[] pOutBuffer;
			}
		}

	}

	const char* pMap = "0000100001"
		"0000100001"
		"0000100001"
		"0000100001"
		"0000100001";

	std::string Out;
	const int nWidth = 10;
	const char* pCurrent = pMap;
	int nLine = nWidth;
	while (*pCurrent)
	{
		if (nLine == 0)
		{
			Out += '\n';
			nLine = nWidth;
		}
		Out += *pCurrent;
		--nLine;
		++pCurrent;
	}
	OutputDebugString("\n--- START ---\n");
	OutputDebugString( Out.c_str() );
	OutputDebugString( "\n--- END ---\n");
	return 0;
}
