#include "LoadScreen.hpp"

LoadScreen::LoadScreen()
{
	draw = 0;
	LoadFile::MyLoad("media\\load.pyn", draw, ELOADFILE::graph);
	endDraw = 0;
	LoadFile::MyLoad("media\\ÉçÅ[ÉhèIóπ.pyn", endDraw, ELOADFILE::graph);
}

LoadScreen::~LoadScreen()
{
	DeleteGraph(draw);
}

void LoadScreen::Process(int num, int max)
{
	if (num < max)
	{
		DrawGraph(0, 0, draw, false);
	}
	else
	{
		DrawGraph(0, 0, endDraw, false);
	}
}
