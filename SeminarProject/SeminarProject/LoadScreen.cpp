#include "LoadScreen.hpp"

LoadScreen::LoadScreen()
{
	draw = 0;
	LoadFile::MyLoad("media\\load.pyn", draw, ELOADFILE::graph);
}

LoadScreen::~LoadScreen()
{
	DeleteGraph(draw);
}

void LoadScreen::Process(int num, int max)
{
	DrawGraph(0, 0, draw, false);
}
