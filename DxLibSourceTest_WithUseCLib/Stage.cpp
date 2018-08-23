#include "Stage.hpp"

Stage::Stage()
{
	// ステージの読み込み
	LoadFile::MyLoad("media\\TESTROOM1\\ROOM1_Graph.fyn", stageHandle, ELOADFILE::mv1model);

	// 座標を指定
	MV1SetPosition(stageHandle, VGet(0, 0, 0));
}

Stage::~Stage()
{
	MV1DeleteModel(stageHandle);
}

void Stage::Draw()
{
	MV1DrawModel(stageHandle);
}
