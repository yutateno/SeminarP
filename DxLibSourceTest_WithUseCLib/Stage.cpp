#include "Stage.hpp"

Stage::Stage()
{
	// �X�e�[�W�̓ǂݍ���
	LoadFile::MyLoad("media\\TESTROOM1\\ROOM1_Graph.fyn", drawStageHandle, ELOADFILE::mv1model);
	LoadFile::MyLoad("media\\TESTROOM1\\ROOM1_hantei.fyn", collStageHandle, ELOADFILE::mv1model);

	// ���W���w��
	MV1SetPosition(drawStageHandle, VGet(0, 0, 0));
}

Stage::~Stage()
{
	MV1DeleteModel(drawStageHandle);
}

void Stage::Draw()
{
	MV1DrawModel(drawStageHandle);
}

int Stage::GetCollStageHandle()
{
	return collStageHandle;
}
