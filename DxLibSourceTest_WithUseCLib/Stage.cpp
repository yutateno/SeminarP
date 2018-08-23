#include "Stage.hpp"

Stage::Stage()
{
	// �X�e�[�W�̓ǂݍ���
	LoadFile::MyLoad("media\\TESTROOM1\\ROOM1_Graph.fyn", stageHandle, ELOADFILE::mv1model);

	// ���W���w��
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
