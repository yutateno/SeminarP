#include "Stage.hpp"

Stage::Stage()
{
	// �X�e�[�W�̓ǂݍ���
	stageHandle = MV1LoadModel("media\\TESTROOM1\\ROOM1_Graph.fbx");

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
