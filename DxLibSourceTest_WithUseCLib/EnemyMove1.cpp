#include "EnemyMove1.hpp"

EnemyMove1::EnemyMove1(int collStageHandle) : BasicActor(collStageHandle)
{
	// �R�c���f���̓ǂݍ���
	LoadFile::MyLoad("media\\�����\\sphere_jamp.fyn", modelHandle, ELOADFILE::mv1model);

	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = 0;
	attachMotion = MV1AttachAnim(modelHandle, attachNum, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	totalTime = MV1GetAttachAnimTotalTime(modelHandle, attachMotion);

	// ���f���̊�{���
	modelHeight = 80.0f;
	modelWigth = 15.0f;

	// ���f���̌����ƈʒu
	area = VGet(400.0f, 0.0f, 0.0f);
	preArea = area;

	// �����̉e�Ɋւ���
	shadowHeight = 5.0f;
	shadowSize = 15.0f;

	// ���ꂼ��̑��x
	//walkSpeed = 0.0f;
	animSpeed = 1.0f;

	// ���f���̍��W���X�V
	MV1SetPosition(modelHandle, area);
}

EnemyMove1::~EnemyMove1()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}

void EnemyMove1::Draw()
{
	BasicActor::Draw();

	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
}

void EnemyMove1::Process()
{
	// ���[�V�����̎���
	Player_AnimProcess();

	// �X�e�[�W�̂����蔻��
	StageHit();

	Player_PlayAnim(0);
	
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(modelHandle, area);
}