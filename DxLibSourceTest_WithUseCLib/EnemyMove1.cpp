#include "EnemyMove1.hpp"

// �����̃v���Z�X
void EnemyMove1::MoveProcess()
{
	// �㏸������
	if (upNow)
	{
		// �㏸��
		if (60.0f > area.y)
		{
			area.y += flyMove;
		}
		// �㏸����������
		else
		{
			upNow = false;
			flyMove = -0.1f;
		}
	}
	// ���~������
	else
	{
		// ���~��
		if (area.y > 30.0f)
		{
			area.y -= flyMove;
		}
		// ���~����������
		else
		{
			upNow = true;
			flyMove = -0.1f;
		}
	}
	// �n����������肳����
	if (flyMove < 0.5f)
	{
		flyMove += 0.05f;
	}
}


// �R���X�g���N�^
EnemyMove1::EnemyMove1(int collStageHandle, float areaX, float areaZ) : BasicActor(collStageHandle)
{
	// �R�c���f���̓ǂݍ���
	LoadFile::MyLoad("media\\�����\\sphere.fyn", modelHandle, ELOADFILE::fbxmodel);
	
	// ���f���̊�{���
	modelHeight = 10.0f;
	modelWigth = 15.0f;

	// ���f���̌����ƈʒu
	this->area = VGet(areaX, 40.0f, areaZ);

	// �����̉e�Ɋւ���
	shadowHeight = 15.0f;
	shadowSize = 15.0f;

	upNow = true;
	flyMove = 0.0f;
	
	// ���f���̍��W���X�V
	MV1SetPosition(modelHandle, area);
}

// �f�X�g���N�^
EnemyMove1::~EnemyMove1()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}


// �`��
void EnemyMove1::Draw()
{
	BasicActor::Draw();		// ��{�I�Ȃ��̂����������Ă���

#ifdef _MODEL_DEBUG
	VECTOR viewArea = VAdd(area, VGet(0.0f, 60.0f, 0.0f));		// ���f���̏���Y���W�������Ă���̂Œ���

	DrawCapsule3D(viewArea, VAdd(viewArea, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // _MODEL_DEBUG
}

// ���C���v���Z�X
void EnemyMove1::Process()
{
	MV1SetMaterialDifColor(modelHandle, 0, GetColorF(1.0f, 1.0f, 1.0f, 1.0f));
	MV1SetMaterialEmiColor(modelHandle, 0, GetColorF(1.0f, 1.0f, 1.0f, 0.0f));

	// �����̃v���Z�X
	MoveProcess();

	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(modelHandle, area);
}