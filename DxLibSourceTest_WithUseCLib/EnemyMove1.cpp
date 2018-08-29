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
EnemyMove1::EnemyMove1(int collStageHandle, float areaX, float areaZ, float color) : BasicActor(collStageHandle)
{
	// ���f���̌����ƈʒu
	this->area = VGet(areaX, 40.0f, areaZ);

	// �}�e���A��
	material.Diffuse = GetColorF(0.0f, 0.0f, 1.0f, 0.0f);
	material.Specular = GetColorF(0.0f, 0.0f, 0.0f, 0.0f);
	material.Ambient = GetColorF(1.0f, 1.0f, 1.0f, 0.0f);
	material.Emissive = GetColorF(color, color, 1.0f, 0.0f);
	material.Power = 10.0f;

	// ���f���̊�{���
	modelHeight = 10.0f;
	modelWigth = 15.0f;

	// �����̉e�Ɋւ���
	shadowHeight = 5.0f;
	shadowSize = 15.0f;

	upNow = true;
	flyMove = 0.0f;

	viewNow = true;
}

// �f�X�g���N�^
EnemyMove1::~EnemyMove1()
{

}


// �`��
void EnemyMove1::Draw()
{
	if (viewNow)
	{
		SetMaterialParam(material);
		// �y�o�b�t�@��L���ɂ���
		SetUseZBuffer3D(TRUE);
		// �y�o�b�t�@�ւ̏������݂�L���ɂ���
		SetWriteZBuffer3D(TRUE);
		DrawSphere3D(VAdd(area, VGet(0.0f, 60.0f, 0.0f)), modelWigth, 16, GetColor(68, 178, 227), GetColor(255, 255, 255), TRUE);
		// �y�o�b�t�@�ւ̏������݂�L���ɂ���
		SetWriteZBuffer3D(FALSE);
		// �y�o�b�t�@��L���ɂ���
		SetUseZBuffer3D(FALSE);

#ifdef _MODEL_DEBUG
		VECTOR viewArea = VAdd(area, VGet(0.0f, 60.0f, 0.0f));		// ���f���̏���Y���W�������Ă���̂Œ���

		DrawSphere3D(VAdd(area, VGet(0.0f, 60.0f, 0.0f)), modelWigth + 3, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);
#endif // _MODEL_DEBUG
	}
}

// ���C���v���Z�X
void EnemyMove1::Process()
{
	if (viewNow)
	{
		// �����̃v���Z�X
		MoveProcess();
	}
}

void EnemyMove1::SetViewNow(bool viewNow)
{
	this->viewNow = viewNow;
}
