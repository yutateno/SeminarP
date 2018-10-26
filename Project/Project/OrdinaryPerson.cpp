#include "OrdinaryPerson.hpp"


// �����̃v���Z�X
void OrdinaryPerson::MoveProcess()
{
	std::random_device rnd;     // �񌈒�I�ȗ���������𐶐�
	std::mt19937 mt(rnd());     // �����Z���k�E�c�C�X�^��32�r�b�g��
	std::uniform_int_distribution<> randInX(0, 200);			// X���W�p����
	std::uniform_int_distribution<> moveTurn(0, 314);				// Z���W�p����

	moveCount++;

	// �X���[�Y�ɓ�������
	if (moveFlag)
	{
		animSpeed = 0.75f;
		if (direXAngle == 0.0f)
		{
			if (walkSpeed < 6.0f)
			{
				walkSpeed += 2.5f;
			}
			else
			{
				walkSpeed = 6.0f;
			}
		}
		else	// �΂ߕ���
		{
			if (walkSpeed < 3.0f)
			{
				walkSpeed += 1.0f;
			}
			else
			{
				walkSpeed = 3.0f;
			}
		}
	}
	else
	{
		animSpeed = 0.5f;
		if (walkSpeed > 0.0f)
		{
			walkSpeed -= 3.0f;
		}
		else
		{
			walkSpeed = 0.0f;
		}
	}


	if (moveCount >= 400)
	{
		moveCount = 0;
	}
	else if (moveCount == 100)
	{
		nextDireZAngle = moveTurn(mt) / 100.0f;
		nextDireXAngle = randInX(mt) / 100.0f;
		if (nextDireZAngle != 0.0f)
		{
			nextDireXAngle = -nextDireXAngle;
		}
	}

	area.x += sinf(angle + direXAngle + direZAngle) * -walkSpeed;
	area.z += cosf(angle + direXAngle + direZAngle) * -walkSpeed;
	moveFlag = true;
	Player_PlayAnim(MOTION::walk);

	if (nextDireXAngle != direXAngle)
	{
		if (direXAngle > nextDireXAngle)
		{
			direXAngle -= 0.01f;
		}
		else
		{
			direXAngle += 0.01f;
		}
	}
	if (nextDireZAngle != direZAngle)
	{
		if (direZAngle > nextDireZAngle)
		{
			direZAngle -= 0.01f;
		}
		else
		{
			direZAngle += 0.01f;
		}
	}
}


OrdinaryPerson::OrdinaryPerson(const int modelHandle, const int collStageHandle, const VECTOR area) : BasicCreature(collStageHandle)
{
	// �R�c���f���̓ǂݍ���
	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(modelHandle);


	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = MOTION::idle;
	attachMotion = MV1AttachAnim(this->modelHandle, attachNum, -1, FALSE);


	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	totalTime = MV1GetAttachAnimTotalTime(this->modelHandle, attachMotion);


	// ���f���̊�{���
	modelHeight = 160.0f;
	modelWigth = 50.0f;


	// ���f���̌����ƈʒu
	this->area = VGet(1000.0f, 0.0f, 1000.0f);
	preArea = this->area;
	direXAngle = 0.0f;
	direZAngle = 0.0f;
	nextDireZAngle = 0.0f;
	nextDireXAngle = 0.0f;


	// �����̉e�Ɋւ���
	shadowHeight = 35.0f;
	shadowSize = 50.0f;


	// ���ꂼ��̑��x
	walkSpeed = 0.0f;
	animSpeed = 0.5f;


	moveCount = 0;


	// ���f���̍��W���X�V
	MV1SetPosition(this->modelHandle, this->area);
}


OrdinaryPerson::~OrdinaryPerson()
{
	MODEL_RELEASE(modelHandle);
}


// ���C���v���Z�X
void OrdinaryPerson::Process()
{
	preArea = area;		// ���O�̍��W

	// �����̃v���Z�X
	MoveProcess();

	// ���[�V�����̎���
	Player_AnimProcess();

	// �X�e�[�W�̂����蔻��
	StageHit();

	// �������̉�]�p�x���Z�b�g
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, direXAngle + direZAngle, 0.0f));
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(modelHandle, area);
}


// �`��
void OrdinaryPerson::Draw()
{
	BasicObject::Draw();		// ��{�I�Ȃ��̂����������Ă���

	BasicObject::ShadowFoot();

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // _MODEL_DEBUG
#ifdef _DEBUG
	//printfDx("X : %f\tZ : %f\tNX : %f\tNZ : %f\n", direXAngle, direZAngle, nextDireXAngle, nextDireZAngle);
#endif // _DEBUG
}