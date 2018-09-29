#include "Character.hpp"

using namespace MY_XINPUT;


// �����̃v���Z�X
void Character::MoveProcess(unsigned __int8 controllNumber)
{
	// �X���[�Y�ɓ�������
	if (moveFlag)
	{
		animSpeed = 1.0f;
		if (direXAngle == 0.0f)
		{
			if (walkSpeed < 20.0f)
			{
				walkSpeed += 2.5f;
			}
			else
			{
				walkSpeed = 20.0f;
			}
		}
		else	// �΂ߕ���
		{
			if (walkSpeed < 12.0f)
			{
				walkSpeed += 3.0f;
			}
			else
			{
				walkSpeed = 12.0f;
			}
		}
	}
	else
	{
		animSpeed = 0.5f;
		if (walkSpeed > 0.0f)
		{
			walkSpeed -= 5.0f;
		}
		else
		{
			walkSpeed = 0.0f;
		}
	}

	// ���X�e�B�b�N���O�ɉ����ꂽ��O�i����
	if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) > 0)
	{
		area.x += sinf(angle + direXAngle) * -walkSpeed;
		area.z += cosf(angle + direXAngle) * -walkSpeed;
		direXAngle = 0.0f;
		direZAngle = 0.0f;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// ���X�e�B�b�N�����ɉ����ꂽ���ނ���
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direZAngle = DX_PI_F;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}

	// ���X�e�B�b�N�����ɉ����ꂽ�獶�Ɉړ�����
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X))
	{
		area.x += cosf(-angle) * walkSpeed;
		area.z += sinf(-angle) * walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, false);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// ���X�e�B�b�N���E�ɉ����ꂽ��E�Ɉړ�����
	else if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) > 0)
	{
		area.x += cosf(-angle) * -walkSpeed;
		area.z += sinf(-angle) * -walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, true);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// �L�����̑O��̌������C�����悭���邽��
	else
	{
		if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) == 0)
		{
			moveFlag = false;
			Player_PlayAnim(MOTION::idle);
		}
	}
}


// �R���X�g���N�^
Character::Character(const int modelHandle, const int collStageHandle) : BasicCreature(collStageHandle)
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
	area = VGet(0.0f, 0.0f, 0.0f);
	preArea = area;
	direXAngle = 0.0f;
	direZAngle = 0.0f;


	// �����̉e�Ɋւ���
	shadowHeight = 35.0f;
	shadowSize = 50.0f;


	// ���ꂼ��̑��x
	walkSpeed = 0.0f;
	animSpeed = 0.5f;


	// ���f���̍��W���X�V
	MV1SetPosition(this->modelHandle, area);
}

// �f�X�g���N�^
Character::~Character()
{
	MODEL_RELEASE(modelHandle);
}


// ���C���v���Z�X
void Character::Process(const unsigned __int8 controllNumber, const float getAngle)
{
	preArea = area;		// ���O�̍��W
	if (moveFlag)
	{
		angle = getAngle;	// �J���������̃A���O��
	}

	// �����̃v���Z�X
	MoveProcess(controllNumber);

	// ���[�V�����̎���
	Player_AnimProcess();

	// �X�e�[�W�̂����蔻��
	StageHit();

	// �������̉�]�p�x���Z�b�g
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, angle + direXAngle + direZAngle, 0.0f));
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(modelHandle, area);
}


void Character::PositionReset()
{
	area = VGet(0.0f, 0.0f, 0.0f);
}


// �`��
void Character::Draw()
{
	BasicObject::Draw();		// ��{�I�Ȃ��̂����������Ă���

	BasicObject::ShadowFoot();

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // _MODEL_DEBUG
#ifdef _CHARACTER_DEBUG
	printfDx("XAngle:%f\tZAngle%f\t��:%d\t��:%d\n", direXAngle, direZAngle, InputPad::GetPadThumbData(0, STICK_LEFT_X), InputPad::GetPadThumbData(0, STICK_LEFT_Y));
#endif // _CHARACTER_DEBUG
}