#include "Character.hpp"

// �����̃v���Z�X
void Character::MoveProcess(unsigned __int8 controllNumber)
{
	// ���X�e�B�b�N���O�ɉ����ꂽ��O�i����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) > 0)
	{
		area.x += sinf(angle + direXAngle) * -walkSpeed;
		area.z += cosf(angle + direXAngle) * -walkSpeed;
		direXAngle = 0.0f;
		direZAngle = 0.0f;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// ���X�e�B�b�N�����ɉ����ꂽ���ނ���
	if (0 > MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y))
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direZAngle = DX_PI_F;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}

	// ���X�e�B�b�N�����ɉ����ꂽ�獶�Ɉړ�����
	if (0 > MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X))
	{
		area.x += cosf(-angle) * walkSpeed;
		area.z += sinf(-angle) * walkSpeed;
		direXAngle = ((float)MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2.0f)) / (float)-BASIC::MAX_STICK_MINUS;
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// ���X�e�B�b�N���E�ɉ����ꂽ��E�Ɉړ�����
	else if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) > 0)
	{
		area.x += cosf(-angle) * -walkSpeed;
		area.z += sinf(-angle) * -walkSpeed;
		direXAngle = ((float)MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2.0f)) / (float)BASIC::MAX_STICK_PLUS;
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
		if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) == 0)
		{
			moveFlag = false;
			Player_PlayAnim(MOTION::idle);
		}
	}

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
}


// �R���X�g���N�^
Character::Character(int collStageHandle) : BasicActor(collStageHandle)
{
	// �R�c���f���̓ǂݍ���
	LoadFile::MyLoad("media\\CLPH\\motion\\CLPH_motionALL.fyn", modelHandle, ELOADFILE::fbxmodel);

	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = MOTION::idle;
	attachMotion = MV1AttachAnim(modelHandle, attachNum, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	totalTime = MV1GetAttachAnimTotalTime(modelHandle, attachMotion);

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
	MV1SetPosition(modelHandle, area);
}

// �f�X�g���N�^
Character::~Character()
{
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
	}
}


// ���C���v���Z�X
void Character::Process(unsigned __int8 controllNumber, float getAngle)
{
	preArea = area;		// ���O�̍��W
	angle = getAngle;	// �J���������̃A���O��

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

// �`��
void Character::Draw()
{
	BasicActor::Draw();		// ��{�I�Ȃ��̂����������Ă���

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // _MODEL_DEBUG
}