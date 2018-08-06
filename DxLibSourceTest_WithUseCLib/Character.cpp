#include "Character.hpp"

void Character::Player_PlayAnim(int attach)
{
	// ���̃��[�V�������Ⴄ���̂�������
	if (attachNum != attach)
	{
		// ���O�̃��[�V�������L����������f�^�b�`����
		if (preAttach != -1 && motionBlendTime < 1.0f)
		{
			MV1DetachAnim(charamodelhandle, preAttach);
			preAttach = -1;
		}
		// ���̃��[�V����������
		preAttach = attachMotion;
		preMotionPlayTime = nowPlayTime;

		// ���������̃��[�V�����ԍ��ɓ����
		attachNum = attach;

		// �V���Ɏw��̃��[�V���������f���ɃA�^�b�`����
		attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, false);

		// ���쎞�Ԃ�����������
		nowPlayTime = 0.0f;

		// �u�����h���͒��O�̃��[�V�������L���ł͂Ȃ��ꍇ�͂P�D�O��( �Đ����̃��[�V�����P���P�O�O���̏�� )�ɂ���
		if (preMotionPlayTime == -1)
		{
			motionBlendTime = 1.0f;
		}
		else
		{
			motionBlendTime = 0.0f;
		}
	}
}


void Character::Player_AnimProcess()
{
	// �u�����h�����P�ȉ��̏ꍇ�͂P�ɋ߂Â���
	if (motionBlendTime < 1.0)
	{
		motionBlendTime += 0.1;
		if (motionBlendTime >= 1.0)
		{
			motionBlendTime = 1.0;
		}
	}

	// �Đ����Ă��錻�݂̃��[�V�����̏���
	if (attachMotion != -1)
	{
		// ���[�V�����̑����Ԃ��擾
		totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

		// �Đ����Ԃ�i�߂�
		nowPlayTime += animSpeed;


		// �Đ����Ԃ������Ԃɓ��B���Ă����烋�[�v������
		if (nowPlayTime >= totalTime)
		{
			nowPlayTime = MYINPUTPAD::fmod(nowPlayTime, totalTime);
		}

		// �ύX�����Đ����Ԃ����f���ɔ��f������
		MV1SetAttachAnimTime(charamodelhandle, attachMotion, nowPlayTime);

		// �A�j���[�V�����P�̃��f���ɑ΂��锽�f�����Z�b�g
		MV1SetAttachAnimBlendRate(charamodelhandle, attachMotion, motionBlendTime);
	}

	// �Đ����Ă��钼�O�̃��[�V�����̏���
	if (preAttach != -1)
	{
		// �A�j���[�V�����̑����Ԃ��擾
		totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, preAttach);

		// �Đ����Ԃ�i�߂�
		preMotionPlayTime += animSpeed;

		// �Đ����Ԃ������Ԃɓ��B���Ă�����Đ����Ԃ����[�v������
		if (preMotionPlayTime > totalTime)
		{
			preMotionPlayTime = MYINPUTPAD::fmod(preMotionPlayTime, totalTime);
		}

		// �ύX�����Đ����Ԃ����f���ɔ��f������
		MV1SetAttachAnimTime(charamodelhandle, preAttach, preMotionPlayTime);

		// �A�j���[�V�����Q�̃��f���ɑ΂��锽�f�����Z�b�g
		MV1SetAttachAnimBlendRate(charamodelhandle, preAttach, (1.0f - motionBlendTime));
	}
}

Character::Character()
{
	// �R�c���f���̓ǂݍ���
	this->charamodelhandle = MV1LoadModel("media\\CLPH\\motion\\CLPH_idle.fbx");

	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = 0;
	attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

	modelHeight = 180;
	modelWigth = 40.0f;

	area = VGet(0, 0, 0);
	angle = 0;

	// ���ꂼ��̑��x
	turnSpeed = DX_PI_F / 90;
	walkSpeed = 3.0f;
	animSpeed = 1.0f;

	// ���[�V�����֘A
	nowPlayTime = 0.0;
	motionBlendTime = 0.0;
	preAttach = -1;
	preMotionPlayTime = 0.0;
}

Character::~Character()
{
	if (charamodelhandle != -1)
	{
		MV1DeleteModel(charamodelhandle);
	}
}

void Character::Process(unsigned __int8 controllNumber)
{
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(charamodelhandle, area);
}

void Character::Process(unsigned __int8 controllNumber, float getAngle)
{
	angle = getAngle;

	// ���X�e�B�b�N���O�ɉ����ꂽ��O�i����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) > 0)
	{
		area.x += MYINPUTPAD::sinf(angle) * -walkSpeed;
		area.z += MYINPUTPAD::cosf(angle) * -walkSpeed;
	}
	// ���X�e�B�b�N�����ɉ����ꂽ���ނ���
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) < 0)
	{
		area.x += MYINPUTPAD::sinf(angle) * walkSpeed;
		area.z += MYINPUTPAD::cosf(angle) * walkSpeed;
	}
	// ���X�e�B�b�N�����ɉ����ꂽ�獶�Ɉړ�����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) < 0)
	{
		area.x += MYINPUTPAD::cosf(-angle) * walkSpeed;
		area.z += MYINPUTPAD::sinf(-angle) * walkSpeed;
	}
	// ���X�e�B�b�N���E�ɉ����ꂽ��E�Ɉړ�����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) > 0)
	{
		area.x += MYINPUTPAD::cosf(-angle) * -walkSpeed;
		area.z += MYINPUTPAD::sinf(-angle) * -walkSpeed;
	}

	// ���[�V�����̎���
	Player_AnimProcess();

	// �������̉�]�p�x���Z�b�g
	MV1SetRotationXYZ(charamodelhandle, VGet(0.0f, angle, 0.0f));
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(charamodelhandle, area);
}


void Character::Draw()
{
	MV1DrawModel(charamodelhandle);
}

VECTOR Character::GetArea()
{
	return area;
}