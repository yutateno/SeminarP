#include "CharacterSword.hpp"

using namespace MY_XINPUT;


// �����̃v���Z�X
void CharacterSword::MoveProcess(unsigned __int8 controllNumber)
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
		Player_PlayAnim(MOTION::dash);
	}
	// ���X�e�B�b�N�����ɉ����ꂽ���ނ���
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direZAngle = DX_PI_F;
		moveFlag = true;
		Player_PlayAnim(MOTION::dash);
	}

	// ���X�e�B�b�N�����ɉ����ꂽ�獶�Ɉړ�����
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X))
	{
		area.x += cosf(-angle) * walkSpeed;
		area.z += sinf(-angle) * walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)-(BASIC::MAX_STICK_MINUS);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
		Player_PlayAnim(MOTION::dash);
	}
	// ���X�e�B�b�N���E�ɉ����ꂽ��E�Ɉړ�����
	else if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) > 0)
	{
		area.x += cosf(-angle) * -walkSpeed;
		area.z += sinf(-angle) * -walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)(BASIC::MAX_STICK_PLUS);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
		Player_PlayAnim(MOTION::dash);
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


// �U���Ɋւ���v���Z�X
void CharacterSword::AttackProcess(unsigned __int8 controllNumber)
{
	// �U���̃R�}���h����������
	if (InputPad::GetPadButtonData(controllNumber, BUTTON_X) == 1)
	{
		// �ŏ��̎�
		if (attackFrame == 0)
		{
			walkSpeed = 60.0f;									// �ړ����x��ύX
			animSpeed = 0.4f;									// �A�j���[�V�������x��ύX
			
			
			// �ړ��v���Z�X���痬�p���đO���Ɉړ�������
			area.x += sinf(angle + direXAngle) * -walkSpeed;
			area.z += cosf(angle + direXAngle) * -walkSpeed;
			direXAngle = 0.0f;
			direZAngle = 0.0f;


			attackNow = true;					// �U�����Ă���t���b�O�𗧂Ă�
			attackNumber = MOTION::action1;		// �U���R�}���h�ԍ���1�Ԃɂ���
		}
		// ���ڈȍ~�̍U����
		else if (attackFrame < 10.0f)
		{
			attackNext = true;			// ���̍U�����[�V�����Ɉڍs����Ƃ����t���b�O�𗧂Ă�
		}
	}


	// �U�����[�V�����̏I�Փ�����Ŏ��̍s�������߂�
	if (attackFrame >= 9.0f)
	{
		// ���̍U���ֈڍs����ƃt���b�O�������Ă�����
		if (attackNext)
		{
			// �O���Ɉړ�����
			area.x += sinf(angle + direXAngle) * -walkSpeed;
			area.z += cosf(angle + direXAngle) * -walkSpeed;
			direXAngle = 0.0f;
			direZAngle = 0.0f;


			// ���O�̍U�����[�V�����Ŏ��̃��[�V���������߂�
			switch (preAttackNumber)
			{
			// �ŏ��̍U����
			case MOTION::action1:
				attackNumber = MOTION::action2;
				preAttackNumber = attackNumber;
				break;
			// ��R���{�ڂ̍U����
			case MOTION::action2:
				attackNumber = MOTION::action3;
				preAttackNumber = attackNumber;
				break;
			// �Ō�̍U����
			case MOTION::action3:
				attackNow = false;					// ���̃R���{���Ȃ��̂ōU���t���b�O��|��
				attackNumber = MOTION::action1;
				preAttackNumber = attackNumber;
				walkSpeed = 0.0f;
				Player_PlayAnim(MOTION::idle);
				break;
			}
			attackFrame = 0;		// �U���̃t���[��������
			attackNext = false;		// ���̍U�����邩�ǂ�����|��
		}
		// ���̍U�������Ȃ�
		else
		{
			walkSpeed = 0.0f;
			Player_PlayAnim(MOTION::idle);
			attackNow = false;					// �U���t���b�O��|��
			attackFrame = 0;
			attackNumber = MOTION::action1;
			preAttackNumber = attackNumber;
		}
	}



	// �U���t���b�O����������
	if (attackNow)
	{
		attackFrame += animSpeed;
		Player_PlayAnim(attackNumber);
		// ���X�e�B�b�N���O�ɉ����ꂽ��O������
		if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) > 0)
		{
			direXAngle = 0.0f;
			direZAngle = 0.0f;
		}
		// ���X�e�B�b�N�����ɉ����ꂽ���������
		if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
		{
			walkSpeed = -60.0f;
			direXAngle = 0.0f;
			direZAngle = DX_PI_F;
		}

		// ���X�e�B�b�N�����ɉ����ꂽ�獶������
		if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X))
		{
			direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)-(BASIC::MAX_STICK_MINUS);
			if (direZAngle != 0.0f)
			{
				direXAngle = -direXAngle;
			}
		}
		// ���X�e�B�b�N���E�ɉ����ꂽ��E������
		else if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) > 0)
		{
			direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)(BASIC::MAX_STICK_PLUS);
			if (direZAngle != 0.0f)
			{
				direXAngle = -direXAngle;
			}
		}
	}
	// �U���t���b�O���|��Ă�����
	else
	{
		
	}
}


CharacterSword::CharacterSword(const int modelHandle, const int collStageHandle, const int stairsHandle) : BasicCreature(collStageHandle)
{
	// �R�c���f���̓ǂݍ���
	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(modelHandle);


	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = MOTION::action1;
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
	shadowHeight = 20.0f;
	shadowSize = 50.0f;


	// ���ꂼ��̑��x
	walkSpeed = 0.0f;
	animSpeed = 0.25f;


	// �U���Ɋւ���
	attackNow = false;
	attackNext = false;
	attackFrame = 0;
	attackNumber = MOTION::action1;
	preAttackNumber = MOTION::action1;


	// �X�e�[�W�̃R���W�������̍X�V
	this->stairsHandle[0] = MV1DuplicateModel(stairsHandle);


	// ���f���̍��W���X�V
	MV1SetPosition(this->modelHandle, area);
}


CharacterSword::~CharacterSword()
{
	for (int i = 0; i != 10; ++i)
	{
		if (stairsHandle[i] != -1)
		{
			MV1DeleteModel(stairsHandle[i]);
			stairsHandle[i] = 0;
		}
	}
	if (modelHandle != -1)
	{
		MV1DeleteModel(modelHandle);
		modelHandle = 0;
	}
}

void CharacterSword::SetStairsArea(const VECTOR stairsArea, const int num)
{
	// �X�e�[�W�̃R���W�������̍X�V
	if (num != 0)
	{
		stairsHandle[num] = MV1DuplicateModel(stairsHandle[0]);
	}
	MV1SetupCollInfo(stairsHandle[num], -1);									// ���f���̃R���W���������Z�b�g�A�b�v(-1�ɂ��S�̃t���[��)
	MV1SetPosition(stairsHandle[num], stairsArea);				// �X�e�[�W�̍��W���X�V
	MV1SetFrameVisible(stairsHandle[num], -1, false);							// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
	MV1RefreshCollInfo(stairsHandle[num], -1);								// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
}


// ���C���v���Z�X
void CharacterSword::Process(const unsigned __int8 controllNumber, const float getAngle)
{
	preArea = area;		// ���O�̍��W
	if (moveFlag || attackNow)
	{
		angle = getAngle;	// �J���������̃A���O��
	}

	// �����̃v���Z�X
	if (!attackNow)
	{
		MoveProcess(controllNumber);
	}

	// �U���̃v���Z�X
	AttackProcess(controllNumber);

	// ���[�V�����̎���
	Player_AnimProcess();


	for (int i = 0; i != 10; ++i)
	{
		ActorHit(stairsHandle[i]);
	}

	// �X�e�[�W�̂����蔻��
	StageHit();


	// �������̉�]�p�x���Z�b�g
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, angle + direXAngle + direZAngle, 0.0f));
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(modelHandle, area);
}


void CharacterSword::PositionReset()
{
	area = VGet(0.0f, 0.0f, 0.0f);
}


// �`��
void CharacterSword::Draw()
{
	BasicObject::Draw();		// ��{�I�Ȃ��̂����������Ă���

	BasicObject::ShadowFoot();

	printfDx("%f\t%f\n", area.x, area.z);

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
#endif // _MODEL_DEBUG
#ifdef _CHARACTER_DEBUG
	printfDx("XAngle:%f\tZAngle%f\t��:%d\t��:%d\n", direXAngle, direZAngle, InputPad::GetPadThumbData(0, STICK_LEFT_X), InputPad::GetPadThumbData(0, STICK_LEFT_Y));
#endif // _CHARACTER_DEBUG
}