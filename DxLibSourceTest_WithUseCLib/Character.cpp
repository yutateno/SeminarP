#include "Character.hpp"

void Character::ShadowFoot()
{
	// ���C�e�B���O�𖳌��ɂ���
	SetUseLighting(FALSE);

	// �y�o�b�t�@��L���ɂ���
	SetUseZBuffer3D(TRUE);

	// �e�N�X�`���A�h���X���[�h�� CLAMP �ɂ���( �e�N�X�`���̒[����͒[�̃h�b�g�����X���� )
	SetTextureAddressMode(DX_TEXADDRESS_CLAMP);

	// �v���C���[�̒����ɑ��݂���n�ʂ̃|���S�����擾
	ShadowHitResDim = MV1CollCheck_Capsule(stageHandle, -1, area, VAdd(area, VGet(0.0f, (float)PLAYER_SHADOW_HEIGHT, 0.0f)), PLAYER_SHADOW_SIZE);

	// ���_�f�[�^�ŕω��������������Z�b�g
	ShadowVertex[0].dif = GetColorU8(255, 255, 255, 255);
	ShadowVertex[0].spc = GetColorU8(0, 0, 0, 0);
	ShadowVertex[0].su = 0.0f;
	ShadowVertex[0].sv = 0.0f;
	ShadowVertex[1] = ShadowVertex[0];
	ShadowVertex[2] = ShadowVertex[0];

	// ���̒����ɑ��݂���|���S���̐������J��Ԃ�
	ShadowHitRes = ShadowHitResDim.Dim;
	for (int i = 0; i != ShadowHitResDim.HitNum; ++i, ++ShadowHitRes)
	{
		// �|���S���̍��W�͒n�ʃ|���S���̍��W
		ShadowVertex[0].pos = ShadowHitRes->Position[0];
		ShadowVertex[1].pos = ShadowHitRes->Position[1];
		ShadowVertex[2].pos = ShadowHitRes->Position[2];

		// ������Ǝ����グ�ďd�Ȃ�Ȃ��悤�ɂ���
		ShadowSlideVec = VScale(ShadowHitRes->Normal, 0.5f);
		ShadowVertex[0].pos = VAdd(ShadowVertex[0].pos, ShadowSlideVec);
		ShadowVertex[1].pos = VAdd(ShadowVertex[1].pos, ShadowSlideVec);
		ShadowVertex[2].pos = VAdd(ShadowVertex[2].pos, ShadowSlideVec);

		// �|���S���̕s�����x��ݒ肷��
		ShadowVertex[0].dif.a = 0;
		ShadowVertex[1].dif.a = 0;
		ShadowVertex[2].dif.a = 0;
		if (ShadowHitRes->Position[0].y > area.y - PLAYER_SHADOW_HEIGHT)
		{
			ShadowVertex[0].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[0].y - area.y) / PLAYER_SHADOW_HEIGHT));
		}

		if (ShadowHitRes->Position[1].y > area.y - PLAYER_SHADOW_HEIGHT)
		{
			ShadowVertex[1].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[1].y - area.y) / PLAYER_SHADOW_HEIGHT));
		}

		if (ShadowHitRes->Position[2].y > area.y - PLAYER_SHADOW_HEIGHT)
		{
			ShadowVertex[2].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[2].y - area.y) / PLAYER_SHADOW_HEIGHT));
		}

		// �t�u�l�͒n�ʃ|���S���ƃv���C���[�̑��΍��W���犄��o��
		ShadowVertex[0].u = (ShadowHitRes->Position[0].x - area.x) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;
		ShadowVertex[0].v = (ShadowHitRes->Position[0].z - area.z) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;
		ShadowVertex[1].u = (ShadowHitRes->Position[1].x - area.x) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;
		ShadowVertex[1].v = (ShadowHitRes->Position[1].z - area.z) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;
		ShadowVertex[2].u = (ShadowHitRes->Position[2].x - area.x) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;
		ShadowVertex[2].v = (ShadowHitRes->Position[2].z - area.z) / (PLAYER_SHADOW_SIZE * 2.0f) + 0.5f;

		// �e�|���S����`��
		DrawPolygon3D(ShadowVertex, 1, shadowhandle, TRUE);
	}

	// ���o�����n�ʃ|���S�����̌�n��
	MV1CollResultPolyDimTerminate(ShadowHitResDim);

	// ���C�e�B���O��L���ɂ���
	SetUseLighting(TRUE);

	// �y�o�b�t�@�𖳌��ɂ���
	SetUseZBuffer3D(FALSE);
}

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
		motionBlendTime += 0.1f;
		if (motionBlendTime >= 1.0f)
		{
			motionBlendTime = 1.0f;
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
			nowPlayTime = fmodf(nowPlayTime, totalTime);
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
			preMotionPlayTime = fmodf(preMotionPlayTime, totalTime);
		}

		// �ύX�����Đ����Ԃ����f���ɔ��f������
		MV1SetAttachAnimTime(charamodelhandle, preAttach, preMotionPlayTime);

		// �A�j���[�V�����Q�̃��f���ɑ΂��锽�f�����Z�b�g
		MV1SetAttachAnimBlendRate(charamodelhandle, preAttach, (1.0f - motionBlendTime));
	}
}

void Character::StageHit()
{
	// �v���C���[���J�v�Z���Ƃ��ăX�e�[�W�Ƃ̃R���W�������𒲂ׂ�(OBB�`��)
	hitDim = MV1CollCheck_Capsule(stageHandle, -1, area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth);

	// �|���S���̐����ď�����
	wallNum = 0;
	floorNum = 0;

	// ���o���ꂽ���������ׂ�
	for (int i = 0, j = hitDim.HitNum; i != j; ++i)
	{
		// �ǂ��ǂ������f���邽�߁AXZ���ɐ������ǂ�����@�����O�ɋ߂����ǂ����Œ��ׂ�
		if (hitDim.Dim[i].Normal.y < 0.1f && hitDim.Dim[i].Normal.y > -0.1f)
		{
			// �ǂ��Ƃ��Ă��L�����N�^�[�̑���菭����𒲂ׂ�
			if (hitDim.Dim[i].Position[0].y > area.y + 10.0f
				|| hitDim.Dim[i].Position[1].y > area.y + 10.0f
				|| hitDim.Dim[i].Position[2].y > area.y + 10.0f)
			{
				// �ő�ɂȂ�܂ŕۑ�����
				if (wallNum < 2048)
				{
					wallPoly[wallNum] = &hitDim.Dim[i];		// �q�b�g�����|���S������ۑ�
					wallNum++;
				}
			}
		}
		else
		{
			// �ő�ɂȂ�܂ŕۑ�
			if (floorNum < 2048)
			{
				floorPoly[floorNum] = &hitDim.Dim[i];		// �q�b�g�����|���S������ۑ�
				floorNum++;
			}
		}
	}

	// �ǔ���
	if (wallNum != 0)
	{
		hitFlag = false;	// �ǂɂ͓������ĂȂ��Ƃ���

		// �ړ����Ă��邩�ǂ����Ŕ��f��ς���
		if (moveFlag)
		{
			for (int i = 0; i != wallNum; ++i)
			{
				int j;		// ��������邽�߃��[�v�O�Ő錾

				mainPoly = wallPoly[i];			// ���̒��ׂ�|���S������n��
				
				// �������Ă��邩�ǂ����𒲂ׂ�
				if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == false)
				{
					continue;			// �������Ă��Ȃ��̂Ŏ��I
				}

				hitFlag = true;			// �������Ă���Ƃ���

				// �ړ����Ă���̂Ŏ��̍��W�œ������Ă��Ȃ�������
				for (j = 0; j != wallNum; ++j)
				{
					mainPoly = wallPoly[j];			// ���̒��ׂ�|���S������n��

					// �������Ă��邩�ǂ����𒲂ׂ�
					if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
					{
						break;			// �������Ă����̂Ŕ�����
					}
				}

				// �ړ���œ�����Ȃ������̂Ŕ��������
				if (j == wallNum)
				{
					hitFlag = false;
					break;
				}
			}
		}
		else
		{
			for (int i = 0; i != wallNum; ++i)
			{
				mainPoly = wallPoly[i];			// ���̒��ׂ�|���S������n��

				// �������Ă��邩�ǂ����𒲂ׂ�
				if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
				{
					hitFlag = true;
					break;			// �������Ă����̂Ŕ�����
				}
			}
		}

		// �������Ă����̂ŉ����o��
		if (hitFlag)
		{
			int j, k;
			// ���萔��32�Ƃ���
			for (int i = 0; i != 64; ++i)
			{
				// �ǃ|���S���̐������J��Ԃ�
				for (j = 0; j != wallNum; ++j)
				{
					mainPoly = wallPoly[j];

					// �������Ă��邩�ǂ����𒲂ׂ�
					if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == false)
					{
						continue;
					}

					VECTOR slideVec = VCross(VSub(area, preArea), mainPoly->Normal);
					slideVec = VCross(mainPoly->Normal, slideVec);
					area = VAdd(preArea, slideVec);

					for (k = 0; k != wallNum; ++k)
					{
						// �������Ă����烋�[�v�𔲂���
						mainPoly = wallPoly[k];
						if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
						{
							break;
						}
					}

					// �S�Ẵ|���S���Ɠ������Ă��Ȃ������̂Ŕ�����
					if (k == wallNum)
					{
						break;
					}
				}

				// ����𒲂ׂ�O�ɑS�Ẵ|���S���Ɠ������Ă��Ȃ������̂Ŕ�����
				if (j != wallNum)
				{
					break;
				}
			}
		}
	}

	// ������
	if (floorNum != 0)
	{
		hitFlag = false;		// �������ĂȂ��Ƃ���

		maxYHit = 0.0f;			// ����������

		for (int i = 0; i != floorNum; ++i)
		{
			mainPoly = floorPoly[i];			// �����ׂ�|���S������n��

			lineResult = HitCheck_Line_Triangle(VAdd(area, VGet(0.0f, modelHeight, 0.0f)), VAdd(area, VGet(0.0f, -5.0f, 0.0f)), mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]);

			// �������ĂȂ������牽�����Ȃ�
			if (!lineResult.HitFlag)
			{
				continue;
			}

			// ���ɓ��������|���S���������č��܂Ō��o�������̂��Ⴉ�����牽�����Ȃ�
			if (hitFlag && maxYHit > lineResult.Position.y)
			{
				continue;
			}

			// �ڐG����Y���W��ێ�����
			maxYHit = lineResult.Position.y;

			hitFlag = true;
		}

		// �����������ǂ����ŏ���
		if (hitFlag)
		{
			area.y = maxYHit;
		}
	}

	// ���o���������������
	MV1CollResultPolyDimTerminate(hitDim);
}

Character::Character(int collStageHandle)
{
	// �X�e�[�W�̃R���W�������̍X�V
	stageHandle = collStageHandle;
	MV1SetupCollInfo(stageHandle, -1);									// ���f���̃R���W���������Z�b�g�A�b�v(-1�ɂ��S�̃t���[��)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// �X�e�[�W�̍��W���X�V
	MV1SetFrameVisible(stageHandle, -1, false);							// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
	MV1RefreshCollInfo(stageHandle, -1);								// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j

	// �R�c���f���̓ǂݍ���
	LoadFile::MyLoad("media\\CLPH\\motion\\CLPH_motionALL.fyn", charamodelhandle, ELOADFILE::mv1model);

	// �R�c���f����0�Ԗڂ̃A�j���[�V�������A�^�b�`����
	attachNum = MOTION::idle;
	attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, FALSE);

	// �A�^�b�`�����A�j���[�V�����̑��Đ����Ԃ��擾����
	totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

	// ���f���̊�{���
	modelHeight = 160.0f;
	modelWigth = 50.0f;

	// ���f���̌����ƈʒu
	area = VGet(282.0f, 0.0f, 0.0f);
	preArea = area;
	angle = 0.0f;
	direXAngle = 0.0f;
	direYAngle = 0.0f;

	// �����蔻��Ɋւ���
	wallNum = 0;
	floorNum = 0;
	hitFlag = false;
	moveFlag = false;
	maxYHit = 0.0f;

	// �����̉e�Ɋւ���
	LoadFile::MyLoad("media\\Shadow.tyn", shadowhandle, ELOADFILE::graph);
	PLAYER_SHADOW_HEIGHT = 35;
	PLAYER_SHADOW_SIZE = 65.0f;

	// ���ꂼ��̑��x
	walkSpeed = 0.0f;
	animSpeed = 1.0f;

	// ���[�V�����֘A
	nowPlayTime = 0.0f;
	motionBlendTime = 0.0f;
	preAttach = -1;
	preMotionPlayTime = 0.0f;

	// ���f���̍��W���X�V
	MV1SetPosition(charamodelhandle, area);
}

Character::~Character()
{
	if (charamodelhandle != -1)
	{
		MV1DeleteModel(charamodelhandle);
	}
	if (stageHandle != -1)
	{
		MV1DeleteModel(stageHandle);
	}
}

void Character::Process(unsigned __int8 controllNumber)
{
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(charamodelhandle, area);
}

void Character::Process(unsigned __int8 controllNumber, float getAngle)
{
	preArea = area;
	angle = getAngle;

	// ���X�e�B�b�N���O�ɉ����ꂽ��O�i����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) > 0)
	{
		area.x += sinf(angle + direXAngle) * -walkSpeed;
		area.z += cosf(angle + direXAngle) * -walkSpeed;
		direXAngle = 0.0f;
		direYAngle = 0.0f;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}
	// ���X�e�B�b�N�����ɉ����ꂽ���ނ���
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) < 0)
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direYAngle = DX_PI_F;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
	}

	// ���X�e�B�b�N�����ɉ����ꂽ�獶�Ɉړ�����
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) < 0)
	{
		area.x += cosf(-angle) * walkSpeed;
		area.z += sinf(-angle) * walkSpeed;
		direXAngle = ((float)MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2.0f)) / (float)-BASIC::MAX_STICK_MINUS;
		if (direYAngle != 0.0f)
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
		if (direYAngle != 0.0f)
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
		if (walkSpeed != 20.0f)
		{
			walkSpeed += 2.5f;
		}
	}
	else
	{
		if (walkSpeed != 0.0f)
		{
			walkSpeed -= 5.0f;
		}
	}

	// ���[�V�����̎���
	Player_AnimProcess();

	// �X�e�[�W�̂����蔻��
	StageHit();

	// �������̉�]�p�x���Z�b�g
	MV1SetRotationXYZ(charamodelhandle, VGet(0.0f, angle + direXAngle + direYAngle, 0.0f));
	// �w��ʒu�Ƀ��f����z�u
	MV1SetPosition(charamodelhandle, area);
}


void Character::Draw()
{
	MV1DrawModel(charamodelhandle);

	ShadowFoot();

	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// �����蔻����m�F�p�̕\���e�X�g
}

VECTOR Character::GetArea()
{
	return area;
}