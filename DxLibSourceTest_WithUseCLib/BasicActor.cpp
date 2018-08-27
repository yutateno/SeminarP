#include "BasicActor.hpp"

void BasicActor::ShadowFoot()
{
	// ���C�e�B���O�𖳌��ɂ���
	SetUseLighting(FALSE);

	// �y�o�b�t�@��L���ɂ���
	SetUseZBuffer3D(TRUE);

	// �e�N�X�`���A�h���X���[�h�� CLAMP �ɂ���( �e�N�X�`���̒[����͒[�̃h�b�g�����X���� )
	SetTextureAddressMode(DX_TEXADDRESS_CLAMP);

	// �v���C���[�̒����ɑ��݂���n�ʂ̃|���S�����擾
	ShadowHitResDim = MV1CollCheck_Capsule(stageHandle, -1, area, VAdd(area, VGet(0.0f, shadowHeight, 0.0f)), shadowSize);

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
		if (ShadowHitRes->Position[0].y > area.y - shadowHeight)
		{
			ShadowVertex[0].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[0].y - area.y) / shadowHeight));
		}

		if (ShadowHitRes->Position[1].y > area.y - shadowHeight)
		{
			ShadowVertex[1].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[1].y - area.y) / shadowHeight));
		}

		if (ShadowHitRes->Position[2].y > area.y - shadowHeight)
		{
			ShadowVertex[2].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[2].y - area.y) / shadowHeight));
		}

		// �t�u�l�͒n�ʃ|���S���ƃv���C���[�̑��΍��W���犄��o��
		ShadowVertex[0].u = (ShadowHitRes->Position[0].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[0].v = (ShadowHitRes->Position[0].z - area.z) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[1].u = (ShadowHitRes->Position[1].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[1].v = (ShadowHitRes->Position[1].z - area.z) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[2].u = (ShadowHitRes->Position[2].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[2].v = (ShadowHitRes->Position[2].z - area.z) / (shadowSize * 2.0f) + 0.5f;

		// �e�|���S����`��
		DrawPolygon3D(ShadowVertex, 1, shadowHandle, TRUE);
	}

	// ���o�����n�ʃ|���S�����̌�n��
	MV1CollResultPolyDimTerminate(ShadowHitResDim);

	// ���C�e�B���O��L���ɂ���
	SetUseLighting(TRUE);

	// �y�o�b�t�@�𖳌��ɂ���
	SetUseZBuffer3D(FALSE);
}

void BasicActor::Player_PlayAnim(int attach)
{
	// ���̃��[�V�������Ⴄ���̂�������
	if (attachNum != attach)
	{
		// ���O�̃��[�V�������L����������f�^�b�`����
		if (preAttach != -1 && motionBlendTime < 1.0f)
		{
			MV1DetachAnim(modelHandle, preAttach);
			preAttach = -1;
		}
		// ���̃��[�V����������
		preAttach = attachMotion;
		preMotionPlayTime = nowPlayTime;

		// ���������̃��[�V�����ԍ��ɓ����
		attachNum = attach;

		// �V���Ɏw��̃��[�V���������f���ɃA�^�b�`����
		attachMotion = MV1AttachAnim(modelHandle, attachNum, -1, false);

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

void BasicActor::Player_AnimProcess()
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
		totalTime = MV1GetAttachAnimTotalTime(modelHandle, attachMotion);

		// �Đ����Ԃ�i�߂�
		nowPlayTime += animSpeed;


		// �Đ����Ԃ������Ԃɓ��B���Ă����烋�[�v������
		if (nowPlayTime >= totalTime)
		{
			nowPlayTime = fmodf(nowPlayTime, totalTime);
		}

		// �ύX�����Đ����Ԃ����f���ɔ��f������
		MV1SetAttachAnimTime(modelHandle, attachMotion, nowPlayTime);

		// �A�j���[�V�����P�̃��f���ɑ΂��锽�f�����Z�b�g
		MV1SetAttachAnimBlendRate(modelHandle, attachMotion, motionBlendTime);
	}

	// �Đ����Ă��钼�O�̃��[�V�����̏���
	if (preAttach != -1)
	{
		// �A�j���[�V�����̑����Ԃ��擾
		totalTime = MV1GetAttachAnimTotalTime(modelHandle, preAttach);

		// �Đ����Ԃ�i�߂�
		preMotionPlayTime += animSpeed;

		// �Đ����Ԃ������Ԃɓ��B���Ă�����Đ����Ԃ����[�v������
		if (preMotionPlayTime > totalTime)
		{
			preMotionPlayTime = fmodf(preMotionPlayTime, totalTime);
		}

		// �ύX�����Đ����Ԃ����f���ɔ��f������
		MV1SetAttachAnimTime(modelHandle, preAttach, preMotionPlayTime);

		// �A�j���[�V�����Q�̃��f���ɑ΂��锽�f�����Z�b�g
		MV1SetAttachAnimBlendRate(modelHandle, preAttach, (1.0f - motionBlendTime));
	}
}

void BasicActor::StageHit()
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

BasicActor::BasicActor(int collStageHandle)
{
	// �e�̓ǂݍ���
	LoadFile::MyLoad("media\\Shadow.tyn", shadowHandle, ELOADFILE::graph);

	// �X�e�[�W�̃R���W�������̍X�V
	stageHandle = collStageHandle;
	MV1SetupCollInfo(stageHandle, -1);									// ���f���̃R���W���������Z�b�g�A�b�v(-1�ɂ��S�̃t���[��)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// �X�e�[�W�̍��W���X�V
	MV1SetFrameVisible(stageHandle, -1, false);							// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
	MV1RefreshCollInfo(stageHandle, -1);								// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j

	// �����蔻��Ɋւ���
	wallNum = 0;
	floorNum = 0;
	hitFlag = false;
	maxYHit = 0.0f;
	moveFlag = false;

	// ���[�V�����֘A
	nowPlayTime = 0.0f;
	motionBlendTime = 0.0f;
	preAttach = -1;
	preMotionPlayTime = 0.0f;

	angle = 0.0f;
}


BasicActor::~BasicActor()
{
	if (shadowHandle != -1)
	{
		DeleteGraph(shadowHandle);
	}

	if (stageHandle != -1)
	{
		MV1DeleteModel(stageHandle);
	}
}

void BasicActor::Draw()
{
	MV1DrawModel(modelHandle);

	ShadowFoot();
}