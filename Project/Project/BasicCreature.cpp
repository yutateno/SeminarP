#include "BasicCreature.hpp"

// �g���A�j���[�V�����Ǘ�
void BasicCreature::Player_PlayAnim(int attach)
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

// �S�ẴA�j���[�V�����̊Ǘ�
void BasicCreature::Player_AnimProcess()
{
	// �u�����h�����P�ȉ��̏ꍇ�͂P�ɋ߂Â���
	if (motionBlendTime < 1.0)
	{
		motionBlendTime += 0.075f;
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


// �X�e�[�W�̂����蔻�菈��
void BasicCreature::StageHit()
{
	ActorHit(stageHandle);
}

// �X�e�[�W�̂����蔻�菈��
void BasicCreature::ActorHit(int stageHandle)
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
		if (hitDim.Dim[i].Normal.y < 0.001f && hitDim.Dim[i].Normal.y > -0.001f)
		{
			// �ǂ��Ƃ��Ă��L�����N�^�[�̑���菭����𒲂ׂ�
			if (hitDim.Dim[i].Position[0].y > area.y + modelHeight + 50.0f
				|| hitDim.Dim[i].Position[1].y > area.y + modelHeight + 50.0f
				|| hitDim.Dim[i].Position[2].y > area.y + modelHeight + 50.0f)
			{
				// �ő�ɂȂ�܂ŕۑ�����
				if (wallNum < 64)
				{
					wallPoly[wallNum] = &hitDim.Dim[i];		// �q�b�g�����|���S������ۑ�
					wallNum++;
				}
			}
		}
		else
		{
			// �ő�ɂȂ�܂ŕۑ�
			if (floorNum < 64)
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

		fallCount++;			// �����Ă���Ƃ���

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

		// ���ɓ����������ǂ����ŏ���
		if (hitFlag)
		{
			area.y = maxYHit;

			fallCount = 0;
		}
	}
	else	// ���ɐG��Ă��Ȃ�
	{
		area.y -= 0.75f;
	}

	// ���o���������������
	MV1CollResultPolyDimTerminate(hitDim);
}


// �R���X�g���N�^
BasicCreature::BasicCreature(const int collStageHandle) :BasicObject(collStageHandle)
{
	// �X�e�[�W�̃R���W�������̍X�V
	stageHandle = MV1DuplicateModel(collStageHandle);
	MV1SetScale(stageHandle, VGet(1.75f, 1.0f, 1.75f));
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

	fallCount = 0;

	angle = 0.0f;
}

// �f�X�g���N�^
BasicCreature::~BasicCreature()
{
	MODEL_RELEASE(stageHandle);
}
