#include "BasicObject.hpp"


// ���̉e
void BasicObject::ShadowFoot()
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



BasicObject::BasicObject(const int collStageHandle)
{
	shadowHandle = 0;
	stageHandle = 0;

	// �e�̓ǂݍ���
	LoadFile::MyLoad("media\\Shadow.tyn", shadowHandle, ELOADFILE::graph);


	// �X�e�[�W�̃R���W�������̍X�V
	stageHandle = MV1DuplicateModel(collStageHandle);
	MV1SetScale(stageHandle, VGet(1.75f, 1.0f, 1.75f));
	MV1SetupCollInfo(stageHandle, -1);									// ���f���̃R���W���������Z�b�g�A�b�v(-1�ɂ��S�̃t���[��)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// �X�e�[�W�̍��W���X�V
	MV1SetFrameVisible(stageHandle, -1, false);							// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
	MV1RefreshCollInfo(stageHandle, -1);								// �X�e�[�W��`�悳���Ȃ��i�ł��ǂ���Draw�Ă΂Ȃ����炱��Ӗ��Ȃ��C������j
}


BasicObject::~BasicObject()
{
	GRAPHIC_RELEASE(shadowHandle);
	MODEL_RELEASE(stageHandle);
}


// ���̍��W��n���p�Q�b�^�[
const VECTOR BasicObject::GetArea() const
{
	return area;
}


// �`��
void BasicObject::Draw()
{
	/*if (modelHeight >= 40.0f)
	{
		if (!CheckCameraViewClip(VAdd(area, VGet(0.0f, 30.0f, 0.0f))))
		{
			MV1DrawModel(modelHandle);
		}
	}
	else
	{
		if (!CheckCameraViewClip(VAdd(area, VGet(0.0f, 10.0f, 0.0f))))
		{
			MV1DrawModel(modelHandle);
		}
	}*/
	if (!CheckCameraViewClip(area) && !CheckCameraViewClip(VAdd(area, VGet(0.0f, modelHeight, 0.0f))) && !CheckCameraViewClip(VAdd(area, VGet(0.0f, modelHeight / 2.0f, 0.0f))))
	{
		MV1DrawModel(modelHandle);
	}
}