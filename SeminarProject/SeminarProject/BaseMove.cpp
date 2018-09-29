#include "BaseMove.hpp"

bool BaseMove::endFlag;		// �I���t���b�O
ESceneNumber BaseMove::scene;	// ���݂̃V�[��


// �V���h�E�}�b�v�ւ̕`��̏���
void BaseMove::ShadowCharaSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapCharaHandle);
}

// �V���h�E�}�b�v�ւ̕`����I��
void BaseMove::ShadowCharaSetUpAfter()
{
	ShadowMap_DrawEnd();
}

// �V���h�E�}�b�v�ւ̕`��̏���
void BaseMove::ShadowAnotherCharaSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapAnotherCharaHandle);
}

// �V���h�E�}�b�v�ւ̕`����I��
void BaseMove::ShadowAnotherCharaSetUpAfter()
{
	ShadowMap_DrawEnd();
}

// �V���h�E�}�b�v�ւ̕`��̏���
void BaseMove::ShadowNoMoveSetUpBefore()
{
	ShadowMap_DrawSetup(shadowMapNoMoveHandle);
}

// �V���h�E�}�b�v�ւ̕`����I��
void BaseMove::ShadowNoMoveSetUpAfter()
{
	ShadowMap_DrawEnd();
}


// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
void BaseMove::ShadowCharaDrawBefore()
{
	SetUseShadowMap(0, shadowMapCharaHandle);
}

// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
void BaseMove::ShadowCharaDrawAfter()
{
	SetUseShadowMap(0, -1);
}

// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
void BaseMove::ShadowAnotherCharaDrawBefore()
{
	SetUseShadowMap(1, shadowMapAnotherCharaHandle);
}

// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
void BaseMove::ShadowAnotherCharaDrawAfter()
{
	SetUseShadowMap(1, -1);
}

// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
void BaseMove::ShadowNoMoveDrawBefore()
{
	SetUseShadowMap(2, shadowMapNoMoveHandle);
}

// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
void BaseMove::ShadowNoMoveDrawAfter()
{
	SetUseShadowMap(2, -1);
}

void BaseMove::ShadowArea(const VECTOR charaArea)
{
	SetShadowMapDrawArea(shadowMapCharaHandle, VAdd(charaArea, shadowCharaLowArea), VAdd(charaArea, shadowCharaHighArea));
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, VAdd(charaArea, shadowAnotherCharaLowArea), VAdd(charaArea, shadowAnotherCharaHighArea));
}


// ��̃L�����̒����I����
int BaseMove::GetDistance(const VECTOR alpha, const VECTOR beta)
{
	double distance = sqrt((alpha.x - beta.x) * (alpha.x - beta.x) + (alpha.z - beta.z) * (alpha.z - beta.z));
	return (int)distance;
}

BaseMove::BaseMove()
{
	SetLightEnable(TRUE);


	// �t�H�O�Ɋւ���
	SetFogEnable(TRUE);					// �t�H�O��L���ɂ���
	SetFogColor(128, 128, 128);			// �t�H�O�̐F�ɂ���
	SetFogStartEnd(3500.0f, 6000.0f);	// �t�H�O�̊J�n����


	backGround = 0;
	backGround = MakeScreen(1920, 1080);


	// �V���h�E�}�b�v�n���h���̍쐬
	shadowMapCharaHandle = MakeShadowMap(2048, 2048);
	shadowMapAnotherCharaHandle = MakeShadowMap(512, 512);
	shadowMapNoMoveHandle = MakeShadowMap(256, 256);


	// �V���h�E�}�b�v�ɕ`�悷��͈͂�ݒ�
	shadowCharaLowArea = VGet(-500.0f, -1.0f, -500.0f);
	shadowCharaHighArea = VGet(500.0f, 10.0f, 500.0f);

	shadowAnotherCharaLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowAnotherCharaHighArea = VGet(2000.0f, 100.0f, 2000.0f);

	shadowNoMoveLowArea = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowNoMoveHighArea = VGet(2000.0f, 1000.0f, 2000.0f);

	SetShadowMapDrawArea(shadowMapCharaHandle, shadowCharaLowArea, shadowCharaHighArea);
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, shadowAnotherCharaLowArea, shadowAnotherCharaHighArea);
	SetShadowMapDrawArea(shadowMapNoMoveHandle, shadowNoMoveLowArea, shadowNoMoveHighArea);


	lightDire = VGet(-0.5f, -0.7f, 0.5f);
	// ���C�g�̕�����ݒ�
	SetLightDirection(lightDire);
	// �V���h�E�}�b�v���z�肷�郉�C�g�̕������Z�b�g
	SetShadowMapLightDirection(shadowMapCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapAnotherCharaHandle, lightDire);
	SetShadowMapLightDirection(shadowMapNoMoveHandle, lightDire);
}

BaseMove::~BaseMove()
{
	// �V���h�E�}�b�v�̍폜
	SHADOW_MAP_RELEASE(shadowMapNoMoveHandle);
	SHADOW_MAP_RELEASE(shadowMapAnotherCharaHandle);
	SHADOW_MAP_RELEASE(shadowMapCharaHandle);
}

const bool BaseMove::GetEndFlag()
{
	return endFlag;
}

const ESceneNumber BaseMove::GetScene()
{
	return scene;
}

void BaseMove::SetScene(const ESceneNumber scene)
{
	this->scene = scene;
}
