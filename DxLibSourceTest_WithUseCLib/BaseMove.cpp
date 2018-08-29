#include "BaseMove.hpp"

void BaseMove::ShadowCharaSetUpBefore()
{
	// �V���h�E�}�b�v�ւ̕`��̏���
	ShadowMap_DrawSetup(shadowMapCharaHandle);
}

void BaseMove::ShadowCharaSetUpAfter()
{
	// �V���h�E�}�b�v�ւ̕`����I��
	ShadowMap_DrawEnd();
}

void BaseMove::ShadowAnotherCharaSetUpBefore()
{
	// �V���h�E�}�b�v�ւ̕`��̏���
	ShadowMap_DrawSetup(shadowMapAnotherCharaHandle);
}

void BaseMove::ShadowAnotherCharaSetUpAfter()
{
	// �V���h�E�}�b�v�ւ̕`����I��
	ShadowMap_DrawEnd();
}

void BaseMove::ShadowNoMoveSetUpBefore()
{
	// �V���h�E�}�b�v�ւ̕`��̏���
	ShadowMap_DrawSetup(shadowMapNoMoveHandle);
}

void BaseMove::ShadowNoMoveSetUpAfter()
{
	// �V���h�E�}�b�v�ւ̕`����I��
	ShadowMap_DrawEnd();
}


void BaseMove::ShadowCharaDrawBefore()
{
	// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
	SetUseShadowMap(0, shadowMapCharaHandle);
}

void BaseMove::ShadowCharaDrawAfter()
{
	// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
	SetUseShadowMap(0, -1);
}

void BaseMove::ShadowAnotherCharaDrawBefore()
{
	// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
	SetUseShadowMap(1, shadowMapAnotherCharaHandle);
}

void BaseMove::ShadowAnotherCharaDrawAfter()
{
	// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
	SetUseShadowMap(1, -1);
}

void BaseMove::ShadowNoMoveDrawBefore()
{
	// �`��Ɏg�p����V���h�E�}�b�v��ݒ�
	SetUseShadowMap(2, shadowMapNoMoveHandle);
}

void BaseMove::ShadowNoMoveDrawAfter()
{
	// �`��Ɏg�p����V���h�E�}�b�v�̐ݒ������
	SetUseShadowMap(2, -1);
}

void BaseMove::ShadowArea(VECTOR charaArea)
{
	SetShadowMapDrawArea(shadowMapCharaHandle		, VAdd(charaArea, shadowCharaLowArea)		, VAdd(charaArea, shadowCharaHighArea));
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, VAdd(charaArea, shadowAnotherCharaLowArea), VAdd(charaArea, shadowAnotherCharaHighArea));
}



int BaseMove::GetDistance(VECTOR alpha, VECTOR beta)
{
	double distance = sqrt((alpha.x - beta.x) * (alpha.x - beta.x) + (alpha.z - beta.z) * (alpha.z - beta.z));
	return (int)distance;
}

BaseMove::BaseMove()
{
	// �V���h�E�}�b�v�n���h���̍쐬
	shadowMapCharaHandle		 = MakeShadowMap(4096, 4096);
	shadowMapAnotherCharaHandle	 = MakeShadowMap(512, 512);
	shadowMapNoMoveHandle		 = MakeShadowMap(4096, 4096);


	// �V���h�E�}�b�v�ɕ`�悷��͈͂�ݒ�
	shadowCharaLowArea			 = VGet(-500.0f, -1.0f, -500.0f);
	shadowCharaHighArea			 = VGet(500.0f, 10.0f, 500.0f);

	shadowAnotherCharaLowArea	 = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowAnotherCharaHighArea	 = VGet(2000.0f, 100.0f, 2000.0f);

	shadowNoMoveLowArea			 = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowNoMoveHighArea		 = VGet(2000.0f, 1000.0f, 2000.0f);

	SetShadowMapDrawArea(shadowMapCharaHandle		, shadowCharaLowArea		, shadowCharaHighArea		);
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, shadowAnotherCharaLowArea	, shadowAnotherCharaHighArea);
	SetShadowMapDrawArea(shadowMapNoMoveHandle		, shadowNoMoveLowArea		, shadowNoMoveHighArea		);


	lightDire = VGet(-0.5f, -0.7f, 0.5f);
	// ���C�g�̕�����ݒ�
	SetLightDirection(lightDire);
	// �V���h�E�}�b�v���z�肷�郉�C�g�̕������Z�b�g
	SetShadowMapLightDirection(shadowMapCharaHandle			, lightDire);
	SetShadowMapLightDirection(shadowMapAnotherCharaHandle	, lightDire);
	SetShadowMapLightDirection(shadowMapNoMoveHandle		, lightDire);
}

BaseMove::~BaseMove()
{
	// �V���h�E�}�b�v�̍폜
	DeleteShadowMap(shadowMapNoMoveHandle);
	DeleteShadowMap(shadowMapAnotherCharaHandle);
	DeleteShadowMap(shadowMapCharaHandle);
}
