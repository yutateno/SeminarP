#include "MainMove1.hpp"

// ���f�����Ƃ̂����蔻�菈��
void MainMove1::ActorHit()
{
	enemy->Process();
}


// �R���X�g���N�^
MainMove1::MainMove1()
{
	stage = new Stage();
	character = new Character(stage->GetCollStageHandle());
	enemy = new EnemyMove1(stage->GetCollStageHandle(), 100.0f, 0.0f);
	camera = new Camera(character->GetArea(), stage->GetCollStageHandle());
	light = new PointLight();

	stage->LoadInit();

	BaseMove::ShadowNoMoveSetUpBefore();
	stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}

// �f�X�g���N�^
MainMove1::~MainMove1()
{
	delete light;
	delete camera;
	delete enemy;
	delete character;
	delete stage;
}


// �`��
void MainMove1::Draw()
{
	BaseMove::ShadowCharaSetUpBefore();
	character->Draw();
	enemy->Draw();
	BaseMove::ShadowCharaSetUpAfter();
	
	BaseMove::ShadowAnotherCharaSetUpBefore();
	enemy->Draw();
	BaseMove::ShadowAnotherCharaSetUpAfter();

	BaseMove::ShadowCharaDrawBefore();
	BaseMove::ShadowAnotherCharaDrawBefore();
	BaseMove::ShadowNoMoveDrawBefore();
	character->Draw();
	enemy->Draw();
	stage->Draw();
	BaseMove::ShadowCharaDrawAfter();
	BaseMove::ShadowAnotherCharaDrawAfter();
	BaseMove::ShadowNoMoveDrawAfter();
	
	//light->Draw(character->GetArea());
}

// ���C���v���Z�X
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	//light->Process(character->GetArea());
	ActorHit();

	BaseMove::ShadowArea(character->GetArea());
}
