#include "MainMove1.hpp"

// ���f�����Ƃ̂����蔻�菈��
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Process();
	}
}


// �R���X�g���N�^
MainMove1::MainMove1()
{
	// ���f���ǂݍ���
	LoadFile::MyLoad("media\\�����\\sphere.fyn", enemyModel, ELOADFILE::fbxmodel);

	stage = new Stage();
	character = new Character(stage->GetCollStageHandle());
	camera = new Camera(character->GetArea(), stage->GetCollStageHandle());
	light = new PointLight();

	std::random_device rnd;     // �񌈒�I�ȗ���������𐶐�
	std::mt19937 mt(rnd());     //  �����Z���k�E�c�C�X�^��32�r�b�g�ŁA�����͏����V�[�h�l
	std::uniform_int_distribution<> randInX(-11020, 11020);        // [0, 99] �͈͂̈�l����
	std::uniform_int_distribution<> randInZ(-11020, 11020);        // [0, 99] �͈͂̈�l����
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i] = new EnemyMove1(enemyModel, stage->GetCollStageHandle(), randInX(mt), randInZ(mt));
	}

	stage->LoadInit();

	BaseMove::ShadowNoMoveSetUpBefore();
	stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}

// �f�X�g���N�^
MainMove1::~MainMove1()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		delete enemyAggre[i];
	}
	delete light;
	delete camera;
	delete character;
	delete stage;

	MV1DeleteModel(enemyModel);
}


// �`��
void MainMove1::Draw()
{
	BaseMove::ShadowCharaSetUpBefore();
	character->Draw();
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Draw();
	}
	BaseMove::ShadowCharaSetUpAfter();
	
	BaseMove::ShadowAnotherCharaSetUpBefore();
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Draw();
	}
	BaseMove::ShadowAnotherCharaSetUpAfter();

	BaseMove::ShadowCharaDrawBefore();
	BaseMove::ShadowAnotherCharaDrawBefore();
	BaseMove::ShadowNoMoveDrawBefore();
	character->Draw();
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Draw();
	}
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
