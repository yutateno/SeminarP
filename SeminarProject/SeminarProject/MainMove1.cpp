#include "MainMove1.hpp"

// ���f�����Ƃ̂����蔻�菈��
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()) <= 60
			&& enemyAggre[i].aliveNow == true)
		{
			enemyAggre[i].enemyMove->SetViewNow(false);
			enemyAggre[i].aliveNow = false;
			if (enemyAggre[i].aliveNow == false)
			{
				character->DoEnemyCatchNum();
			}
		}
	}
}

//void MainMove1::ShadowDraw()
//{
//	BaseMove::ShadowCharaSetUpBefore();
//	character->Draw();
//	for (int i = 0; i < enemyNum; ++i)
//	{
//		enemyAggre[i]->Draw();
//	}
//	BaseMove::ShadowCharaSetUpAfter();
//
//	BaseMove::ShadowAnotherCharaSetUpBefore();
//	for (int i = 0; i < enemyNum; ++i)
//	{
//		enemyAggre[i]->Draw();
//	}
//	BaseMove::ShadowAnotherCharaSetUpAfter();
//
//	BaseMove::ShadowCharaDrawBefore();
//	BaseMove::ShadowAnotherCharaDrawBefore();
//	BaseMove::ShadowNoMoveDrawBefore();
//	character->Draw();
//	for (int i = 0; i < enemyNum; ++i)
//	{
//		enemyAggre[i]->Draw();
//	}
//	stage->Draw();
//	BaseMove::ShadowCharaDrawAfter();
//	BaseMove::ShadowAnotherCharaDrawAfter();
//	BaseMove::ShadowNoMoveDrawAfter();
//}


// �R���X�g���N�^
MainMove1::MainMove1(std::vector<int> file)
{
	stage = new Stage(file[0]);
	character = new Character(file[2], file[1]);
	camera = new Camera(character->GetArea(), file[1]);
	light = new PointLight();

	std::random_device rnd;     // �񌈒�I�ȗ���������𐶐�
	std::mt19937 mt(rnd());     //  �����Z���k�E�c�C�X�^��32�r�b�g�ŁA�����͏����V�[�h�l
	std::uniform_int_distribution<> randInX(-4000, 4000);        // ��l����
	std::uniform_int_distribution<> randInZ(-4000, 4000);        // ��l����
	std::uniform_int_distribution<> color(1, 100);				 // ��l����
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove = new EnemyMove1(file[1], (float)randInX(mt), (float)randInZ(mt), (float)color(mt) / 100.0f);
		enemyAggre[i].aliveNow = true;
	}


	////BaseMove::ShadowNoMoveSetUpBefore();
	//stage->Draw();
	//BaseMove::ShadowNoMoveSetUpAfter();
}

// �f�X�g���N�^
MainMove1::~MainMove1()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		delete enemyAggre[i].enemyMove;
	}
	delete light;
	delete camera;
	delete character;
	delete stage;
}


// �`��
void MainMove1::Draw()
{
	// �t�H�O��L���ɂ���
	SetFogEnable(TRUE);

	// �t�H�O�̐F�����F�ɂ���
	SetFogColor(0, 0, 0);

	// �t�H�O�̊J�n����
	SetFogStartEnd(8000.0f, 10000.0f);

	//ShadowDraw();

	stage->Draw();
	character->Draw();
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove->Draw();
	}

	//light->Draw(character->GetArea());
#ifdef _SEARCH_MODEL_DEBUG
	for (int i = 0; i < enemyNum; ++i)
	{
		DrawFormatString(0, i * 16, GetColor(255, 255, 255), "%d", BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()));
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()) <= 500)
		{
			DrawLine3D(VAdd(character->GetArea(), VGet(0.0f, 80.0f, 0.0f)), VAdd(enemyAggre[i].enemyMove->GetArea(), VGet(0.0f, 60.0f, 0.0f)), GetColor(255, 0, 0));
		}
	}
#endif
}

// ���C���v���Z�X
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove->Process();
	}
	//light->Process(character->GetArea());
	ActorHit();

	BaseMove::ShadowArea(character->GetArea());
}
