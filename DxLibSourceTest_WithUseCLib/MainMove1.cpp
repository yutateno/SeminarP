#include "MainMove1.hpp"

// モデルごとのあたり判定処理
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i]->GetArea()) <= 60)
		{
			enemyAggre[i]->SetViewNow(false);
		}
	}
}


// コンストラクタ
MainMove1::MainMove1()
{
	stage = new Stage();
	character = new Character(stage->GetCollStageHandle());
	camera = new Camera(character->GetArea(), stage->GetCollStageHandle());
	light = new PointLight();

	std::random_device rnd;     // 非決定的な乱数生成器を生成
	std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
	std::uniform_int_distribution<> randInX(-4000, 4000);        // 一様乱数
	std::uniform_int_distribution<> randInZ(-4000, 4000);        // 一様乱数
	std::uniform_int_distribution<> color(1, 100);				 // 一様乱数
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i] = new EnemyMove1(stage->GetCollStageHandle(), (float)randInX(mt), (float)randInZ(mt), (float)color(mt) / 100.0f);
	}

	stage->LoadInit();

	BaseMove::ShadowNoMoveSetUpBefore();
	stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}

// デストラクタ
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
}


// 描画
void MainMove1::Draw()
{
	// フォグを有効にする
	SetFogEnable(TRUE);

	// フォグの色を黄色にする
	SetFogColor(0, 0, 0);

	// フォグの開始距離
	SetFogStartEnd(8000.0f, 10000.0f);

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
#ifdef _SEARCH_MODEL_DEBUG
	for (int i = 0; i < enemyNum; ++i)
	{
		DrawFormatString(0, i*16, GetColor(255, 255, 255), "%d", BaseMove::GetDistance(character->GetArea(),enemyAggre[i]->GetArea()));
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i]->GetArea()) <= 500)
		{
			DrawLine3D(VAdd(character->GetArea(), VGet(0.0f,80.0f,0.0f)), VAdd(enemyAggre[i]->GetArea(), VGet(0.0f, 60.0f, 0.0f)), GetColor(255, 0, 0));
		}
	}
#endif
}

// メインプロセス
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Process();
	}
	//light->Process(character->GetArea());
	ActorHit();

	BaseMove::ShadowArea(character->GetArea());
}
