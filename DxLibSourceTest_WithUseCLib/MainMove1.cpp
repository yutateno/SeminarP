#include "MainMove1.hpp"

// モデルごとのあたり判定処理
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i]->Process();
	}
}


// コンストラクタ
MainMove1::MainMove1()
{
	// モデル読み込み
	LoadFile::MyLoad("media\\光る玉\\sphere.fyn", enemyModel, ELOADFILE::fbxmodel);

	stage = new Stage();
	character = new Character(stage->GetCollStageHandle());
	camera = new Camera(character->GetArea(), stage->GetCollStageHandle());
	light = new PointLight();

	std::random_device rnd;     // 非決定的な乱数生成器を生成
	std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
	std::uniform_int_distribution<> randInX(-11020, 11020);        // [0, 99] 範囲の一様乱数
	std::uniform_int_distribution<> randInZ(-11020, 11020);        // [0, 99] 範囲の一様乱数
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i] = new EnemyMove1(enemyModel, stage->GetCollStageHandle(), randInX(mt), randInZ(mt));
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

	MV1DeleteModel(enemyModel);
}


// 描画
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

// メインプロセス
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	//light->Process(character->GetArea());
	ActorHit();

	BaseMove::ShadowArea(character->GetArea());
}
