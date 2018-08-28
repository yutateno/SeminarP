#include "MainMove1.hpp"

// モデルごとのあたり判定処理
void MainMove1::ActorHit()
{
	enemy->Process();
}


// コンストラクタ
MainMove1::MainMove1()
{
	stage = new Stage();
	character = new Character(stage->GetCollStageHandle());
	enemy = new EnemyMove1(stage->GetCollStageHandle(), 100.0f, 0.0f);
	camera = new Camera(character->GetArea(), stage->GetCollStageHandle());

	stage->LoadInit();
}

// デストラクタ
MainMove1::~MainMove1()
{
	delete camera;
	delete enemy;
	delete character;
	delete stage;
}


// 描画
void MainMove1::Draw()
{
	stage->Draw();
	character->Draw();
	enemy->Draw();
}

// メインプロセス
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	ActorHit();
}
