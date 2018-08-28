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
	light = new PointLight();

	stage->LoadInit();
}

// デストラクタ
MainMove1::~MainMove1()
{
	delete light;
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
	//light->Draw(character->GetArea());
}

// メインプロセス
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	//light->Process(character->GetArea());
	ActorHit();
}
