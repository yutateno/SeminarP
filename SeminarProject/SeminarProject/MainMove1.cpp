#include "MainMove1.hpp"

// モデルごとのあたり判定処理
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()) <= 60
			&& enemyAggre[i].aliveNow == true)
		{
			enemyAggre[i].enemyMove->ViewLost();
			enemyAggre[i].aliveNow = false;
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


// コンストラクタ
MainMove1::MainMove1(std::vector<int> file)
{
	SetLightEnable(FALSE);

	// フォグを有効にする
	SetFogEnable(TRUE);

	// フォグの色にする
	SetFogColor(128, 128, 128);

	// フォグの開始距離
	SetFogStartEnd(8000.0f, 10000.0f);

	stage = new Stage(file[0]);
	character = new Character(file[2], file[1]);
	camera = new Camera(character->GetArea(), file[1]);

	std::random_device rnd;     // 非決定的な乱数生成器を生成
	std::mt19937 mt(rnd());     //  メルセンヌ・ツイスタの32ビット版、引数は初期シード値
	std::uniform_int_distribution<> randInX(-4000, 4000);        // 一様乱数
	std::uniform_int_distribution<> randInZ(-4000, 4000);        // 一様乱数
	std::uniform_int_distribution<> color(1, 100);				 // 一様乱数
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove = new EnemyMove1(file[1], (float)randInX(mt), (float)randInZ(mt), (float)color(mt) / 100.0f);
		enemyAggre[i].aliveNow = true;
	}

	stage->Draw();

	// ライトに関する
	lightArea = VAdd(character->GetArea(), VGet(0.0f, 100.0f, 0.0f));
	for (int i = 0; i != lightNum; ++i)
	{
		lightRange[i] = 1000.0f;
		lightHandle[i] = CreatePointLightHandle(lightArea, lightRange[i], 0.0f, 0.002f, 0.0f);
	}

	////BaseMove::ShadowNoMoveSetUpBefore();
	//stage->Draw();
	//BaseMove::ShadowNoMoveSetUpAfter();
}

// デストラクタ
MainMove1::~MainMove1()
{
	for (int i = 0; i != lightNum; ++i)
	{
		if (lightHandle[i] != -1)
		{
			DeleteLightHandle(lightHandle[i]);
		}
	}
	for (int i = 0; i < enemyNum; ++i)
	{
		delete enemyAggre[i].enemyMove;
	}
	delete camera;
	delete character;
	delete stage;
}


// 描画
void MainMove1::Draw()
{
	DrawBox(0, 0, 1920, 1080, GetColor(0, 0, 0), true);

	//ShadowDraw();
	stage->Draw();
	character->Draw();
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove->Draw();
	}
	
#ifdef _SEARCH_MODEL_DEBUG
	for (int i = 0; i < enemyNum; ++i)
	{
		//DrawFormatString(0, i * 16, GetColor(255, 255, 255), "%d", BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()));
		if (BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()) <= 500)
		{
			DrawLine3D(VAdd(character->GetArea(), VGet(0.0f, 80.0f, 0.0f)), VAdd(enemyAggre[i].enemyMove->GetArea(), VGet(0.0f, 60.0f, 0.0f)), GetColor(255, 0, 0));
		}
	}
#endif

	printfDx("%f\t%d\n", lightRange, GetEnableLightHandleNum());
}

// メインプロセス
void MainMove1::Process(unsigned __int8 controllNumber)
{
	character->Process(controllNumber, camera->GetAngle());
	camera->Process(character->GetArea(), controllNumber);
	for (int i = 0; i < enemyNum; ++i)
	{
		enemyAggre[i].enemyMove->Process();
	}
	ActorHit();

	BaseMove::ShadowArea(character->GetArea());

	
	if (KeyData::Get(KEY_INPUT_H) == 1)
	{
		SetLightEnable(TRUE);
	}
	if (KeyData::Get(KEY_INPUT_I) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightRange[i] += 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_K) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightRange[i] -= 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_W) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea.z += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea);
		}
	}
	if (KeyData::Get(KEY_INPUT_S) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea.z -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea);
		}
	}
	if (KeyData::Get(KEY_INPUT_A) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea.x -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea);
		}
	}
	if (KeyData::Get(KEY_INPUT_D) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea.x += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea);
		}
	}
}
