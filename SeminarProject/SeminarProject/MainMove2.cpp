#include "MainMove2.hpp"



void MainMove2::ShadowDraw()
{
	BaseMove::ShadowCharaSetUpBefore();
	p_character->Draw();
	p_enemy->Draw();
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i]->Draw();
	}
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i]->Draw();
	}
	BaseMove::ShadowCharaSetUpAfter();


	BaseMove::ShadowAnotherCharaSetUpBefore();
	p_enemy->Draw();
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i]->Draw();
	}
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i]->Draw();
	}
	BaseMove::ShadowAnotherCharaSetUpAfter();


	BaseMove::ShadowNoMoveDrawBefore();
	BaseMove::ShadowAnotherCharaDrawBefore();
	BaseMove::ShadowCharaDrawBefore();
	p_stage->Draw();
	p_enemy->Draw();
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i]->Draw();
	}
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i]->Draw();
	}
	p_character->Draw();
	BaseMove::ShadowNoMoveDrawAfter();
	BaseMove::ShadowAnotherCharaDrawAfter();
	BaseMove::ShadowCharaDrawAfter();
}

MainMove2::MainMove2(std::vector<int> v_file)
{
	// ポインタNULL初期化
	p_camera					 = NULL;
	p_character					 = NULL;
	p_enemy						 = NULL;
	p_stage						 = NULL;
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i]		 = NULL;
	}
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i]	 = NULL;
	}


	// ポインタ初期化
	p_stage		 = new Stage(v_file[EFILE::stage]);
	p_character	 = new CharacterSword(v_file[EFILE::characterAttack], v_file[EFILE::stage]);
	p_camera	 = new Camera(p_character->GetArea(), v_file[EFILE::stage]);
	p_enemy		 = new EnemyMove2(v_file[EFILE::stage], VGet(1000.0f, 0.0f, 1000.0f), v_file[EFILE::block]);
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i] = new StageStairs(v_file[EFILE::stairs], v_file[EFILE::stage], VGet(-1000.0f*i, 0.0f, -1000.0f));
	}
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i] = new StageStreetLight(v_file[EFILE::streetLight], v_file[EFILE::stage], VGet(250.0f*i, 0.0f, -100.0f*i));
	}


	BaseMove::ShadowNoMoveSetUpBefore();
	p_stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}


MainMove2::~MainMove2()
{
	for (int i = 0; i != 30; ++i)
	{
		if (p_stageStreetLight[i] != NULL)
		{
			delete p_stageStreetLight[i];
			p_stageStreetLight[i] = NULL;
		}
	}
	for (int i = 0; i != 10; ++i)
	{
		if (p_stageStairs[i] != NULL)
		{
			delete p_stageStairs[i];
			p_stageStairs[i] = NULL;
		}
	}
	if (p_enemy != NULL)
	{
		delete p_enemy;
		p_enemy = NULL;
	}
	if (p_camera != NULL)
	{
		delete p_camera;
		p_camera = NULL;
	}
	if (p_character != NULL)
	{
		delete p_character;
		p_character = NULL;
	}
	if (p_stage != NULL)
	{
		delete p_stage;
		p_stage = NULL;
	}
}


// 描画
void MainMove2::Draw()
{
	ShadowDraw();


#ifdef _MOVE1_DEBUG
#ifdef _SEARCH_MODEL_DEBUG
	for (int i = 0; i < enemyNum; ++i)
	{
		//DrawFormatString(0, i * 16, GetColor(255, 255, 255), "%d", BaseMove::GetDistance(character->GetArea(), enemyAggre[i].enemyMove->GetArea()));
		if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 500)
		{
			DrawLine3D(VAdd(p_character->GetArea(), VGet(0.0f, 80.0f, 0.0f)), VAdd(s_enemyAggre[i].p_enemyMove->GetArea(), VGet(0.0f, 60.0f, 0.0f)), GetColor(255, 0, 0));
		}
	}
#endif
	//printfDx("NUM:%d\tCOUNT:%d\tX:%f\tY:%f\tZ:%f\n", catchEnemyNum, lightEventCount, p_character->GetArea().x, p_character->GetArea().y, p_character->GetArea().z);
#endif // _MOVE1_DEBUG

}


// メインプロセス
void MainMove2::Process(unsigned __int8 controllNumber)
{
	p_character->Process(controllNumber, p_camera->GetAngle());		// キャラクターのプロセスを呼ぶ


	p_camera->Process(p_character->GetArea(), controllNumber);		// カメラのプロセスを呼ぶ


	p_enemy->Process();												// 敵のプロセス


	BaseMove::ShadowArea(p_character->GetArea());
}