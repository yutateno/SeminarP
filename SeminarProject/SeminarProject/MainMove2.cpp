#include "MainMove2.hpp"



void MainMove2::ShadowDraw()
{
	BaseMove::ShadowCharaSetUpBefore();
	p_character->Draw();
	for (int i = 0; i != 50; ++i)
	{
		p_person[i]->Draw();
	}
	BaseMove::ShadowCharaSetUpAfter();


	BaseMove::ShadowAnotherCharaSetUpBefore();
	for (int i = 0; i != 50; ++i)
	{
		p_person[i]->Draw();
	}
	BaseMove::ShadowAnotherCharaSetUpAfter();


	BaseMove::ShadowNoMoveDrawBefore();
	BaseMove::ShadowAnotherCharaDrawBefore();
	BaseMove::ShadowCharaDrawBefore();
	p_stage->Draw();
	for (int i = 0; i != 50; ++i)
	{
		p_person[i]->Draw();
	}
	p_character->Draw();
	BaseMove::ShadowNoMoveDrawAfter();
	BaseMove::ShadowAnotherCharaDrawAfter();
	BaseMove::ShadowCharaDrawAfter();
}

MainMove2::MainMove2(std::vector<int> v_file)
{
	// ポインタNULL初期化
	p_camera	 = NULL;
	p_character	 = NULL;
	for (int i = 0; i != 50; ++i)
	{
		p_person[i] = NULL;
	}
	p_stage		 = NULL;


	// ポインタ初期化
	p_stage		 = new Stage(v_file[EFILE::stage]);
	p_character	 = new CharacterSword(v_file[EFILE::characterAttack], v_file[EFILE::stage]);
	p_camera	 = new Camera(p_character->GetArea(), v_file[EFILE::stage]);
	for (int i = 0; i != 50; ++i)
	{
		p_person[i] = new OrdinaryPerson(v_file[EFILE::character], v_file[EFILE::stage]);
	}
	

	BaseMove::ShadowNoMoveSetUpBefore();
	p_stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}


MainMove2::~MainMove2()
{
	for (int i = 0; i != 50; ++i)
	{
		if (p_person[i] != NULL)
		{
			delete p_person[i];
			p_person[i] = NULL;
		}
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


	for (int i = 0; i != 50; ++i)
	{
		p_person[i]->Process();
	}
	//p_person->Process();												// 敵のプロセス


	BaseMove::ShadowArea(p_character->GetArea());
}