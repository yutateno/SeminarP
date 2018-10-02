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


//void MainMove2::AttackProcess()
//{
//	VECTOR ChkChToChVec;
//	VECTOR PushVec;
//	VECTOR ChPosition;
//	float Length;
//
//	// 移動後の ch の座標を算出
//	ChPosition = p_enemy->GetArea();
//
//	// 当たっていなかったら何もしない
//	if (HitCheck_Capsule_Capsule(
//		p_character->GetArea(), VAdd(p_character->GetArea(), VGet(0.0f, 160.0f, 0.0f)), 50.0f,
//		p_enemy->GetArea(), VAdd(p_enemy->GetArea(), VGet(0.0f, 10.0f, 0.0f)), 10.0f) == TRUE)
//	{
//		// 当たっていたら ch が chk から離れる処理をする
//
//		// chk_ch から ch へのベクトルを算出
//		ChkChToChVec = VSub(p_character->GetArea(), p_enemy->GetArea());
//
//		// Ｙ軸は見ない
//		ChkChToChVec.y = 0.0f;
//
//		// 二人の距離を算出
//		Length = VSize(ChkChToChVec);
//
//		// chk_ch から ch へのベクトルを正規化( ベクトルの長さを 1.0f にする )
//		PushVec = VScale(ChkChToChVec, 1.0f / Length);
//
//		// 押し出す距離を算出、もし二人の距離から二人の大きさを引いた値に押し出し力を足して離れてしまう場合は、ぴったりくっつく距離に移動する
//		if (Length - (50.0f + 10.0f) + 30.0f > 0.0f)
//		{
//			float TempY;
//
//			TempY = ChPosition.y;
//			p_enemy->SetArea(VAdd(p_enemy->GetArea(), VScale(PushVec, (50.0f + 10.0f))));
//
//			// Ｙ座標は変化させない
//			p_enemy->SetArea(VGet(p_enemy->GetArea().x, TempY, p_enemy->GetArea().z));
//		}
//		else
//		{
//			// 押し出し
//			p_enemy->SetArea(VAdd(ChPosition, VScale(PushVec, 60.0f)));
//		}
//	}
//}


MainMove2::MainMove2(const std::vector<int> v_file)
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
	for (int i = 0; i != 10; ++i)
	{
		p_stageStairs[i] = new StageStairs(v_file[EFILE::stairs], v_file[EFILE::stage], VGet(-100.0f*i, 0.0f, -1000.0f));
	}
	p_stage		 = new Stage(v_file[EFILE::stage]);
	p_character	 = new CharacterSword(v_file[EFILE::characterAttack], v_file[EFILE::stage], v_file[EFILE::stairsColl]);
	p_camera	 = new Camera(p_character->GetArea(), v_file[EFILE::stage]);
	p_enemy		 = new EnemyMove2(v_file[EFILE::stage], VGet(1000.0f, 0.0f, 1000.0f), v_file[EFILE::block]);
	for (int i = 0; i != 30; ++i)
	{
		p_stageStreetLight[i] = new StageStreetLight(v_file[EFILE::streetLight], v_file[EFILE::stage], VGet(250.0f*i, 0.0f, -100.0f*i));
	}


	// スカイボックス読み込み
	BaseMove::SetInitSkyBox(v_file[EFILE::skyBox]);


	// 階段のあたり判定
	for (int i = 0; i != 10; ++i)
	{
		p_character->SetStairsArea(p_stageStairs[i]->GetArea(), i);
	}

	BaseMove::ShadowNoMoveSetUpBefore();
	p_stage->Draw();
	BaseMove::ShadowNoMoveSetUpAfter();
}


MainMove2::~MainMove2()
{
	for (int i = 0; i != 30; ++i)
	{
		POINTER_RELEASE(p_stageStreetLight[i]);
	}
	for (int i = 0; i != 10; ++i)
	{
		POINTER_RELEASE(p_stageStairs[i]);
	}
	POINTER_RELEASE(p_enemy);
	POINTER_RELEASE(p_camera);
	POINTER_RELEASE(p_character);
	POINTER_RELEASE(p_stage);
}


// 描画
void MainMove2::Draw()
{
	//DrawGraph(0, 0, backGround, false);

	
	BaseMove::SkyBoxDraw();


	ShadowDraw();


	p_character->Draw();


#ifdef _MOVE2_DEBUG
	printfDx("%f\t%f\t%f\n", p_character->GetArea().x, p_character->GetArea().y, p_character->GetArea().z);
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
#endif // _MOVE2_DEBUG
}


// メインプロセス
void MainMove2::Process(const unsigned __int8 controllNumber)
{
	//GetDrawScreenGraph(0, 0, 1920, 1080, backGround);						// 現在の画面をキャプチャする
	//GraphFilter(backGround, DX_GRAPH_FILTER_GAUSS, 8, 1400);				// 現在の画面にガウスフィルタかけてぼかす

	p_character->Process(controllNumber, p_camera->GetAngle());		// キャラクターのプロセスを呼ぶ


	p_camera->Process(p_character->GetArea(), controllNumber);		// カメラのプロセスを呼ぶ


	p_enemy->Process();												// 敵のプロセス


	BaseMove::ShadowArea(p_character->GetArea());

	//AttackProcess();

	BaseMove::SkyBoxProcess(p_character->GetArea());
}


void MainMove2::CameraProcess()
{
	p_camera->SetUp();		// カメラのプロセスを呼ぶ
}