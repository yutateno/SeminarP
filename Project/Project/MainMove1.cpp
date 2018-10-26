#include "MainMove1.hpp"


// モデルごとのあたり判定処理
void MainMove1::ActorHit()
{
	for (int i = 0; i < enemyNum; ++i)
	{
		// 当たった判定になったら
		if (s_enemyAggre[i].aliveNow)
		{
			if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 60)
			{
				s_enemyAggre[i].aliveNow = false;			// 生きさせない
				catchEnemyNum++;				// 取った個数をカウント
			}
			else if (BaseMove::GetDistance(p_character->GetArea(), s_enemyAggre[i].p_enemyMove->GetArea()) <= 300)
			{
				s_enemyAggre[i].p_enemyMove->StolenChara(p_character->GetArea());
			}
		}
	}

	if (catchEnemyNum == 30 && lightEventEnd
		&& BaseMove::GetDistance(p_character->GetArea(), p_dropItem->GetArea()) <= 60)
	{
		touchSword = true;
	}
}



// ライトに関する
void MainMove1::LightProcess()
{
	// 光源の情報を更新させる
	for (int i = 0; i != lightNum; ++i)
	{
		SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);		// 光源の広さを更新
		SetLightPositionHandle(lightHandle[i], lightArea[i]);								// 光源の位置を更新
	}


	// 玉の個数で判断させる
	switch (catchEnemyNum)
	{
	case 1:		// 一つ目を有効にする
		SetLightEnableHandle(lightHandle[0], TRUE);
		break;


	case 2:		// 二つ目を有効にする
		SetLightEnableHandle(lightHandle[1], TRUE);
		break;


	case 3:		// 三つ目を有効にする
		SetLightEnableHandle(lightHandle[2], TRUE);
		break;


	case 4:		// 四つ目を有効にしつつ地面の明るさをじわじわ広げる
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 7.0f;
			lightRangePreMax = 1000.0f;
			lightEventStart = true;
		}
		break;


	case 5:		// フラグを倒す
		lightEventEnd = false;
		break;


	case 7:		// 地面の明るさを広くする
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 9.0f;
			lightRangePreMax = 1700.0f;
			lightEventStart = true;
		}
		break;


	case 8:		// フラグを倒す
		lightEventEnd = false;
		break;


	case 12:	// 地面の明るさを広くする
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 11.0f;
			lightRangePreMax = 2600.0f;
			lightEventStart = true;
		}
		break;


	case 13:	// フラグを倒す
		lightEventEnd = false;
		break;


	case 19:	// 地面の明るさを広くする
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 14.0f;
			lightRangePreMax = 3700.0f;
			lightEventStart = true;
		}
		break;


	case 20:	// フラグを倒す
		lightEventEnd = false;
		break;


	case 29:	// 地面の明るさを広くする
		if (!lightEventStart && !lightEventEnd)
		{
			lightEventCount = 0;
			lightRangeSpeed = 20.0f;
			lightRangePreMax = 5100.0f;
			lightEventStart = true;
		}
		break;


	case 30:	// 光源を消して自然光源を通常にする
		if (!lightEnd && !lightEventStart)
		{
			lightEventCount = 0;
			lightEventStart = true;
			lightEnd = true;
		}
		break;


	default:
		break;
	}


	// イベントを開始する
	if (lightEventStart)
	{
		lightEventCount++;		// カウントを続ける

		// 光源の明るさを広げる
		lightRange[0] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[1] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[2] = lightRangePreMax + lightEventCount * lightRangeSpeed;
		lightRange[3] = lightRangePreMax + lightEventCount * lightRangeSpeed;


		// カウントが規定以上になったら
		if (lightEventCount >= 100)
		{
			lightEventStart = false;		// イベントの開始のフラグを倒す
			lightEventEnd = true;			// イベントの終了のフラグを上げる


			// 全ての玉を手に入れたら
			if (catchEnemyNum == 30)
			{
				for (int i = 0; i != lightNum; ++i)
				{
					DeleteLightHandle(lightHandle[i]);		// 光源を消す
					lightHandle[i] = 0;
					SetLightEnable(TRUE);					// 自然光源を有効にする
					p_character->PositionReset();			// キャラクターのポジションを元に戻す
				}
			}
		}


		// 個数によるイベントの違い
		switch (catchEnemyNum)
		{
		case 4:			// 地面を明るくする
			if (lightEventCount >= 10)
			{
				SetLightEnableHandle(lightHandle[3], TRUE);		// 四つ目の光源を出す
			}
			break;


		case 12:		// 背景を明るくする
			backgroundColor = GetColor(lightEventCount, lightEventCount, lightEventCount);
			break;


		case 19:		// 背景を明るくしつつ光源をカメラの方向に動かす
			lightArea[0].z = -630.0f + lightEventCount * 5;
			lightArea[1].z = -630.0f + lightEventCount * 4;
			lightArea[2].z = -630.0f + lightEventCount * 3;
			lightArea[3].z = -630.0f + lightEventCount * 2;
			backgroundColor = GetColor(lightEventCount / 2 + 100, lightEventCount / 2 + 100, lightEventCount / 2 + 100);
			break;


		case 29:		// 背景を明るくする
			backgroundColor = GetColor(lightEventCount / 2 + 150, lightEventCount / 2 + 150, lightEventCount / 2 + 150);
			break;


		case 30:		// 背景を明るくしつつまぶしくさせてる間に光源を消して自然光源に切り替える
			backgroundColor = GetColor(lightEventCount + 155, lightEventCount + 155, lightEventCount + 155);
			// フェードアウトの処理をさせてまぶしくする
			if (lightEventCount < 50)
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, 255 - (lightEventCount * 2));
				//DrawGraph(0, 0, drawWhite, false);
				DrawBox(0, 0, 1920, 1080, GetColor(255, 255, 255), true);
				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 255);
			}
			else		// フェードインの処理をさせて戻す
			{
				SetDrawBlendMode(DX_BLENDMODE_ALPHA, 155 + ((lightEventCount - 50) * 2));
				//DrawGraph(0, 0, drawWhite, false);
				DrawBox(0, 0, 1920, 1080, GetColor(255, 255, 255), true);
				SetDrawBlendMode(DX_BLENDMODE_NOBLEND, 255);
			}
			break;

		}
	}
}


// コンストラクタ
MainMove1::MainMove1(const std::vector<int> v_file)
{
	SetLightEnable(FALSE);		// 自然光源を一切遮断


	// フォグに関する
	SetFogEnable(FALSE);					// フォグを有効にする
	//SetFogColor(128, 128, 128);			// フォグの色にする
	//SetFogStartEnd(8000.0f, 10000.0f);	// フォグの開始距離


	// 背景色に関する
	backgroundColor = GetColor(0, 0, 0);


	// ポインタNULL初期化
	p_camera = NULL;
	p_character = NULL;
	p_stage = NULL;
	p_dropItem = NULL;
	for (int i = 0; i != enemyNum; ++i)
	{
		s_enemyAggre[i].p_enemyMove = NULL;
	}


	// 敵以外のポインタの初期化
	p_stage = new Stage(v_file[EFILE::drawStage]);									// ステージ初期化
	p_character = new Character(v_file[EFILE::character], v_file[EFILE::collStage]);		// キャラクター初期化
	p_camera = new Camera(p_character->GetArea(), v_file[EFILE::collStage]);			// カメラ初期化
	p_dropItem = new DropItemMove1(v_file[EFILE::sword], v_file[EFILE::collStage]);


	// 敵生成に関する
	std::random_device rnd;     // 非決定的な乱数生成器を生成
	std::mt19937 mt(rnd());     // メルセンヌ・ツイスタの32ビット版
	std::uniform_int_distribution<> randInX(-4000, 4000);        // X座標用乱数
	std::uniform_int_distribution<> randInZ(-4000, 4000);        // Z座標用乱数
	std::uniform_int_distribution<> color(1, 100);				 // 色用の乱数
	for (int i = 0; i != enemyNum; ++i)
	{
		s_enemyAggre[i].aliveNow = true;
		s_enemyAggre[i].p_enemyMove = new EnemyMove1(v_file[1], (float)randInX(mt), (float)randInZ(mt), color(mt) / 100.0f);		// 敵初期化
	}
	catchEnemyNum = 0;		// 敵を手に入れた数を初期化
	;

	// ライトに関する
	for (int i = 0; i != lightNum; ++i)
	{
		lightHandle[i] = 0;
		lightArea[i] = p_character->GetArea();						// ライトの位置初期化
		lightRange[i] = 1700.0f;
		lightArea[0].y = -200.0f;
		lightArea[i].z = -630.0f;															// ライトの大きさ初期化
		lightHandle[i] = CreatePointLightHandle(lightArea[i], lightRange[i], 0.0f, 0.002f, 0.0f);	// ライトを生成
		SetLightEnableHandle(lightHandle[i], FALSE);
	}
	lightEventStart = false;
	lightEventEnd = false;
	lightEventCount = 0;
	lightRangePreMax = 0.0f;
	lightRangeSpeed = 0.0f;
	lightEnd = false;


	touchSword = false;


	soundBG = v_file[EFILE::sound];
	ChangeVolumeSoundMem(150, soundBG);
	PlaySoundMem(soundBG, DX_PLAYTYPE_LOOP);
}


// デストラクタ
MainMove1::~MainMove1()
{
	SOUND_RELEASE(soundBG);
	for (int i = 0; i != lightNum; ++i)
	{
		LIGHT_RELEASE(lightHandle[i]);
	}
	for (int i = 0; i < enemyNum; ++i)
	{
		POINTER_RELEASE(s_enemyAggre[i].p_enemyMove);
	}
	POINTER_RELEASE(p_dropItem);
	POINTER_RELEASE(p_camera);
	POINTER_RELEASE(p_character);
	POINTER_RELEASE(p_stage);
}


// 描画
void MainMove1::Draw()
{
	DrawBox(0, 0, 1920, 1080, backgroundColor, true);	// 背景を描画


	p_stage->Draw();					// ステージを描画


	p_character->Draw();				// キャラクターを描画


	for (int i = 0; i < enemyNum; ++i)
	{
		if (s_enemyAggre[i].aliveNow)
		{
			s_enemyAggre[i].p_enemyMove->Draw();		// 敵を描画
		}
	}


	if (catchEnemyNum == 30)
	{
		p_dropItem->Draw();				// 剣を描画
	}

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
void MainMove1::Process(const unsigned __int8 controllNumber)
{
	p_character->Process(controllNumber, p_camera->GetAngle());		// キャラクターのプロセスを呼ぶ


	p_camera->Process(p_character->GetArea(), controllNumber);		// カメラのプロセスを呼ぶ


	for (int i = 0; i < enemyNum; ++i)
	{
		// 生きていたら
		if (s_enemyAggre[i].aliveNow)
		{
			s_enemyAggre[i].p_enemyMove->Process();					// 敵のプロセスを呼ぶ
		}
	}


	ActorHit();		// アクターごとのあたり判定を呼ぶ


	LightProcess();		// ライトのプロセスを呼ぶ


#ifdef _DEBUG
	DebugKeyControll();
#endif // _DEBUG
}


void MainMove1::CameraProcess()
{
	p_camera->SetUp();
}



#ifdef _DEBUG
void MainMove1::DebugKeyControll()
{
	if (KeyData::Get(KEY_INPUT_Z) == 1)
	{
		if (catchEnemyNum != 30)
		{
			catchEnemyNum++;
		}
		s_enemyAggre[catchEnemyNum - 1].aliveNow = false;
	}
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
	if (KeyData::Get(KEY_INPUT_R) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].y += 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_F) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].y -= 10.0f;
			SetLightRangeAttenHandle(lightHandle[i], lightRange[i], 0.0f, 0.002f, 0.0f);
		}
	}
	if (KeyData::Get(KEY_INPUT_W) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].z += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_S) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].z -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_A) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].x -= 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}
	if (KeyData::Get(KEY_INPUT_D) >= 1)
	{
		for (int i = 0; i != lightNum; ++i)
		{
			lightArea[i].x += 10.0f;
			SetLightPositionHandle(lightHandle[i], lightArea[i]);
		}
	}

	if (KeyData::Get(KEY_INPUT_K) == 1)
	{
		scene = ESceneNumber::SECONDLOAD;
	}
}
#endif // _DEBUG
