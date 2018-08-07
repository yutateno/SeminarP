#include "Character.hpp"

void Character::Player_PlayAnim(int attach)
{
	// 今のモーションが違うものだったら
	if (attachNum != attach)
	{
		// 直前のモーションが有効だったらデタッチする
		if (preAttach != -1 && motionBlendTime < 1.0f)
		{
			MV1DetachAnim(charamodelhandle, preAttach);
			preAttach = -1;
		}
		// 今のモーションを入れる
		preAttach = attachMotion;
		preMotionPlayTime = nowPlayTime;

		// 引数を今のモーション番号に入れる
		attachNum = attach;

		// 新たに指定のモーションをモデルにアタッチする
		attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, false);

		// 動作時間を初期化する
		nowPlayTime = 0.0f;

		// ブレンド率は直前のモーションが有効ではない場合は１．０ｆ( 再生中のモーション１が１００％の状態 )にする
		if (preMotionPlayTime == -1)
		{
			motionBlendTime = 1.0f;
		}
		else
		{
			motionBlendTime = 0.0f;
		}
	}
}

void Character::Player_AnimProcess()
{
	// ブレンド率が１以下の場合は１に近づける
	if (motionBlendTime < 1.0)
	{
		motionBlendTime += 0.1f;
		if (motionBlendTime >= 1.0f)
		{
			motionBlendTime = 1.0f;
		}
	}

	// 再生している現在のモーションの処理
	if (attachMotion != -1)
	{
		// モーションの総時間を取得
		totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

		// 再生時間を進める
		nowPlayTime += animSpeed;


		// 再生時間が総時間に到達していたらループさせる
		if (nowPlayTime >= totalTime)
		{
			nowPlayTime = MYINPUTPAD::fmodf(nowPlayTime, totalTime);
		}

		// 変更した再生時間をモデルに反映させる
		MV1SetAttachAnimTime(charamodelhandle, attachMotion, nowPlayTime);

		// アニメーション１のモデルに対する反映率をセット
		MV1SetAttachAnimBlendRate(charamodelhandle, attachMotion, motionBlendTime);
	}

	// 再生している直前のモーションの処理
	if (preAttach != -1)
	{
		// アニメーションの総時間を取得
		totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, preAttach);

		// 再生時間を進める
		preMotionPlayTime += animSpeed;

		// 再生時間が総時間に到達していたら再生時間をループさせる
		if (preMotionPlayTime > totalTime)
		{
			preMotionPlayTime = MYINPUTPAD::fmodf(preMotionPlayTime, totalTime);
		}

		// 変更した再生時間をモデルに反映させる
		MV1SetAttachAnimTime(charamodelhandle, preAttach, preMotionPlayTime);

		// アニメーション２のモデルに対する反映率をセット
		MV1SetAttachAnimBlendRate(charamodelhandle, preAttach, (1.0f - motionBlendTime));
	}
}

void Character::StageHit()
{
	// プレイヤーをカプセルとしてステージとのコリジョン情報を調べる(OBB形式)
	hitDim = MV1CollCheck_Capsule(stageHandle, -1, area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth);

	// ポリゴンの数を再初期化
	wallNum = 0;
	floorNum = 0;

	// 検出された数だけ調べる
	for (int i = 0, j = hitDim.HitNum; i != j; ++i)
	{
		// 壁かどうか判断するため、XZ軸に垂直かどうかを法線が０に近いかどうかで調べる
		if (hitDim.Dim[i].Normal.y < 0.1f && hitDim.Dim[i].Normal.y > -0.1f)
		{
			// 壁だとしてもキャラクターの足より少し上を調べる
			if (hitDim.Dim[i].Position[0].y > area.y + 10.0f
				|| hitDim.Dim[i].Position[1].y > area.y + 10.0f
				|| hitDim.Dim[i].Position[2].y > area.y + 10.0f)
			{
				// 最大になるまで保存する
				if (wallNum < 2048)
				{
					wallPoly[wallNum] = &hitDim.Dim[i];		// ヒットしたポリゴン情報を保存
					wallNum++;
				}
			}
		}
		else
		{
			// 最大になるまで保存
			if (floorNum < 2048)
			{
				floorPoly[floorNum] = &hitDim.Dim[i];		// ヒットしたポリゴン情報を保存
				floorNum++;
			}
		}
	}

	// 壁判定
	if (wallNum != 0)
	{
		hitFlag = false;	// 壁には当たってないとする

		// 移動しているかどうかで判断を変える
		if (moveFlag)
		{
			for (int i = 0; i != wallNum; ++i)
			{
				int j;		// 判定をするためループ外で宣言

				mainPoly = wallPoly[i];			// 今の調べるポリゴン情報を渡す
				
				// 当たっているかどうかを調べる
				if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == false)
				{
					continue;			// 当たっていないので次！
				}

				hitFlag = true;			// 当たっているとする

				// 移動しているので次の座標で当たっていないか判定
				for (j = 0; j != wallNum; ++j)
				{
					mainPoly = wallPoly[j];			// 今の調べるポリゴン情報を渡す

					// 当たっているかどうかを調べる
					if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
					{
						break;			// 当たっていたので抜ける
					}
				}

				// 移動先で当たらなかったので判定を消す
				if (j == wallNum)
				{
					hitFlag = false;
					break;
				}
			}
		}
		else
		{
			for (int i = 0; i != wallNum; ++i)
			{
				mainPoly = wallPoly[i];			// 今の調べるポリゴン情報を渡す

				// 当たっているかどうかを調べる
				if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
				{
					hitFlag = true;
					break;			// 当たっていたので抜ける
				}
			}
		}

		// 当たっていたので押し出す
		if (hitFlag)
		{
			int j, k;
			// 判定数を32とする
			for (int i = 0; i != 32; ++i)
			{
				// 壁ポリゴンの数だけ繰り返し
				for (j = 0; j != wallNum; ++j)
				{
					mainPoly = wallPoly[j];

					// 当たっているかどうかを調べる
					if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == false)
					{
						continue;
					}

					area = VAdd(area, VScale(mainPoly->Normal, walkSpeed / 3.0f));		// 当たっていたら規定距離文プレイヤーを壁の法線方向に移動させる

					for (k = 0; k != wallNum; ++k)
					{
						// 当たっていたらループを抜ける
						mainPoly = wallPoly[k];
						if (HitCheck_Capsule_Triangle(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]) == TRUE)
						{
							break;
						}
					}

					// 全てのポリゴンと当たっていなかったので抜ける
					if (k == wallNum)
					{
						break;
					}
				}

				// 判定を調べる前に全てのポリゴンと当たっていなかったので抜ける
				if (j != wallNum)
				{
					break;
				}
			}
		}
	}

	// 床判定
	if (floorNum != 0)
	{
		hitFlag = false;		// 当たってないとする

		maxYHit = 0.0f;			// 初期化する

		for (int i = 0; i != floorNum; ++i)
		{
			mainPoly = floorPoly[i];			// 今調べるポリゴン情報を渡す

			lineResult = HitCheck_Line_Triangle(VAdd(area, VGet(0.0f, modelHeight, 0.0f)), VAdd(area, VGet(0.0f, -5.0f, 0.0f)), mainPoly->Position[0], mainPoly->Position[1], mainPoly->Position[2]);

			// 当たってなかったら何もしない
			if (!lineResult.HitFlag)
			{
				continue;
			}

			// 既に当たったポリゴンがあって今まで検出したものより低かったら何もしない
			if (hitFlag && maxYHit > lineResult.Position.y)
			{
				continue;
			}

			// 接触したY座標を保持する
			maxYHit = lineResult.Position.y;

			hitFlag = true;
		}

		// 当たったかどうかで処理
		if (hitFlag)
		{
			area.y = maxYHit;
		}
		else
		{

		}
	}

	// 検出した情報を解放する
	MV1CollResultPolyDimTerminate(hitDim);
}

Character::Character()
{
	// ステージのコリジョン情報の更新
	stageHandle = MV1LoadModel("media\\TESTROOM1\\ROOM1_hantei.fbx");	// 当たり判定用ステージの読み込み
	MV1SetupCollInfo(stageHandle, -1);									// モデルのコリジョン情報をセットアップ(-1による全体フレーム)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// ステージの座標を更新
	MV1SetFrameVisible(stageHandle, -1, false);							// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）
	MV1RefreshCollInfo(stageHandle, -1);								// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）

	// ３Ｄモデルの読み込み
	charamodelhandle = MV1LoadModel("media\\CLPH\\motion\\CLPH_idle.fbx");

	// ３Ｄモデルの0番目のアニメーションをアタッチする
	attachNum = 0;
	attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, FALSE);

	// アタッチしたアニメーションの総再生時間を取得する
	totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

	// モデルの基本情報
	modelHeight = 160.0f;
	modelWigth = 50.0f;

	// モデルの向きと位置
	area = VGet(282.0f, 0.0f, 0.0f);
	angle = 0.0f;
	direXAngle = 0.0f;
	direYAngle = 0.0f;

	// 当たり判定に関する
	wallNum = 0;
	floorNum = 0;
	hitFlag = false;
	moveFlag = false;
	maxYHit = 0.0f;

	// それぞれの速度
	walkSpeed = 10.0f;
	animSpeed = 1.0f;

	// モーション関連
	nowPlayTime = 0.0f;
	motionBlendTime = 0.0f;
	preAttach = -1;
	preMotionPlayTime = 0.0f;

	// モデルの座標を更新
	MV1SetPosition(charamodelhandle, area);
}

Character::~Character()
{
	if (charamodelhandle != -1)
	{
		MV1DeleteModel(charamodelhandle);
	}
	if (stageHandle != -1)
	{
		MV1DeleteModel(stageHandle);
	}
}

void Character::Process(unsigned __int8 controllNumber)
{
	// 指定位置にモデルを配置
	MV1SetPosition(charamodelhandle, area);
}

void Character::Process(unsigned __int8 controllNumber, float getAngle)
{
	angle = getAngle;

	// 左スティックが前に押されたら前進する
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) > 0)
	{
		area.x += MYINPUTPAD::sinf(angle) * -walkSpeed;
		area.z += MYINPUTPAD::cosf(angle) * -walkSpeed;
		direYAngle = 0.0f;
		moveFlag = true;
	}
	// 左スティックが後ろに押されたら後退する
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) < 0)
	{
		area.x += MYINPUTPAD::sinf(angle) * walkSpeed;
		area.z += MYINPUTPAD::cosf(angle) * walkSpeed;
		direYAngle = DX_PI_F;
		moveFlag = true;
	}

	// 左スティックが左に押されたら左に移動する
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) < 0)
	{
		area.x += MYINPUTPAD::cosf(-angle) * walkSpeed;
		area.z += MYINPUTPAD::sinf(-angle) * walkSpeed;
		direXAngle = (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2)) / -BASIC::MAX_STICK_MINUS;
		if (direYAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
	}
	// 左スティックが右に押されたら右に移動する
	else if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) > 0)
	{
		area.x += MYINPUTPAD::cosf(-angle) * -walkSpeed;
		area.z += MYINPUTPAD::sinf(-angle) * -walkSpeed;
		direXAngle = (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2)) / BASIC::MAX_STICK_PLUS;
		if (direYAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
	}
	// キャラの前後の向きを気持ちよくするため
	else
	{
		direXAngle = 0.0f;
		if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) == 0)
		{
			moveFlag = false;
		}
	}

	// モーションの実態
	Player_AnimProcess();

	// ステージのあたり判定
	StageHit();

	// 第二引数の回転角度をセット
	MV1SetRotationXYZ(charamodelhandle, VGet(0.0f, angle + direXAngle + direYAngle, 0.0f));
	// 指定位置にモデルを配置
	MV1SetPosition(charamodelhandle, area);
}


void Character::Draw()
{
	MV1DrawModel(charamodelhandle);

	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);
}

VECTOR Character::GetArea()
{
	return area;
}