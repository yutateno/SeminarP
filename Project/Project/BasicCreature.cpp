#include "BasicCreature.hpp"

// 使うアニメーション管理
void BasicCreature::Player_PlayAnim(int attach)
{
	// 今のモーションが違うものだったら
	if (attachNum != attach)
	{
		// 直前のモーションが有効だったらデタッチする
		if (preAttach != -1 && motionBlendTime < 1.0f)
		{
			MV1DetachAnim(modelHandle, preAttach);
			preAttach = -1;
		}
		// 今のモーションを入れる
		preAttach = attachMotion;
		preMotionPlayTime = nowPlayTime;

		// 引数を今のモーション番号に入れる
		attachNum = attach;

		// 新たに指定のモーションをモデルにアタッチする
		attachMotion = MV1AttachAnim(modelHandle, attachNum, -1, false);

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

// 全てのアニメーションの管理
void BasicCreature::Player_AnimProcess()
{
	// ブレンド率が１以下の場合は１に近づける
	if (motionBlendTime < 1.0)
	{
		motionBlendTime += 0.075f;
		if (motionBlendTime >= 1.0f)
		{
			motionBlendTime = 1.0f;
		}
	}

	// 再生している現在のモーションの処理
	if (attachMotion != -1)
	{
		// モーションの総時間を取得
		totalTime = MV1GetAttachAnimTotalTime(modelHandle, attachMotion);

		// 再生時間を進める
		nowPlayTime += animSpeed;


		// 再生時間が総時間に到達していたらループさせる
		if (nowPlayTime >= totalTime)
		{
			nowPlayTime = fmodf(nowPlayTime, totalTime);
		}

		// 変更した再生時間をモデルに反映させる
		MV1SetAttachAnimTime(modelHandle, attachMotion, nowPlayTime);

		// アニメーション１のモデルに対する反映率をセット
		MV1SetAttachAnimBlendRate(modelHandle, attachMotion, motionBlendTime);
	}

	// 再生している直前のモーションの処理
	if (preAttach != -1)
	{
		// アニメーションの総時間を取得
		totalTime = MV1GetAttachAnimTotalTime(modelHandle, preAttach);

		// 再生時間を進める
		preMotionPlayTime += animSpeed;

		// 再生時間が総時間に到達していたら再生時間をループさせる
		if (preMotionPlayTime > totalTime)
		{
			preMotionPlayTime = fmodf(preMotionPlayTime, totalTime);
		}

		// 変更した再生時間をモデルに反映させる
		MV1SetAttachAnimTime(modelHandle, preAttach, preMotionPlayTime);

		// アニメーション２のモデルに対する反映率をセット
		MV1SetAttachAnimBlendRate(modelHandle, preAttach, (1.0f - motionBlendTime));
	}
}


// ステージのあたり判定処理
void BasicCreature::StageHit()
{
	ActorHit(stageHandle);
}

// ステージのあたり判定処理
void BasicCreature::ActorHit(int stageHandle)
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
		if (hitDim.Dim[i].Normal.y < 0.001f && hitDim.Dim[i].Normal.y > -0.001f)
		{
			// 壁だとしてもキャラクターの足より少し上を調べる
			if (hitDim.Dim[i].Position[0].y > area.y + modelHeight + 50.0f
				|| hitDim.Dim[i].Position[1].y > area.y + modelHeight + 50.0f
				|| hitDim.Dim[i].Position[2].y > area.y + modelHeight + 50.0f)
			{
				// 最大になるまで保存する
				if (wallNum < 64)
				{
					wallPoly[wallNum] = &hitDim.Dim[i];		// ヒットしたポリゴン情報を保存
					wallNum++;
				}
			}
		}
		else
		{
			// 最大になるまで保存
			if (floorNum < 64)
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
			for (int i = 0; i != 64; ++i)
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

					VECTOR slideVec = VCross(VSub(area, preArea), mainPoly->Normal);
					slideVec = VCross(mainPoly->Normal, slideVec);
					area = VAdd(preArea, slideVec);

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

		fallCount++;			// 浮いているとする

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

		// 床に当たったかどうかで処理
		if (hitFlag)
		{
			area.y = maxYHit;

			fallCount = 0;
		}
	}
	else	// 床に触れていない
	{
		area.y -= 0.75f;
	}

	// 検出した情報を解放する
	MV1CollResultPolyDimTerminate(hitDim);
}


// コンストラクタ
BasicCreature::BasicCreature(const int collStageHandle) :BasicObject(collStageHandle)
{
	// ステージのコリジョン情報の更新
	stageHandle = MV1DuplicateModel(collStageHandle);
	MV1SetScale(stageHandle, VGet(1.75f, 1.0f, 1.75f));
	MV1SetupCollInfo(stageHandle, -1);									// モデルのコリジョン情報をセットアップ(-1による全体フレーム)
	MV1SetPosition(stageHandle, VGet(0.0f, 0.0f, 0.0f));				// ステージの座標を更新
	MV1SetFrameVisible(stageHandle, -1, false);							// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）
	MV1RefreshCollInfo(stageHandle, -1);								// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）

	// 当たり判定に関する
	wallNum = 0;
	floorNum = 0;
	hitFlag = false;
	maxYHit = 0.0f;
	moveFlag = false;

	// モーション関連
	nowPlayTime = 0.0f;
	motionBlendTime = 0.0f;
	preAttach = -1;
	preMotionPlayTime = 0.0f;

	fallCount = 0;

	angle = 0.0f;
}

// デストラクタ
BasicCreature::~BasicCreature()
{
	MODEL_RELEASE(stageHandle);
}
