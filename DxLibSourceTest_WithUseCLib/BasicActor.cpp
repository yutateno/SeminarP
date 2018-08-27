#include "BasicActor.hpp"

void BasicActor::ShadowFoot()
{
	// ライティングを無効にする
	SetUseLighting(FALSE);

	// Ｚバッファを有効にする
	SetUseZBuffer3D(TRUE);

	// テクスチャアドレスモードを CLAMP にする( テクスチャの端より先は端のドットが延々続く )
	SetTextureAddressMode(DX_TEXADDRESS_CLAMP);

	// プレイヤーの直下に存在する地面のポリゴンを取得
	ShadowHitResDim = MV1CollCheck_Capsule(stageHandle, -1, area, VAdd(area, VGet(0.0f, shadowHeight, 0.0f)), shadowSize);

	// 頂点データで変化が無い部分をセット
	ShadowVertex[0].dif = GetColorU8(255, 255, 255, 255);
	ShadowVertex[0].spc = GetColorU8(0, 0, 0, 0);
	ShadowVertex[0].su = 0.0f;
	ShadowVertex[0].sv = 0.0f;
	ShadowVertex[1] = ShadowVertex[0];
	ShadowVertex[2] = ShadowVertex[0];

	// 球の直下に存在するポリゴンの数だけ繰り返し
	ShadowHitRes = ShadowHitResDim.Dim;
	for (int i = 0; i != ShadowHitResDim.HitNum; ++i, ++ShadowHitRes)
	{
		// ポリゴンの座標は地面ポリゴンの座標
		ShadowVertex[0].pos = ShadowHitRes->Position[0];
		ShadowVertex[1].pos = ShadowHitRes->Position[1];
		ShadowVertex[2].pos = ShadowHitRes->Position[2];

		// ちょっと持ち上げて重ならないようにする
		ShadowSlideVec = VScale(ShadowHitRes->Normal, 0.5f);
		ShadowVertex[0].pos = VAdd(ShadowVertex[0].pos, ShadowSlideVec);
		ShadowVertex[1].pos = VAdd(ShadowVertex[1].pos, ShadowSlideVec);
		ShadowVertex[2].pos = VAdd(ShadowVertex[2].pos, ShadowSlideVec);

		// ポリゴンの不透明度を設定する
		ShadowVertex[0].dif.a = 0;
		ShadowVertex[1].dif.a = 0;
		ShadowVertex[2].dif.a = 0;
		if (ShadowHitRes->Position[0].y > area.y - shadowHeight)
		{
			ShadowVertex[0].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[0].y - area.y) / shadowHeight));
		}

		if (ShadowHitRes->Position[1].y > area.y - shadowHeight)
		{
			ShadowVertex[1].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[1].y - area.y) / shadowHeight));
		}

		if (ShadowHitRes->Position[2].y > area.y - shadowHeight)
		{
			ShadowVertex[2].dif.a = (BYTE)(128 * (1.0f - fabs(ShadowHitRes->Position[2].y - area.y) / shadowHeight));
		}

		// ＵＶ値は地面ポリゴンとプレイヤーの相対座標から割り出す
		ShadowVertex[0].u = (ShadowHitRes->Position[0].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[0].v = (ShadowHitRes->Position[0].z - area.z) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[1].u = (ShadowHitRes->Position[1].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[1].v = (ShadowHitRes->Position[1].z - area.z) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[2].u = (ShadowHitRes->Position[2].x - area.x) / (shadowSize * 2.0f) + 0.5f;
		ShadowVertex[2].v = (ShadowHitRes->Position[2].z - area.z) / (shadowSize * 2.0f) + 0.5f;

		// 影ポリゴンを描画
		DrawPolygon3D(ShadowVertex, 1, shadowHandle, TRUE);
	}

	// 検出した地面ポリゴン情報の後始末
	MV1CollResultPolyDimTerminate(ShadowHitResDim);

	// ライティングを有効にする
	SetUseLighting(TRUE);

	// Ｚバッファを無効にする
	SetUseZBuffer3D(FALSE);
}

void BasicActor::Player_PlayAnim(int attach)
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

void BasicActor::Player_AnimProcess()
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

void BasicActor::StageHit()
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
	}

	// 検出した情報を解放する
	MV1CollResultPolyDimTerminate(hitDim);
}

BasicActor::BasicActor(int collStageHandle)
{
	// 影の読み込み
	LoadFile::MyLoad("media\\Shadow.tyn", shadowHandle, ELOADFILE::graph);

	// ステージのコリジョン情報の更新
	stageHandle = collStageHandle;
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

	angle = 0.0f;
}


BasicActor::~BasicActor()
{
	if (shadowHandle != -1)
	{
		DeleteGraph(shadowHandle);
	}

	if (stageHandle != -1)
	{
		MV1DeleteModel(stageHandle);
	}
}

void BasicActor::Draw()
{
	MV1DrawModel(modelHandle);

	ShadowFoot();
}