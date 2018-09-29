#include "CharacterSword.hpp"

using namespace MY_XINPUT;


// 動きのプロセス
void CharacterSword::MoveProcess(unsigned __int8 controllNumber)
{
	// スムーズに動かせる
	if (moveFlag)
	{
		animSpeed = 0.75f;
		if (direXAngle == 0.0f || direXAngle == DX_PI_F / 2.0f || direXAngle == -DX_PI_F / 2.0f)
		{
			if (walkSpeed < 30.0f)
			{
				walkNow = true;
				walkSpeed += 5.0f;
			}
			else
			{
				walkNow = false;
				walkSpeed = 30.0f;
			}
		}
		else	// 斜め方向
		{
			if (walkSpeed < 20.0f)
			{
				walkNow = true;
				walkSpeed += 5.0f;
			}
			else
			{
				walkNow = false;
				walkSpeed = 20.0f;
			}
		}
	}
	else
	{
		animSpeed = 0.5f;
		if (walkSpeed > 0.0f)
		{
			walkNow = true;
			walkSpeed -= 5.0f;
		}
		else
		{
			walkNow = false;
			walkSpeed = 0.0f;
		}
	}

	// 左スティックが前に押されたら前進する
	if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) > 0)
	{
		area.x += sinf(angle + direXAngle) * -walkSpeed;
		area.z += cosf(angle + direXAngle) * -walkSpeed;
		direXAngle = 0.0f;
		direZAngle = 0.0f;
		moveFlag = true;
	}
	// 左スティックが後ろに押されたら後退する
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direZAngle = DX_PI_F;
		moveFlag = true;
	}

	// 左スティックが左に押されたら左に移動する
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X))
	{
		area.x += cosf(-angle) * walkSpeed;
		area.z += sinf(-angle) * walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, false);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
	}
	// 左スティックが右に押されたら右に移動する
	else if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) > 0)
	{
		area.x += cosf(-angle) * -walkSpeed;
		area.z += sinf(-angle) * -walkSpeed;
		direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, true);
		if (direZAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
		moveFlag = true;
	}
	// キャラの前後の向きを気持ちよくするため
	else
	{
		if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) == 0)
		{
			moveFlag = false;
		}
	}
}


// 攻撃に関するプロセス
void CharacterSword::AttackProcess(unsigned __int8 controllNumber)
{
	// 攻撃のコマンドを押したら
	if (InputPad::GetPadButtonData(controllNumber, BUTTON_X) == 1)
	{
		// 最初の時
		if (attackFrame == 0)
		{
			animSpeed = 0.4f;									// アニメーション速度を変更
			
			
			// 移動プロセスから流用して前方に移動させる
			area.x += sinf(angle + direXAngle) * -walkSpeed;
			area.z += cosf(angle + direXAngle) * -walkSpeed;


			attackNow = true;					// 攻撃しているフラッグを立てる
			attackNumber = MOTION::action1;		// 攻撃コマンド番号を1番にする
		}
		// 二回目以降の攻撃時
		else if (attackFrame < 10.0f)
		{
			attackNext = true;			// 次の攻撃モーションに移行するというフラッグを立てる
		}
	}


	// 攻撃モーションの終盤当たりで次の行動を決める
	if (attackFrame >= 9.0f)
	{
		// 次の攻撃へ移行するとフラッグが立っていたら
		if (attackNext)
		{
			// 前方に移動する
			area.x += sinf(angle + direXAngle) * -walkSpeed * (1 + (-2 * (direZAngle / DX_PI_F)));
			area.z += cosf(angle + direXAngle) * -walkSpeed * (1 + (-2 * (direZAngle / DX_PI_F)));


			// 直前の攻撃モーションで次のモーションを決める
			switch (preAttackNumber)
			{
			// 最初の攻撃時
			case MOTION::action1:
				animSpeed = 0.4f;									// アニメーション速度を変更
				attackNumber = MOTION::action2;
				preAttackNumber = attackNumber;
				break;
			// 二コンボ目の攻撃時
			case MOTION::action2:
				animSpeed = 0.4f;									// アニメーション速度を変更
				attackNumber = MOTION::action3;
				preAttackNumber = attackNumber;
				break;
			// 最後の攻撃時
			case MOTION::action3:
				attackNow = false;					// 次のコンボがないので攻撃フラッグを倒す
				attackNumber = MOTION::action1;
				preAttackNumber = attackNumber;
				walkSpeed = 0.0f;
				break;
			}


			attackFrame = 0;		// 攻撃のフレームを消す
			attackNext = false;		// 次の攻撃するかどうかを倒す
		}
		// 次の攻撃をしない
		else
		{
			walkSpeed = 0.0f;
			attackNow = false;					// 攻撃フラッグを倒す
			attackFrame = 0;
			attackNumber = MOTION::action1;
			preAttackNumber = attackNumber;
		}
	}



	// 攻撃フラッグが立ったら
	if (attackNow)
	{
		if (walkSpeed < 60.0f)
		{
			walkSpeed += 20.0f;
		}
		else
		{
			walkSpeed = 60.0f;
		}


		attackFrame += animSpeed;


		// 左スティックが前に押されたら前を向く
		if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) > 0)
		{
			direXAngle = 0.0f;
			direZAngle = 0.0f;
		}
		// 左スティックが後ろに押されたら後ろを向く
		if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
		{
			direXAngle = 0.0f;
			direZAngle = DX_PI_F;
		}

		// 左スティックが左に押されたら左を向く
		if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X))
		{
			direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, false);
			if (direZAngle != 0.0f)
			{
				direXAngle = -direXAngle;
			}
		}
		// 左スティックが右に押されたら右を向く
		else if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) > 0)
		{
			direXAngle = ((float)InputPad::GetPadThumbData(controllNumber, STICK_LEFT_X) * (DX_PI_F / 2.0f)) / (float)InputPad::GetPadThumbMax(false, true, true);
			if (direZAngle != 0.0f)
			{
				direXAngle = -direXAngle;
			}
		}
	}
}


// ジャンプに関するプロセス
void CharacterSword::JumpProcess(unsigned __int8 controllNumber)
{
	// 浮いてない状態でジャンプするコマンドを押したら
	if (InputPad::GetPadButtonData(controllNumber, BUTTON_A) == 1
		&& !jumpNow)
	{
		jumpNow = true;					// 飛んでいる
		jumpUpNow = true;				// 上に上がっている
		jumpPower = flyJumpPower;		// 飛ぶ速度を加える
	}


	// 足元に何もなかったら
	if (fallCount > 1)
	{
		// 飛ぶコマンドで飛んでいなかったら
		if (!jumpNow)
		{
			jumpNow = true;				// 飛んでいる

			jumpPower = fallJumpPower;	// 落下速度を加える
		}
	}


	// 飛んでいる
	if (jumpNow)
	{
		walkSpeed = 10.0f;
		animSpeed = 1.0f;
		jumpPower -= gravity;			// 落下重力を加え続ける
		area.y += jumpPower;			// Y座標に加え続ける
		
		
		// ジャンプにて最頂点に到達したら
		if (jumpPower <= 0.0f)
		{
			jumpUpNow = false;			// 落下に切り替える

			// 地面に触れたら
			if (fallCount <= 1)
			{
				jumpNow = false;
				jumpPower = 0;
				jumpUpNow = false;
			}
		}
		else
		{
			// 地面に触れてなかったら
			if (fallCount > 1)
			{
				jumpUpNow = false;		// 階段から落ちてる
			}
			else
			{
				jumpUpNow = true;		// 通常ジャンプにてジャンプした
			}
		}
	}
}


// アニメーションのプロセス
void CharacterSword::AnimProcess()
{
	// 飛んでいる
	if (jumpNow)
	{
		// 上昇している
		if (jumpUpNow)
		{
			Player_PlayAnim(MOTION::jump);
		}
		else
		{
			Player_PlayAnim(MOTION::fall);
		}
	}
	else
	{
		// 攻撃している
		if (attackNow)
		{
			Player_PlayAnim(attackNumber);
		}
		else
		{
			// 動いている
			if (moveFlag)
			{
				// 歩く速度の時
				if (walkNow)
				{
					Player_PlayAnim(MOTION::walk);
				}
				else
				{
					Player_PlayAnim(MOTION::dash);
				}
			}
			else
			{
				Player_PlayAnim(MOTION::idle);
			}
		}
	}
}


CharacterSword::CharacterSword(const int modelHandle, const int collStageHandle, const int stairsHandle) : BasicCreature(collStageHandle)
{
	// ３Ｄモデルの読み込み
	this->modelHandle = 0;
	this->modelHandle = MV1DuplicateModel(modelHandle);


	// ３Ｄモデルの0番目のアニメーションをアタッチする
	attachNum = MOTION::idle;
	attachMotion = MV1AttachAnim(this->modelHandle, attachNum, -1, FALSE);


	// アタッチしたアニメーションの総再生時間を取得する
	totalTime = MV1GetAttachAnimTotalTime(this->modelHandle, attachMotion);


	// モデルの基本情報
	modelHeight = 160.0f;
	modelWigth = 50.0f;


	// モデルの向きと位置
	area = VGet(0.0f, 0.0f, 0.0f);
	preArea = area;
	direXAngle = 0.0f;
	direZAngle = 0.0f;
	walkNow = false;


	// 足元の影に関する
	shadowHeight = 20.0f;
	shadowSize = 50.0f;


	// それぞれの速度
	walkSpeed = 0.0f;
	animSpeed = 0.25f;


	// 攻撃に関して
	attackNow = false;
	attackNext = false;
	attackFrame = 0;
	attackNumber = MOTION::action1;
	preAttackNumber = MOTION::action1;


	// ジャンプに関して
	jumpNow = false;
	jumpUpNow = false;
	jumpPower = 0.0f;
	gravity = 0.75f;
	flyJumpPower = 30.0f;
	fallJumpPower = 3.0f;


	// ステージのコリジョン情報の更新
	this->stairsHandle[0] = MV1DuplicateModel(stairsHandle);


	// モデルの座標を更新
	MV1SetPosition(this->modelHandle, area);
}


CharacterSword::~CharacterSword()
{
	for (int i = 0; i != 10; ++i)
	{
		MODEL_RELEASE(stairsHandle[i]);
	}
	MODEL_RELEASE(modelHandle);
}

void CharacterSword::SetStairsArea(const VECTOR stairsArea, const int num)
{
	// ステージのコリジョン情報の更新
	if (num != 0)
	{
		stairsHandle[num] = MV1DuplicateModel(stairsHandle[0]);
	}
	MV1SetupCollInfo(stairsHandle[num], -1);									// モデルのコリジョン情報をセットアップ(-1による全体フレーム)
	MV1SetPosition(stairsHandle[num], stairsArea);				// ステージの座標を更新
	MV1SetFrameVisible(stairsHandle[num], -1, false);							// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）
	MV1RefreshCollInfo(stairsHandle[num], -1);								// ステージを描画させない（でもどうせDraw呼ばないからこれ意味ない気もする）
}


// メインプロセス
void CharacterSword::Process(const unsigned __int8 controllNumber, const float getAngle)
{
	preArea = area;		// 直前の座標
	if (moveFlag || attackNow)
	{
		angle = getAngle;	// カメラ向きのアングル
	}

	// 動きのプロセス
	if (!attackNow)
	{
		MoveProcess(controllNumber);
	}

	// 攻撃のプロセス
	AttackProcess(controllNumber);

	
	// モーションの実態
	Player_AnimProcess();


	// モーションのプロセス
	AnimProcess();


	// 階段のあたり判定
	for (int i = 0; i != 10; ++i)
	{
		ActorHit(stairsHandle[i]);
	}

	// ジャンプのプロセス
	JumpProcess(controllNumber);

	// ステージのあたり判定
	StageHit();



	// 第二引数の回転角度をセット
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, angle + direXAngle + direZAngle, 0.0f));
	// 指定位置にモデルを配置
	MV1SetPosition(modelHandle, area);
}


void CharacterSword::PositionReset()
{
	area = VGet(0.0f, 0.0f, 0.0f);
}


// 描画
void CharacterSword::Draw()
{
	BasicObject::Draw();		// 基本的なものを引っ張ってくる

	BasicObject::ShadowFoot();

	printfDx("%f\t%d\n", walkSpeed, fallCount);

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _MODEL_DEBUG
#ifdef _CHARACTER_DEBUG
	printfDx("XAngle:%f\tZAngle%f\t左:%d\t上:%d\n", direXAngle, direZAngle, InputPad::GetPadThumbData(0, STICK_LEFT_X), InputPad::GetPadThumbData(0, STICK_LEFT_Y));
#endif // _CHARACTER_DEBUG
}