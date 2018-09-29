#include "Character.hpp"

using namespace MY_XINPUT;


// 動きのプロセス
void Character::MoveProcess(unsigned __int8 controllNumber)
{
	// スムーズに動かせる
	if (moveFlag)
	{
		animSpeed = 1.0f;
		if (direXAngle == 0.0f)
		{
			if (walkSpeed < 20.0f)
			{
				walkSpeed += 2.5f;
			}
			else
			{
				walkSpeed = 20.0f;
			}
		}
		else	// 斜め方向
		{
			if (walkSpeed < 12.0f)
			{
				walkSpeed += 3.0f;
			}
			else
			{
				walkSpeed = 12.0f;
			}
		}
	}
	else
	{
		animSpeed = 0.5f;
		if (walkSpeed > 0.0f)
		{
			walkSpeed -= 5.0f;
		}
		else
		{
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
		Player_PlayAnim(MOTION::walk);
	}
	// 左スティックが後ろに押されたら後退する
	if (0 > InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y))
	{
		area.x += sinf(angle + direXAngle) * walkSpeed;
		area.z += cosf(angle + direXAngle) * walkSpeed;
		direXAngle = 0.0f;
		direZAngle = DX_PI_F;
		moveFlag = true;
		Player_PlayAnim(MOTION::walk);
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
		Player_PlayAnim(MOTION::walk);
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
		Player_PlayAnim(MOTION::walk);
	}
	// キャラの前後の向きを気持ちよくするため
	else
	{
		if (InputPad::GetPadThumbData(controllNumber, STICK_LEFT_Y) == 0)
		{
			moveFlag = false;
			Player_PlayAnim(MOTION::idle);
		}
	}
}


// コンストラクタ
Character::Character(const int modelHandle, const int collStageHandle) : BasicCreature(collStageHandle)
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


	// 足元の影に関する
	shadowHeight = 35.0f;
	shadowSize = 50.0f;


	// それぞれの速度
	walkSpeed = 0.0f;
	animSpeed = 0.5f;


	// モデルの座標を更新
	MV1SetPosition(this->modelHandle, area);
}

// デストラクタ
Character::~Character()
{
	MODEL_RELEASE(modelHandle);
}


// メインプロセス
void Character::Process(const unsigned __int8 controllNumber, const float getAngle)
{
	preArea = area;		// 直前の座標
	if (moveFlag)
	{
		angle = getAngle;	// カメラ向きのアングル
	}

	// 動きのプロセス
	MoveProcess(controllNumber);

	// モーションの実態
	Player_AnimProcess();

	// ステージのあたり判定
	StageHit();

	// 第二引数の回転角度をセット
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, angle + direXAngle + direZAngle, 0.0f));
	// 指定位置にモデルを配置
	MV1SetPosition(modelHandle, area);
}


void Character::PositionReset()
{
	area = VGet(0.0f, 0.0f, 0.0f);
}


// 描画
void Character::Draw()
{
	BasicObject::Draw();		// 基本的なものを引っ張ってくる

	BasicObject::ShadowFoot();

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _MODEL_DEBUG
#ifdef _CHARACTER_DEBUG
	printfDx("XAngle:%f\tZAngle%f\t左:%d\t上:%d\n", direXAngle, direZAngle, InputPad::GetPadThumbData(0, STICK_LEFT_X), InputPad::GetPadThumbData(0, STICK_LEFT_Y));
#endif // _CHARACTER_DEBUG
}