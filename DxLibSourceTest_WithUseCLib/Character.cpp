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

Character::Character()
{
	// ３Ｄモデルの読み込み
	this->charamodelhandle = MV1LoadModel("media\\CLPH\\motion\\CLPH_idle.fbx");

	// ３Ｄモデルの0番目のアニメーションをアタッチする
	attachNum = 0;
	attachMotion = MV1AttachAnim(charamodelhandle, attachNum, -1, FALSE);

	// アタッチしたアニメーションの総再生時間を取得する
	totalTime = MV1GetAttachAnimTotalTime(charamodelhandle, attachMotion);

	modelHeight = 180;
	modelWigth = 40.0f;

	area = VGet(0, 0, 0);
	angle = 0.0f;
	direXAngle = 0.0f;
	direYAngle = 0.0f;

	// それぞれの速度
	walkSpeed = 3.0f;
	animSpeed = 1.0f;

	// モーション関連
	nowPlayTime = 0.0;
	motionBlendTime = 0.0;
	preAttach = -1;
	preMotionPlayTime = 0.0;
}

Character::~Character()
{
	if (charamodelhandle != -1)
	{
		MV1DeleteModel(charamodelhandle);
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
	}
	// 左スティックが後ろに押されたら後退する
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_Y) < 0)
	{
		area.x += MYINPUTPAD::sinf(angle) * walkSpeed;
		area.z += MYINPUTPAD::cosf(angle) * walkSpeed;
		direYAngle = DX_PI_F;
	}

	// 左スティックが左に押されたら左に移動する
	if (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) < 0)
	{
		area.x += MYINPUTPAD::cosf(-angle) * walkSpeed;
		area.z += MYINPUTPAD::sinf(-angle) * walkSpeed;
		direXAngle = (MYINPUTPAD::InputPad::GetPadThumbData(controllNumber, MYINPUTPAD::XINPUT_PAD::STICK_LEFT_AXIS_X) * (DX_PI_F / 2)) / -BASIC::MAX_STICK_MINUS;		// デッドゾーン範囲で困ったことになってる
		if (direYAngle != 0.0f)
		{
			direXAngle = -direXAngle;
		}
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
	}
	// キャラの前後の向きを気持ちよくするため
	else
	{
		direXAngle = 0.0f;
	}

	// モーションの実態
	Player_AnimProcess();

	// 第二引数の回転角度をセット
	MV1SetRotationXYZ(charamodelhandle, VGet(0.0f, angle + direXAngle + direYAngle, 0.0f));
	// 指定位置にモデルを配置
	MV1SetPosition(charamodelhandle, area);
}


void Character::Draw()
{
	MV1DrawModel(charamodelhandle);
}

VECTOR Character::GetArea()
{
	return area;
}