#include "OrdinaryPerson.hpp"


// 動きのプロセス
void OrdinaryPerson::MoveProcess()
{
	std::random_device rnd;     // 非決定的な乱数生成器を生成
	std::mt19937 mt(rnd());     // メルセンヌ・ツイスタの32ビット版
	std::uniform_int_distribution<> randInX(0, 200);			// X座標用乱数
	std::uniform_int_distribution<> moveTurn(0, 314);				// Z座標用乱数

	moveCount++;

	// スムーズに動かせる
	if (moveFlag)
	{
		animSpeed = 0.75f;
		if (direXAngle == 0.0f)
		{
			if (walkSpeed < 6.0f)
			{
				walkSpeed += 2.5f;
			}
			else
			{
				walkSpeed = 6.0f;
			}
		}
		else	// 斜め方向
		{
			if (walkSpeed < 3.0f)
			{
				walkSpeed += 1.0f;
			}
			else
			{
				walkSpeed = 3.0f;
			}
		}
	}
	else
	{
		animSpeed = 0.5f;
		if (walkSpeed > 0.0f)
		{
			walkSpeed -= 3.0f;
		}
		else
		{
			walkSpeed = 0.0f;
		}
	}


	if (moveCount >= 400)
	{
		moveCount = 0;
	}
	else if (moveCount == 100)
	{
		nextDireZAngle = moveTurn(mt) / 100.0f;
		nextDireXAngle = randInX(mt) / 100.0f;
		if (nextDireZAngle != 0.0f)
		{
			nextDireXAngle = -nextDireXAngle;
		}
	}

	area.x += sinf(angle + direXAngle + direZAngle) * -walkSpeed;
	area.z += cosf(angle + direXAngle + direZAngle) * -walkSpeed;
	moveFlag = true;
	Player_PlayAnim(MOTION::walk);

	if (nextDireXAngle != direXAngle)
	{
		if (direXAngle > nextDireXAngle)
		{
			direXAngle -= 0.01f;
		}
		else
		{
			direXAngle += 0.01f;
		}
	}
	if (nextDireZAngle != direZAngle)
	{
		if (direZAngle > nextDireZAngle)
		{
			direZAngle -= 0.01f;
		}
		else
		{
			direZAngle += 0.01f;
		}
	}
}


OrdinaryPerson::OrdinaryPerson(const int modelHandle, const int collStageHandle, const VECTOR area) : BasicCreature(collStageHandle)
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
	this->area = VGet(1000.0f, 0.0f, 1000.0f);
	preArea = this->area;
	direXAngle = 0.0f;
	direZAngle = 0.0f;
	nextDireZAngle = 0.0f;
	nextDireXAngle = 0.0f;


	// 足元の影に関する
	shadowHeight = 35.0f;
	shadowSize = 50.0f;


	// それぞれの速度
	walkSpeed = 0.0f;
	animSpeed = 0.5f;


	moveCount = 0;


	// モデルの座標を更新
	MV1SetPosition(this->modelHandle, this->area);
}


OrdinaryPerson::~OrdinaryPerson()
{
	MODEL_RELEASE(modelHandle);
}


// メインプロセス
void OrdinaryPerson::Process()
{
	preArea = area;		// 直前の座標

	// 動きのプロセス
	MoveProcess();

	// モーションの実態
	Player_AnimProcess();

	// ステージのあたり判定
	StageHit();

	// 第二引数の回転角度をセット
	MV1SetRotationXYZ(modelHandle, VGet(0.0f, direXAngle + direZAngle, 0.0f));
	// 指定位置にモデルを配置
	MV1SetPosition(modelHandle, area);
}


// 描画
void OrdinaryPerson::Draw()
{
	BasicObject::Draw();		// 基本的なものを引っ張ってくる

	BasicObject::ShadowFoot();

#ifdef _MODEL_DEBUG
	DrawCapsule3D(area, VAdd(area, VGet(0.0f, modelHeight, 0.0f)), modelWigth, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);		// 当たり判定を確認用の表示テスト
#endif // _MODEL_DEBUG
#ifdef _DEBUG
	//printfDx("X : %f\tZ : %f\tNX : %f\tNZ : %f\n", direXAngle, direZAngle, nextDireXAngle, nextDireZAngle);
#endif // _DEBUG
}