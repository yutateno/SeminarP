#include "EnemyMove1.hpp"

// 動きのプロセス
void EnemyMove1::MoveProcess()
{
	// 上昇させる
	if (upNow)
	{
		// 上昇中
		if (60.0f > area.y)
		{
			area.y += flyMove;
		}
		// 上昇しきったら
		else
		{
			upNow = false;
			flyMove = -0.1f;
		}
	}
	// 下降させる
	else
	{
		// 下降中
		if (area.y > 30.0f)
		{
			area.y -= flyMove;
		}
		// 下降しきったら
		else
		{
			upNow = true;
			flyMove = -0.1f;
		}
	}
	// 始発をゆっくりさせる
	if (flyMove < 0.5f)
	{
		flyMove += 0.05f;
	}
}


// コンストラクタ
EnemyMove1::EnemyMove1(int collStageHandle, float areaX, float areaZ, float color) : BasicActor(collStageHandle)
{
	// モデルの向きと位置
	this->area = VGet(areaX, 40.0f, areaZ);

	// マテリアル
	material.Diffuse = GetColorF(0.0f, 0.0f, 1.0f, 0.0f);
	material.Specular = GetColorF(0.0f, 0.0f, 0.0f, 0.0f);
	material.Ambient = GetColorF(1.0f, 1.0f, 1.0f, 0.0f);
	material.Emissive = GetColorF(color, color, 1.0f, 0.0f);
	material.Power = 10.0f;

	// モデルの基本情報
	modelHeight = 10.0f;
	modelWigth = 15.0f;

	// 足元の影に関する
	shadowHeight = 5.0f;
	shadowSize = 15.0f;

	upNow = true;
	flyMove = 0.0f;

	viewNow = true;
}

// デストラクタ
EnemyMove1::~EnemyMove1()
{

}


// 描画
void EnemyMove1::Draw()
{
	if (viewNow)
	{
		SetMaterialParam(material);
		// Ｚバッファを有効にする
		SetUseZBuffer3D(TRUE);
		// Ｚバッファへの書き込みを有効にする
		SetWriteZBuffer3D(TRUE);
		DrawSphere3D(VAdd(area, VGet(0.0f, 60.0f, 0.0f)), modelWigth, 16, GetColor(68, 178, 227), GetColor(255, 255, 255), TRUE);
		// Ｚバッファへの書き込みを有効にする
		SetWriteZBuffer3D(FALSE);
		// Ｚバッファを有効にする
		SetUseZBuffer3D(FALSE);

#ifdef _MODEL_DEBUG
		VECTOR viewArea = VAdd(area, VGet(0.0f, 60.0f, 0.0f));		// モデルの初期Y座標が浮いているので調整

		DrawSphere3D(VAdd(area, VGet(0.0f, 60.0f, 0.0f)), modelWigth + 3, 8, GetColor(0, 255, 0), GetColor(255, 255, 255), false);
#endif // _MODEL_DEBUG
	}
}

// メインプロセス
void EnemyMove1::Process()
{
	if (viewNow)
	{
		// 動きのプロセス
		MoveProcess();
	}
}

void EnemyMove1::SetViewNow(bool viewNow)
{
	this->viewNow = viewNow;
}
