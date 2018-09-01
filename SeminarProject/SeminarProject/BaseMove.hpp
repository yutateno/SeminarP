#pragma once
#include "DxLib.h"

#include <math.h>

enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE
};

class BaseMove
{
private:
	// 影に関して-----------------------------------------------
	int shadowMapCharaHandle;
	int shadowMapAnotherCharaHandle;
	int shadowMapNoMoveHandle;

	VECTOR shadowCharaLowArea;
	VECTOR shadowCharaHighArea;

	VECTOR shadowAnotherCharaLowArea;
	VECTOR shadowAnotherCharaHighArea;

	VECTOR shadowNoMoveLowArea;
	VECTOR shadowNoMoveHighArea;

	VECTOR lightDire;
	// ---------------------------------------------------------

protected:
	static bool endFlag;		// 終了フラッグ

	static ESceneNumber scene;	// 現在のシーン

	// 影に関して-----------------------------------------------
	// シャドウマップ０番：主人公
	void ShadowCharaSetUpBefore();
	void ShadowCharaSetUpAfter();

	// シャドウマップ１番：主人公以外
	void ShadowAnotherCharaSetUpBefore();
	void ShadowAnotherCharaSetUpAfter();

	// シャドウマップ２番：動かないもの
	void ShadowNoMoveSetUpBefore();
	void ShadowNoMoveSetUpAfter();

	// 描画へ使用する
	// シャドウマップ０番：主人公
	void ShadowCharaDrawBefore();
	void ShadowCharaDrawAfter();

	// シャドウマップ１番：主人公以外
	void ShadowAnotherCharaDrawBefore();
	void ShadowAnotherCharaDrawAfter();

	// シャドウマップ２番：動かないもの
	void ShadowNoMoveDrawBefore();
	void ShadowNoMoveDrawAfter();

	// 座標を更新し続ける
	void ShadowArea(VECTOR charaArea);
	// ---------------------------------------------------------

	// 二つのモデルの距離
	int GetDistance(VECTOR alpha, VECTOR beta);

public:
	BaseMove();
	virtual ~BaseMove();

	virtual void Draw() = 0;
	virtual void Process(unsigned __int8 controllNumber) = 0;

	static bool GetEndFlag();		// 終了ゲッター
	static ESceneNumber GetScene();	// 今のシーンゲッター

	void SetScene(ESceneNumber scene);	// 今のシーンセッター
};

