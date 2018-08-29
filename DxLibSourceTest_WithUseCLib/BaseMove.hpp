#pragma once
#include "DxLib.h"

class BaseMove
{
private:
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

protected:
	// 描画の準備
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

public:
	BaseMove();
	virtual ~BaseMove();
};

