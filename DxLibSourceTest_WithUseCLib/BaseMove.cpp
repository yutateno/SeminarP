#include "BaseMove.hpp"

void BaseMove::ShadowCharaSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapCharaHandle);
}

void BaseMove::ShadowCharaSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}

void BaseMove::ShadowAnotherCharaSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapAnotherCharaHandle);
}

void BaseMove::ShadowAnotherCharaSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}

void BaseMove::ShadowNoMoveSetUpBefore()
{
	// シャドウマップへの描画の準備
	ShadowMap_DrawSetup(shadowMapNoMoveHandle);
}

void BaseMove::ShadowNoMoveSetUpAfter()
{
	// シャドウマップへの描画を終了
	ShadowMap_DrawEnd();
}


void BaseMove::ShadowCharaDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(0, shadowMapCharaHandle);
}

void BaseMove::ShadowCharaDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(0, -1);
}

void BaseMove::ShadowAnotherCharaDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(1, shadowMapAnotherCharaHandle);
}

void BaseMove::ShadowAnotherCharaDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(1, -1);
}

void BaseMove::ShadowNoMoveDrawBefore()
{
	// 描画に使用するシャドウマップを設定
	SetUseShadowMap(2, shadowMapNoMoveHandle);
}

void BaseMove::ShadowNoMoveDrawAfter()
{
	// 描画に使用するシャドウマップの設定を解除
	SetUseShadowMap(2, -1);
}

void BaseMove::ShadowArea(VECTOR charaArea)
{
	SetShadowMapDrawArea(shadowMapCharaHandle		, VAdd(charaArea, shadowCharaLowArea)		, VAdd(charaArea, shadowCharaHighArea));
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, VAdd(charaArea, shadowAnotherCharaLowArea), VAdd(charaArea, shadowAnotherCharaHighArea));
}



int BaseMove::GetDistance(VECTOR alpha, VECTOR beta)
{
	double distance = sqrt((alpha.x - beta.x) * (alpha.x - beta.x) + (alpha.z - beta.z) * (alpha.z - beta.z));
	return (int)distance;
}

BaseMove::BaseMove()
{
	// シャドウマップハンドルの作成
	shadowMapCharaHandle		 = MakeShadowMap(4096, 4096);
	shadowMapAnotherCharaHandle	 = MakeShadowMap(512, 512);
	shadowMapNoMoveHandle		 = MakeShadowMap(4096, 4096);


	// シャドウマップに描画する範囲を設定
	shadowCharaLowArea			 = VGet(-500.0f, -1.0f, -500.0f);
	shadowCharaHighArea			 = VGet(500.0f, 10.0f, 500.0f);

	shadowAnotherCharaLowArea	 = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowAnotherCharaHighArea	 = VGet(2000.0f, 100.0f, 2000.0f);

	shadowNoMoveLowArea			 = VGet(-2000.0f, -1.0f, -2000.0f);
	shadowNoMoveHighArea		 = VGet(2000.0f, 1000.0f, 2000.0f);

	SetShadowMapDrawArea(shadowMapCharaHandle		, shadowCharaLowArea		, shadowCharaHighArea		);
	SetShadowMapDrawArea(shadowMapAnotherCharaHandle, shadowAnotherCharaLowArea	, shadowAnotherCharaHighArea);
	SetShadowMapDrawArea(shadowMapNoMoveHandle		, shadowNoMoveLowArea		, shadowNoMoveHighArea		);


	lightDire = VGet(-0.5f, -0.7f, 0.5f);
	// ライトの方向を設定
	SetLightDirection(lightDire);
	// シャドウマップが想定するライトの方向もセット
	SetShadowMapLightDirection(shadowMapCharaHandle			, lightDire);
	SetShadowMapLightDirection(shadowMapAnotherCharaHandle	, lightDire);
	SetShadowMapLightDirection(shadowMapNoMoveHandle		, lightDire);
}

BaseMove::~BaseMove()
{
	// シャドウマップの削除
	DeleteShadowMap(shadowMapNoMoveHandle);
	DeleteShadowMap(shadowMapAnotherCharaHandle);
	DeleteShadowMap(shadowMapCharaHandle);
}
