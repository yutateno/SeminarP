#pragma once
#include "LoadFile.hpp"


class BasicObject
{
protected:
	float shadowHeight;			// 足元影の高さ
	float shadowSize;			// 足元影のサイズ
	void ShadowFoot();			// 足元影のプロセス
	VECTOR area;				// 足元影のエリア


	int modelHandle;	// モデルのハンドル


	float modelHeight;	// モデルの高さ


private:
	MV1_COLL_RESULT_POLY_DIM ShadowHitResDim;		// 周囲のポリゴンを代入する構造体
	MV1_COLL_RESULT_POLY *ShadowHitRes;				// 影のポリゴンの構造体
	VERTEX3D ShadowVertex[3];
	VECTOR ShadowSlideVec;
	int shadowHandle;


	int stageHandle;


public:
	BasicObject(const int collStageHandle);
	virtual ~BasicObject();


	const VECTOR GetArea() const;

	void Draw();		// 描画
};

