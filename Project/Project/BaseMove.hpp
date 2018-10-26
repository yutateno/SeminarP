#pragma once
#include "Basic.hpp"


// シーンの動き
enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE, SECONDLOAD, SECONDMOVE, THORDLOAD, THORDMOVE, FORTHLOAD, FORTHMOVE, FIFTHLOAD, FIFTHMOVE
	, SIXTHLOAD, SIXTHMOVE, TITLELOAD, TITLEMOVE
};


class BaseMove
{
private:
	// 影に関して-----------------------------------------------
	int shadowMapCharaHandle;				// キャラクターの影のハンドル
	int shadowMapAnotherCharaHandle;		// キャラクター以外の動くアクターのハンドル
	int shadowMapNoMoveHandle;				// 全く動かないやつのハンドル

	VECTOR shadowCharaLowArea;				// キャラクターの影のマイナス範囲
	VECTOR shadowCharaHighArea;				// キャラクターの影のプラス範囲

	VECTOR shadowAnotherCharaLowArea;		// キャラクター以外動くやつの影のマイナス範囲
	VECTOR shadowAnotherCharaHighArea;		// キャラクター以外動くやつの影のプラス範囲

	VECTOR shadowNoMoveLowArea;				// 全く動かないやつの影のマイナス範囲
	VECTOR shadowNoMoveHighArea;			// 全く動かないやつの影のプラス範囲

	VECTOR lightDire;						// ライトのディレクション方向
	// ---------------------------------------------------------

	// スカイボックスに関して
	int skyBoxUp, skyBoxUnder;

protected:
	// 高負荷覚悟で試したいので試す
	int backGround;


	static bool endFlag;		// シーンの終了フラッグ

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
	void ShadowArea(const VECTOR charaArea);
	// ---------------------------------------------------------


	// 二つのモデルの距離
	int GetDistance(const VECTOR alpha, const VECTOR beta);


	// スカイボックスに関して
	void SkyBoxDraw();
	void SkyBoxProcess(const VECTOR characterArea);
	void SetInitSkyBox(const int skyBoxUp);


public:
	BaseMove();					// コンストラクタ
	virtual ~BaseMove();		// デストラクタ

	virtual void Draw() = 0;										// 描画
	virtual void Process(const unsigned __int8 controllNumber) = 0;		// プロセス
	virtual void CameraProcess() = 0;

	static const bool GetEndFlag();		// 終了ゲッター
	static const ESceneNumber GetScene();	// 今のシーンゲッター

	void SetScene(const ESceneNumber scene);	// 今のシーンセッター
};

