#pragma once
#include "Basic.hpp"


/// シーンの動き
enum class ESceneNumber
{
	STARTLOAD, FIRSTMOVE, SECONDLOAD, SECONDMOVE, THORDLOAD, THORDMOVE, FORTHLOAD, FORTHMOVE, FIFTHLOAD, FIFTHMOVE
	, SIXTHLOAD, SIXTHMOVE, TITLELOAD, TITLEMOVE
};

/// シーンの親
/// シーンごとに必須なシャドウマップとフォグを行う
class BaseMove
{
private:
	// 影に関して-----------------------------------------------
	/// キャラクターの影のハンドル
	int shadowMapCharaHandle;
	/// キャラクター以外の動くアクターのハンドル
	int shadowMapAnotherCharaHandle;
	/// 全く動かないやつのハンドル
	int shadowMapNoMoveHandle;

	/// キャラクターの影のマイナス範囲
	VECTOR shadowCharaLowArea;				
	/// キャラクターの影のプラス範囲
	VECTOR shadowCharaHighArea;				

	/// キャラクター以外動くやつの影のマイナス範囲
	VECTOR shadowAnotherCharaLowArea;		
	/// キャラクター以外動くやつの影のプラス範囲
	VECTOR shadowAnotherCharaHighArea;		

	/// 全く動かないやつの影のマイナス範囲
	VECTOR shadowNoMoveLowArea;			
	/// 全く動かないやつの影のプラス範囲
	VECTOR shadowNoMoveHighArea;			

	/// ライトのディレクション方向
	VECTOR lightDire;						
	// ---------------------------------------------------------

	/// スカイボックスに関して
	int skyBoxUp, skyBoxUnder;



protected:
	/// シーンの終了フラッグ
	static bool endFlag;		

	/// 現在のシーン
	static ESceneNumber scene;	


	// 影に関して-----------------------------------------------
	// 設定する
	/// シャドウマップ０番：主人公
	void ShadowCharaSetUpBefore();
	void ShadowCharaSetUpAfter();

	/// シャドウマップ１番：主人公以外
	void ShadowAnotherCharaSetUpBefore();
	void ShadowAnotherCharaSetUpAfter();

	/// シャドウマップ２番：動かないもの
	void ShadowNoMoveSetUpBefore();
	void ShadowNoMoveSetUpAfter();

	// 描画へ使用する
	/// シャドウマップ０番：主人公
	void ShadowCharaDrawBefore();
	void ShadowCharaDrawAfter();

	/// シャドウマップ１番：主人公以外
	void ShadowAnotherCharaDrawBefore();
	void ShadowAnotherCharaDrawAfter();

	/// シャドウマップ２番：動かないもの
	void ShadowNoMoveDrawBefore();
	void ShadowNoMoveDrawAfter();

	/// 座標を更新し続ける
	void ShadowArea(const VECTOR charaArea);
	// ---------------------------------------------------------


	/// ある二つのモデルの距離を調べる
	int GetDistance(const VECTOR alpha, const VECTOR beta);


	// スカイボックスに関して-----------------------------------
	/// 描画する
	void SkyBoxDraw();
	/// キャラクターの座標との相対に更新する
	void SkyBoxProcess(const VECTOR characterArea);
	/// 初期化
	void SetInitSkyBox(const int skyBoxUp);



public:
	/// コンストラクタ
	BaseMove();					
	/// デストラクタ
	virtual ~BaseMove();		

	/// 描画
	virtual void Draw() = 0;										
	/// プロセス
	virtual void Process(const unsigned __int8 controllNumber) = 0;		
	/// カメラのプロセス
	virtual void CameraProcess() = 0;

	/// 終了ゲッター
	static const bool GetEndFlag();		
	/// 今のシーンゲッター
	static const ESceneNumber GetScene();	

	/// 今のシーンセッター
	void SetScene(const ESceneNumber scene);	
};

