#pragma once
#include "BaseMove.hpp"
#include "MainMove1.hpp"
#include "LoadThread.hpp"
#include "MainMove2.hpp"


class Manager
{
private:
	ESceneNumber e_preScene;		// 直前のシーン
	ESceneNumber e_nowScene;		// 今のシーン


	BaseMove* p_baseMove;			// シーンの基底クラス
	LoadThread* p_loadThread;		// ロードのクラス


	void SceneChange();				// シーンを切り替える


	// ムーブ１に関する
	const int max1 = 4;
	std::string move1str[4];
	ELOADFILE load1[4];



	// ムーブ２に関する
	const int max2 = 3;
	std::string move2str[3];
	ELOADFILE load2[3];



public:
	Manager();			// コンストラクタ
	~Manager();			// デストラクタ

	void Update(unsigned __int8 controllNumber);		// メインプロセス
};