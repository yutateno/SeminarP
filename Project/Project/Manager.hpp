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
	const int max1 = 5;
	std::string move1str[5];
	ELOADFILE load1[5];


	// ムーブ２に関する
	const int max2 = 7;
	std::string move2str[7];
	ELOADFILE load2[7];


	// ムーブ３に関する
	const int max3 = 7;
	std::string move3str[7];
	ELOADFILE load3[7];


	// ムーブ４に関する
	const int max4 = 7;
	std::string move4str[7];
	ELOADFILE load4[7];


	// ムーブ５に関する
	const int max5 = 7;
	std::string move5str[7];
	ELOADFILE load5[7];


	// ムーブ６に関する
	const int max6 = 7;
	std::string move6str[7];
	ELOADFILE load6[7];


	// タイトルに関する
	const int maxTitle = 7;
	std::string moveTitleStr[7];
	ELOADFILE loadTitle[7];



	int antiAliasScreen;



public:
	Manager();			// コンストラクタ
	~Manager();			// デストラクタ

	void Update(const unsigned __int8 controllNumber);		// メインプロセス
};