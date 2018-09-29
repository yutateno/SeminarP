#include "LoadFile.hpp"

using namespace std;

// ファイルサイズを調べる
unsigned int LoadFile::file_size(ifstream &fin)
{
	unsigned int pos = static_cast<unsigned int>(fin.tellg()); // 現在位置保存

	unsigned int size = static_cast<unsigned int>(fin.seekg(0, ios::end).tellg()); // 最後にシークして位置取得→サイズ

	fin.seekg(pos); // 元の位置に戻す

	return size; // サイズを返す
}

// ファイルの読み込み
void LoadFile::MyLoad(const string path, int& file, const ELOADFILE type)
{
	vector<BYTE> data;  // ファイルデータ
	UINT size;          // ファイルサイズ
	string outstr;		// 出力するファイル名

	// ファイルの読み込み
	ifstream fin(path.c_str(), ios::binary); // ファイルオープン

	size = file_size(fin); // ファイルサイズ取得
	data.resize(size); // メモリ確保
	fin.read((char*)&data[0], size); // 読み込み
	fin.close();

	// 解読
	// 全データループ
	for (UINT i = 0; i < size; i += 5)
	{
		data[i] = (data[i] ^ rad);
	}

	if (type == ELOADFILE::fbxmodel)
	{
		//保存
		outstr = path;
		outstr.erase(outstr.end() - 2, outstr.end());
		outstr.append("bx");
		ofstream fout(outstr.c_str(), ios::binary);
		fout.write((char*)&data[0], size);
		fout.close();
	}
	else if (type == ELOADFILE::mv1model)
	{
		//保存
		outstr = path;
		outstr.erase(outstr.end() - 2, outstr.end());
		outstr.append("v1");
		ofstream fout(outstr.c_str(), ios::binary);
		fout.write((char*)&data[0], size);
		fout.close();
	}

	// ロードする
	switch (type)
	{
	case ELOADFILE::graph:
		file = CreateGraphFromMem((char*)&data[0], size);
		break;
	case ELOADFILE::soundmem:
		file = LoadSoundMemByMemImage((char*)&data[0], size);
		break;
	case ELOADFILE::fbxmodel:
		file = MV1LoadModel(outstr.c_str());

		// 一時出力したものを削除
		std::remove(outstr.c_str());
		break;
	case ELOADFILE::mv1model:
		file = MV1LoadModel(outstr.c_str());

		// 一時出力したものを削除
		std::remove(outstr.c_str());
		break;
	default:
		break;
	}


}
