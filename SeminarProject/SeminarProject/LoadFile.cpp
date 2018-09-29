#include "LoadFile.hpp"

using namespace std;

// �t�@�C���T�C�Y�𒲂ׂ�
unsigned int LoadFile::file_size(ifstream &fin)
{
	unsigned int pos = static_cast<unsigned int>(fin.tellg()); // ���݈ʒu�ۑ�

	unsigned int size = static_cast<unsigned int>(fin.seekg(0, ios::end).tellg()); // �Ō�ɃV�[�N���Ĉʒu�擾���T�C�Y

	fin.seekg(pos); // ���̈ʒu�ɖ߂�

	return size; // �T�C�Y��Ԃ�
}

// �t�@�C���̓ǂݍ���
void LoadFile::MyLoad(const string path, int& file, const ELOADFILE type)
{
	vector<BYTE> data;  // �t�@�C���f�[�^
	UINT size;          // �t�@�C���T�C�Y
	string outstr;		// �o�͂���t�@�C����

	// �t�@�C���̓ǂݍ���
	ifstream fin(path.c_str(), ios::binary); // �t�@�C���I�[�v��

	size = file_size(fin); // �t�@�C���T�C�Y�擾
	data.resize(size); // �������m��
	fin.read((char*)&data[0], size); // �ǂݍ���
	fin.close();

	// ���
	// �S�f�[�^���[�v
	for (UINT i = 0; i < size; i += 5)
	{
		data[i] = (data[i] ^ rad);
	}

	if (type == ELOADFILE::fbxmodel)
	{
		//�ۑ�
		outstr = path;
		outstr.erase(outstr.end() - 2, outstr.end());
		outstr.append("bx");
		ofstream fout(outstr.c_str(), ios::binary);
		fout.write((char*)&data[0], size);
		fout.close();
	}
	else if (type == ELOADFILE::mv1model)
	{
		//�ۑ�
		outstr = path;
		outstr.erase(outstr.end() - 2, outstr.end());
		outstr.append("v1");
		ofstream fout(outstr.c_str(), ios::binary);
		fout.write((char*)&data[0], size);
		fout.close();
	}

	// ���[�h����
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

		// �ꎞ�o�͂������̂��폜
		std::remove(outstr.c_str());
		break;
	case ELOADFILE::mv1model:
		file = MV1LoadModel(outstr.c_str());

		// �ꎞ�o�͂������̂��폜
		std::remove(outstr.c_str());
		break;
	default:
		break;
	}


}
