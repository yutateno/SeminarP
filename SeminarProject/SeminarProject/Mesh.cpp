#include "Mesh.h"

Mesh::Mesh()
{
	ZeroMemory(this, sizeof(Mesh));
	m_fScale = 1.0f;
}

Mesh::~Mesh()
{
	SAFE_DELETE_ARRAY(m_pMaterial);
	SAFE_DELETE_ARRAY(m_ppIndexBuffer);
	SAFE_RELEASE(m_pConstantBuffer0);
	SAFE_RELEASE(m_pConstantBuffer1);
	SAFE_RELEASE(m_pVertexShader);
	SAFE_RELEASE(m_pPixelShader);
	SAFE_RELEASE(m_pVertexBuffer);
	SAFE_RELEASE(m_pVertexLayout);
	SAFE_RELEASE(m_pSampleLinear);
	SAFE_RELEASE(m_pTexture);
}

HRESULT Mesh::Init(ID3D11Device* pDevice, ID3D11DeviceContext* pContext, LPSTR FileName)
{
	m_pDevice = pDevice;
	m_pDeviceContext = pContext;

	if (FAILED(InitShader()))
	{
		MessageBox(0, L"���b�V���p�V�F�[�_�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	if (FAILED(LoadStaticMesh(FileName)))
	{
		MessageBox(0, L"���b�V���쐬���s", NULL, MB_OK);
		return E_FAIL;
	}

	return S_OK;
}

HRESULT Mesh::InitShader()
{
	// �u���u�쐬(�V�F�[�_�̉�)
	ID3D10Blob* pCompileShader = NULL;
	ID3D10Blob* pErrors = NULL;

	// �u���u���璸�_�V�F�[�_�쐬
	// �t�@�C���ǂݍ���
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "VS", "vs_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl�ǂݍ��ݎ��s(VS)", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// ���_�V�F�[�_�쐬
	if (FAILED(m_pDevice->CreateVertexShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pVertexShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"���_�V�F�[�_�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	// ���_�C���v�b�g���C�A�E�g���`
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
	};
	UINT numElements = sizeof(layout) / sizeof(layout[0]);
	//���_�C���v�b�g���쐬
	if (FAILED(m_pDevice->CreateInputLayout(layout, numElements, pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), &m_pVertexLayout)))
	{
		return FALSE;
	}

	// �u���u����s�N�Z���V�F�[�_�쐬
	// �t�@�C���ǂݍ���
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "PS", "ps_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"�s�N�Z���V�F�[�_�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// �s�N�Z���V�F�[�_�쐬
	if (FAILED(m_pDevice->CreatePixelShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pPixelShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"�s�N�Z���V�F�[�_�쐬", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pCompileShader);
	// �R���X�^���g�o�b�t�@�[�쐬(�ϊ��s��p)
	D3D11_BUFFER_DESC cb;
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(SIMPLECONSTANT_BUFFER0);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;
	if (FAILED(m_pDevice->CreateBuffer(&cb, NULL, &m_pConstantBuffer0)))
	{
		return E_FAIL;
	}
	// �R���X�^���g�o�b�t�@�[�쐬(�}�e���A���p)
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(SIMPLECONSTANT_BUFFER1);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;
	if (FAILED(m_pDevice->CreateBuffer(&cb, NULL, &m_pConstantBuffer1)))
	{
		return E_FAIL;
	}

	return S_OK;
}

HRESULT Mesh::LoadMaterialFromFile(LPSTR FileName, MY_MATERIAL** ppMaterial)
{
	// �}�e���A���t�@�C�����J���ē��e��ǂݍ���
	FILE* fp = NULL;
	fopen_s(&fp, FileName, "rt");
	char key[110] = { 0 };
	D3DXVECTOR4 v(0, 0, 0, 1);

	// �}�e���A���̐��𒲂ׂ�
	m_dwNumMaterial = 0;
	while (!feof(fp))
	{
		// �L�[���[�h��ǂݍ���
		fscanf_s(fp, "%s", key, sizeof(key));
		// �}�e���A����
		if (strcmp(key, "newmtl") == 0)
		{
			m_dwNumMaterial++;
		}
	}
	// �}�e���A���̐���������
	MY_MATERIAL* pMaterial = new MY_MATERIAL[m_dwNumMaterial];

	// �ǂݍ��݊J�n
	fseek(fp, SEEK_SET, 0);
	INT iMCount = -1;

	while (!feof(fp))
	{
		// �L�[���[�h�ǂݍ���
		fscanf_s(fp, "%s", key, sizeof(key));
		// �}�e���A����
		if (strcmp(key, "newmtl") == 0)
		{
			iMCount++;
			fscanf_s(fp, "%s", key, sizeof(key));
			strcpy_s(pMaterial[iMCount].szName, key);
		}
		// �A���r�G���g
		if (strcmp(key, "Ka") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Ka = v;
		}
		// �f�B�t���[�Y
		if (strcmp(key, "Kd") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Kd = v;
		}
		// �X�y�L�����\
		if (strcmp(key, "Ks") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Ks = v;
		}
		// �e�N�X�`���\
		if (strcmp(key, "map_Kd") == 0)
		{
			fscanf_s(fp, "%s", &pMaterial[iMCount].szTextureName, sizeof(pMaterial[iMCount].szTextureName));
			// �e�N�X�`���\���쐬
			if (FAILED(D3DX11CreateShaderResourceViewFromFileA(m_pDevice, pMaterial[iMCount].szTextureName, NULL, NULL, &pMaterial[iMCount].pTexture, NULL)))
			{
				return E_FAIL;
			}
		}
	}
	fclose(fp);

	*ppMaterial = pMaterial;

	return S_OK;
}

HRESULT Mesh::LoadStaticMesh(LPSTR FileName)
{
	float x, y, z;
	int v1 = 0, v2 = 0, v3 = 0;			// ���_
	int vn1 = 0, vn2 = 0, vn3 = 0;		// �@��
	int vt1 = 0, vt2 = 0, vt3 = 0;		// UV
	// �ǂݍ��݃J�E���^�[
	DWORD dwVCount = 0, dwVNCount = 0, dwVTCount = 0, dwFCount = 0;

	char key[200] = { 0 };
	// obj�t�@�C����ǂݍ���
	FILE* fp = NULL;
	fopen_s(&fp, FileName, "rt");

	// �|���S�����𒲂ׂ�
	while (!feof(fp))
	{
		// �L�[���[�h�ǂݍ���
		fscanf_s(fp, "%s", key, sizeof(key));
		// �}�e���A���ǂݍ���
		if (strcmp(key, "mtllib") == 0)
		{
			fscanf_s(fp, "%s", key, sizeof(key));
			LoadMaterialFromFile(key, &m_pMaterial);
		}
		// ���_
		if (strcmp(key, "v") == 0)
		{
			m_dwNumVert++;
		}
		// �@��
		if (strcmp(key, "vn") == 0)
		{
			dwVNCount++;
		}
		// �e�N�X�`���\���W
		if (strcmp(key, "vt") == 0)
		{
			dwVTCount++;
		}
		// �t�F�C�X
		if (strcmp(key, "f") == 0)
		{
			m_dwNumFace++;
		}
	}

	// ���_�o�b�t�@�ƃC���f�b�N�X�o�b�t�@�̂��߂̈ꎞ�I�ȃ������m��
	MY_VERTEX* pvVertexBuffer = new MY_VERTEX[m_dwNumFace * 3];
	D3DXVECTOR3* pvCoord = new D3DXVECTOR3[m_dwNumVert];
	D3DXVECTOR3* pvNormal = new D3DXVECTOR3[dwVNCount];
	D3DXVECTOR2* pvTexture = new D3DXVECTOR2[dwVTCount];

	// �ǂݍ��݊J�n
	fseek(fp, SEEK_SET, 0);
	dwVCount = 0;
	dwVNCount = 0;
	dwVTCount = 0;
	dwFCount = 0;

	while (!feof(fp))
	{
		// �L�[���[�h�ǂݍ���
		ZeroMemory(key, sizeof(key));
		fscanf_s(fp, "%s", key, sizeof(key));

		// ���_
		if (strcmp(key, "v") == 0)
		{
			fscanf_s(fp, "%f %f %f", &x, &y, &z);
			pvCoord[dwVCount].x = x;
			pvCoord[dwVCount].y = y;
			pvCoord[dwVCount].z = z;
			dwVCount++;
		}
		// �@��
		if (strcmp(key, "vn") == 0)
		{
			fscanf_s(fp, "%f %f %f", &x, &y, &z);
			pvNormal[dwVNCount].x = x;
			pvNormal[dwVNCount].y = y;
			pvNormal[dwVNCount].z = z;
			dwVNCount++;
		}
		// UV
		if (strcmp(key, "vt") == 0)
		{
			fscanf_s(fp, "%f %f", &x, &y);
			pvTexture[dwVTCount].x = x;
			pvTexture[dwVTCount].y = 1 - y;		// obj�t�@�C���ɍ��킹��y�𔽓]
			dwVTCount++;
		}
	}

	// �}�e���A���̐������C���f�b�N�X�o�b�t�@�[���쐬
	m_ppIndexBuffer = new ID3D11Buffer*[m_dwNumMaterial];

	// �}�e���A�����𗊂�Ƀt�@�C�X���q�����킹��
	bool boFlag = false;
	int* piFaceBuffer = new int[m_dwNumFace * 3];
	dwFCount = 0;
	DWORD dwPartFCount = 0;
	for (DWORD i = 0; i < m_dwNumMaterial; i++)
	{
		dwPartFCount = 0;
		fseek(fp, SEEK_SET, 0);

		while (!feof(fp))
		{
			// �L�[���[�h�ǂݍ���
			ZeroMemory(key, sizeof(key));
			fscanf_s(fp, "%s", key, sizeof(key));

			// �t�F�C�X�ǂݍ��݂��Ē��_�C���f�b�N�X�ɕϊ�
			if (strcmp(key, "usemtl") == 0)
			{
				fscanf_s(fp, "%s", key, sizeof(key));
				if (strcmp(key, m_pMaterial[i].szName) == 0)
				{
					boFlag = true;
				}
				else
				{
					boFlag = false;
				}
			}
			if (strcmp(key, "f") == 0 && boFlag == true)
			{
				if (m_pMaterial[i].pTexture != NULL)	// �e�N�X�`���\����T�[�t�F�C�X
				{
					fscanf_s(fp, "%d/%d/%d %d/%d/%d %d/%d/%d", &v1, &vt1, &vn1, &v2, &vt2, &vn2, &v3, &vt3, &vn3);
				}
				else	// �e�N�X�`���\�����T�[�t�F�C�X
				{
					fscanf_s(fp, "%d//%d %d//%d %d//%d", &v1, &vn1, &v2, &vn2, &v3, &vn3);
				}

				// �C���f�b�N�X�o�b�t�@�[
				piFaceBuffer[dwPartFCount * 3] = dwFCount * 3;
				piFaceBuffer[dwPartFCount * 3 + 1] = dwFCount * 3 + 1;
				piFaceBuffer[dwPartFCount * 3 + 2] = dwFCount * 3 + 2;
				// ���_�\���̂ɂ��ꂼ����
				pvVertexBuffer[dwFCount * 3].vPos = pvCoord[v1 - 1];
				pvVertexBuffer[dwFCount * 3].vNorm = pvNormal[vn1 - 1];
				pvVertexBuffer[dwFCount * 3].vTex = pvTexture[vt1 - 1];
				pvVertexBuffer[dwFCount * 3 + 1].vPos = pvCoord[v2 - 1];
				pvVertexBuffer[dwFCount * 3 + 1].vNorm = pvNormal[vn2 - 1];
				pvVertexBuffer[dwFCount * 3 + 1].vTex = pvTexture[vt2 - 1];
				pvVertexBuffer[dwFCount * 3 + 2].vPos = pvCoord[v3 - 1];
				pvVertexBuffer[dwFCount * 3 + 2].vNorm = pvNormal[vn3 - 1];
				pvVertexBuffer[dwFCount * 3 + 2].vTex = pvTexture[vt3 - 1];

				dwPartFCount++;
				dwFCount++;
			}
		}
		if (dwFCount == 0)	// �g�p����Ă��Ȃ��}�e���A���̑΍�
		{
			m_ppIndexBuffer[i] = NULL;
			continue;
		}

		// �C���f�b�N�X�o�b�t�@�[���쐬
		D3D11_BUFFER_DESC bd;
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.ByteWidth = sizeof(int)*dwFCount * 3;
		bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
		bd.CPUAccessFlags = 0;
		bd.MiscFlags = 0;
		D3D11_SUBRESOURCE_DATA Initdata;
		Initdata.pSysMem = piFaceBuffer;
		Initdata.SysMemPitch = 0;
		Initdata.SysMemSlicePitch = 0;
		if (FAILED(m_pDevice->CreateBuffer(&bd, &Initdata, &m_ppIndexBuffer[i])))
		{
			return FALSE;
		}
		m_pMaterial[i].dwNumFace = dwFCount;
	}
	delete[] piFaceBuffer;
	fclose(fp);

	// �o�[�e�b�N�X�o�b�t�@�[���쐬
	D3D11_BUFFER_DESC bd;
	bd.Usage = D3D11_USAGE_DEFAULT;
	bd.ByteWidth = sizeof(MY_VERTEX)*m_dwNumFace * 3;
	bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
	bd.CPUAccessFlags = 0;
	bd.MiscFlags = 0;
	D3D11_SUBRESOURCE_DATA InitData;
	InitData.pSysMem = pvVertexBuffer;
	if (FAILED(m_pDevice->CreateBuffer(&bd, &InitData, &m_pVertexBuffer)))
	{
		return FALSE;
	}

	// �ꎞ�I�ɍ�����s�v�Ȃ��̂��폜
	delete pvCoord;
	delete pvNormal;
	delete pvTexture;
	delete[] pvVertexBuffer;

	// �e�N�X�`���\�p�T���v���[�쐬
	D3D11_SAMPLER_DESC SamDesc;
	ZeroMemory(&SamDesc, sizeof(D3D11_SAMPLER_DESC));

	SamDesc.Filter = D3D11_FILTER_MIN_MAG_MIP_LINEAR;
	SamDesc.AddressU = D3D11_TEXTURE_ADDRESS_WRAP;
	SamDesc.AddressV = D3D11_TEXTURE_ADDRESS_WRAP;
	SamDesc.AddressW = D3D11_TEXTURE_ADDRESS_WRAP;
	m_pDevice->CreateSamplerState(&SamDesc, &m_pSampleLinear);

	return S_OK;
}

void Mesh::Render(D3DXMATRIX& mView, D3DXMATRIX& mProj, D3DXVECTOR3& vLight, D3DXVECTOR3& vEye)
{
	D3DXMATRIX mWorld, mTran, mYaw, mPitch, mRoll, mScale;
	// ���[���h���W�ϊ�
	D3DXMatrixScaling(&mScale, m_fScale, m_fScale, m_fScale);
	D3DXMatrixRotationY(&mYaw, m_fYaw);
	D3DXMatrixRotationX(&mPitch, m_fPitch);
	D3DXMatrixRotationZ(&mRoll, m_fRoll);
	D3DXMatrixTranslation(&mTran, m_vPos.x, m_vPos.y, m_vPos.z);

	mWorld = mScale * mYaw * mPitch * mRoll * mTran;

	// �g�p����V�F�[�_��o�^
	m_pDeviceContext->VSSetShader(m_pVertexShader, NULL, 0);
	m_pDeviceContext->PSSetShader(m_pPixelShader, NULL, 0);

	// �V�F�[�_�[�̃R���X�^���g�o�b�t�@�[�Ƀf�[�^��n��
	D3D11_MAPPED_SUBRESOURCE pData;
	if (SUCCEEDED(m_pDeviceContext->Map(m_pConstantBuffer0, 0, D3D11_MAP_WRITE_DISCARD, 0, &pData)))
	{
		SIMPLECONSTANT_BUFFER0 sg;
		// ���[���h���W��n��
		sg.mW = mWorld;
		D3DXMatrixTranspose(&sg.mW, &sg.mW);
		// ���[���h�A�J�����A�ˉe�s���n��
		D3DXMATRIX m = mWorld * mView * mProj;
		D3DXMatrixTranspose(&m, &m);
		sg.mWVP = m;
		// ���C�g�̈ʒu��n��
		int row = sqrt((double)MAX_LIGHT);
		for (int i = 0; i < row; i++)
		{
			for (int k = 0; k < row; k++)
			{
				sg.vLightPos[i*row + k] = D3DXVECTOR4(-row * 1.5 + k * 3, 1, -row * 1.5 + i * 3, 1.0f);
			}
		}
		// ���_�ʒu��n��
		sg.vEye = D3DXVECTOR4(vEye.x, vEye.y, vEye.z, 0);

		memcpy_s(pData.pData, pData.RowPitch, (void*)&sg, sizeof(SIMPLECONSTANT_BUFFER0));
		m_pDeviceContext->Unmap(m_pConstantBuffer0, 0);
	}
	// �R���X�^���g�o�b�t�@�[���g���V�F�[�_�̓o�^
	m_pDeviceContext->VSSetConstantBuffers(0, 1, &m_pConstantBuffer0);
	m_pDeviceContext->PSSetConstantBuffers(0, 1, &m_pConstantBuffer0);
	// ���_�C���v�b�g���C�A�E�g���Z�b�g
	m_pDeviceContext->IASetInputLayout(m_pVertexLayout);
	// �v���~�e�B�u�E�g�|���W�[���Z�b�g
	m_pDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	// ���_�o�b�t�@�\���Z�b�g
	UINT stride = sizeof(MY_VERTEX);
	UINT offset = 0;
	m_pDeviceContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
	// �}�e���A���̐������A���ꂼ��̃}�e���A���̃C���f�b�N�X�o�b�t�@�[��`��
	for (DWORD i = 0; i < m_dwNumMaterial; i++)
	{
		// �g�p����Ȃ��}�e���A���΍�
		if (m_pMaterial[i].dwNumFace == 0)
		{
			continue;
		}
		// �C���f�b�N�X�o�b�t�@�[���Z�b�g
		stride = sizeof(int);
		offset = 0;
		m_pDeviceContext->IASetIndexBuffer(m_ppIndexBuffer[i], DXGI_FORMAT_R32_UINT, 0);
		// �}�e���A���̊e�v�f���V�F�[�_�[�ɓn��
		D3D11_MAPPED_SUBRESOURCE pData;
		if (SUCCEEDED(m_pDeviceContext->Map(m_pConstantBuffer1, 0, D3D11_MAP_WRITE_DISCARD, 0, &pData)))
		{
			SIMPLECONSTANT_BUFFER1 sg;
			sg.vAmbient = m_pMaterial[i].Ka;
			sg.vDiffuse = m_pMaterial[i].Kd;
			sg.vSpecular = m_pMaterial[i].Ks;
			memcpy_s(pData.pData, pData.RowPitch, (void*)&sg, sizeof(SIMPLECONSTANT_BUFFER1));
			m_pDeviceContext->Unmap(m_pConstantBuffer1, 0);
		}
		m_pDeviceContext->VSSetConstantBuffers(1, 1, &m_pConstantBuffer1);
		m_pDeviceContext->PSSetConstantBuffers(1, 1, &m_pConstantBuffer1);
		// �e�N�X�`�����V�F�[�_�[�ɓn��
		if (m_pMaterial[i].szTextureName[0] != NULL)
		{
			m_pDeviceContext->PSSetSamplers(0, 1, &m_pSampleLinear);
			m_pDeviceContext->PSSetShaderResources(0, 1, &m_pMaterial[i].pTexture);
		}
		// �v���~�e�B�u�������_�����O
		m_pDeviceContext->DrawIndexed(m_pMaterial[i].dwNumFace * 3, 0, 0);
	}
}