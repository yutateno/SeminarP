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
		MessageBox(0, L"メッシュ用シェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	if (FAILED(LoadStaticMesh(FileName)))
	{
		MessageBox(0, L"メッシュ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}

	return S_OK;
}

HRESULT Mesh::InitShader()
{
	// ブロブ作成(シェーダの塊)
	ID3D10Blob* pCompileShader = NULL;
	ID3D10Blob* pErrors = NULL;

	// ブロブから頂点シェーダ作成
	// ファイル読み込み
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "VS", "vs_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl読み込み失敗(VS)", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// 頂点シェーダ作成
	if (FAILED(m_pDevice->CreateVertexShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pVertexShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"頂点シェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	// 頂点インプットレイアウトを定義
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{"POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0},
		{"TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0},
	};
	UINT numElements = sizeof(layout) / sizeof(layout[0]);
	//頂点インプットを作成
	if (FAILED(m_pDevice->CreateInputLayout(layout, numElements, pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), &m_pVertexLayout)))
	{
		return FALSE;
	}

	// ブロブからピクセルシェーダ作成
	// ファイル読み込み
	if (FAILED(D3DX11CompileFromFile(L"Simple.hlsl", NULL, NULL, "PS", "ps_5_0", 0, 0, NULL, &pCompileShader, &pErrors, NULL)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"ピクセルシェーダ作成失敗", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	// ピクセルシェーダ作成
	if (FAILED(m_pDevice->CreatePixelShader(pCompileShader->GetBufferPointer(), pCompileShader->GetBufferSize(), NULL, &m_pPixelShader)))
	{
		SAFE_RELEASE(pCompileShader);
		MessageBox(0, L"ピクセルシェーダ作成", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pCompileShader);
	// コンスタントバッファー作成(変換行列用)
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
	// コンスタントバッファー作成(マテリアル用)
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
	// マテリアルファイルを開いて内容を読み込む
	FILE* fp = NULL;
	fopen_s(&fp, FileName, "rt");
	char key[110] = { 0 };
	D3DXVECTOR4 v(0, 0, 0, 1);

	// マテリアルの数を調べる
	m_dwNumMaterial = 0;
	while (!feof(fp))
	{
		// キーワードを読み込む
		fscanf_s(fp, "%s", key, sizeof(key));
		// マテリアル名
		if (strcmp(key, "newmtl") == 0)
		{
			m_dwNumMaterial++;
		}
	}
	// マテリアルの数だけ生成
	MY_MATERIAL* pMaterial = new MY_MATERIAL[m_dwNumMaterial];

	// 読み込み開始
	fseek(fp, SEEK_SET, 0);
	INT iMCount = -1;

	while (!feof(fp))
	{
		// キーワード読み込み
		fscanf_s(fp, "%s", key, sizeof(key));
		// マテリアル名
		if (strcmp(key, "newmtl") == 0)
		{
			iMCount++;
			fscanf_s(fp, "%s", key, sizeof(key));
			strcpy_s(pMaterial[iMCount].szName, key);
		}
		// アンビエント
		if (strcmp(key, "Ka") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Ka = v;
		}
		// ディフューズ
		if (strcmp(key, "Kd") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Kd = v;
		}
		// スペキュラ―
		if (strcmp(key, "Ks") == 0)
		{
			fscanf_s(fp, "%f %f %f", &v.x, &v.y, &v.z);
			pMaterial[iMCount].Ks = v;
		}
		// テクスチャ―
		if (strcmp(key, "map_Kd") == 0)
		{
			fscanf_s(fp, "%s", &pMaterial[iMCount].szTextureName, sizeof(pMaterial[iMCount].szTextureName));
			// テクスチャ―を作成
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
	int v1 = 0, v2 = 0, v3 = 0;			// 頂点
	int vn1 = 0, vn2 = 0, vn3 = 0;		// 法線
	int vt1 = 0, vt2 = 0, vt3 = 0;		// UV
	// 読み込みカウンター
	DWORD dwVCount = 0, dwVNCount = 0, dwVTCount = 0, dwFCount = 0;

	char key[200] = { 0 };
	// objファイルを読み込む
	FILE* fp = NULL;
	fopen_s(&fp, FileName, "rt");

	// ポリゴン数を調べる
	while (!feof(fp))
	{
		// キーワード読み込み
		fscanf_s(fp, "%s", key, sizeof(key));
		// マテリアル読み込み
		if (strcmp(key, "mtllib") == 0)
		{
			fscanf_s(fp, "%s", key, sizeof(key));
			LoadMaterialFromFile(key, &m_pMaterial);
		}
		// 頂点
		if (strcmp(key, "v") == 0)
		{
			m_dwNumVert++;
		}
		// 法線
		if (strcmp(key, "vn") == 0)
		{
			dwVNCount++;
		}
		// テクスチャ―座標
		if (strcmp(key, "vt") == 0)
		{
			dwVTCount++;
		}
		// フェイス
		if (strcmp(key, "f") == 0)
		{
			m_dwNumFace++;
		}
	}

	// 頂点バッファとインデックスバッファのための一時的なメモリ確保
	MY_VERTEX* pvVertexBuffer = new MY_VERTEX[m_dwNumFace * 3];
	D3DXVECTOR3* pvCoord = new D3DXVECTOR3[m_dwNumVert];
	D3DXVECTOR3* pvNormal = new D3DXVECTOR3[dwVNCount];
	D3DXVECTOR2* pvTexture = new D3DXVECTOR2[dwVTCount];

	// 読み込み開始
	fseek(fp, SEEK_SET, 0);
	dwVCount = 0;
	dwVNCount = 0;
	dwVTCount = 0;
	dwFCount = 0;

	while (!feof(fp))
	{
		// キーワード読み込み
		ZeroMemory(key, sizeof(key));
		fscanf_s(fp, "%s", key, sizeof(key));

		// 頂点
		if (strcmp(key, "v") == 0)
		{
			fscanf_s(fp, "%f %f %f", &x, &y, &z);
			pvCoord[dwVCount].x = x;
			pvCoord[dwVCount].y = y;
			pvCoord[dwVCount].z = z;
			dwVCount++;
		}
		// 法線
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
			pvTexture[dwVTCount].y = 1 - y;		// objファイルに合わせてyを反転
			dwVTCount++;
		}
	}

	// マテリアルの数だけインデックスバッファーを作成
	m_ppIndexBuffer = new ID3D11Buffer*[m_dwNumMaterial];

	// マテリアル名を頼りにファイスを繋ぎ合わせる
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
			// キーワード読み込み
			ZeroMemory(key, sizeof(key));
			fscanf_s(fp, "%s", key, sizeof(key));

			// フェイス読み込みして頂点インデックスに変換
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
				if (m_pMaterial[i].pTexture != NULL)	// テクスチャ―ありサーフェイス
				{
					fscanf_s(fp, "%d/%d/%d %d/%d/%d %d/%d/%d", &v1, &vt1, &vn1, &v2, &vt2, &vn2, &v3, &vt3, &vn3);
				}
				else	// テクスチャ―無しサーフェイス
				{
					fscanf_s(fp, "%d//%d %d//%d %d//%d", &v1, &vn1, &v2, &vn2, &v3, &vn3);
				}

				// インデックスバッファー
				piFaceBuffer[dwPartFCount * 3] = dwFCount * 3;
				piFaceBuffer[dwPartFCount * 3 + 1] = dwFCount * 3 + 1;
				piFaceBuffer[dwPartFCount * 3 + 2] = dwFCount * 3 + 2;
				// 頂点構造体にそれぞれ代入
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
		if (dwFCount == 0)	// 使用されていないマテリアルの対策
		{
			m_ppIndexBuffer[i] = NULL;
			continue;
		}

		// インデックスバッファーを作成
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

	// バーテックスバッファーを作成
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

	// 一時的に作った不要なものを削除
	delete pvCoord;
	delete pvNormal;
	delete pvTexture;
	delete[] pvVertexBuffer;

	// テクスチャ―用サンプラー作成
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
	// ワールド座標変換
	D3DXMatrixScaling(&mScale, m_fScale, m_fScale, m_fScale);
	D3DXMatrixRotationY(&mYaw, m_fYaw);
	D3DXMatrixRotationX(&mPitch, m_fPitch);
	D3DXMatrixRotationZ(&mRoll, m_fRoll);
	D3DXMatrixTranslation(&mTran, m_vPos.x, m_vPos.y, m_vPos.z);

	mWorld = mScale * mYaw * mPitch * mRoll * mTran;

	// 使用するシェーダを登録
	m_pDeviceContext->VSSetShader(m_pVertexShader, NULL, 0);
	m_pDeviceContext->PSSetShader(m_pPixelShader, NULL, 0);

	// シェーダーのコンスタントバッファーにデータを渡す
	D3D11_MAPPED_SUBRESOURCE pData;
	if (SUCCEEDED(m_pDeviceContext->Map(m_pConstantBuffer0, 0, D3D11_MAP_WRITE_DISCARD, 0, &pData)))
	{
		SIMPLECONSTANT_BUFFER0 sg;
		// ワールド座標を渡す
		sg.mW = mWorld;
		D3DXMatrixTranspose(&sg.mW, &sg.mW);
		// ワールド、カメラ、射影行列を渡す
		D3DXMATRIX m = mWorld * mView * mProj;
		D3DXMatrixTranspose(&m, &m);
		sg.mWVP = m;
		// ライトの位置を渡す
		int row = sqrt((double)MAX_LIGHT);
		for (int i = 0; i < row; i++)
		{
			for (int k = 0; k < row; k++)
			{
				sg.vLightPos[i*row + k] = D3DXVECTOR4(-row * 1.5 + k * 3, 1, -row * 1.5 + i * 3, 1.0f);
			}
		}
		// 視点位置を渡す
		sg.vEye = D3DXVECTOR4(vEye.x, vEye.y, vEye.z, 0);

		memcpy_s(pData.pData, pData.RowPitch, (void*)&sg, sizeof(SIMPLECONSTANT_BUFFER0));
		m_pDeviceContext->Unmap(m_pConstantBuffer0, 0);
	}
	// コンスタントバッファーを使うシェーダの登録
	m_pDeviceContext->VSSetConstantBuffers(0, 1, &m_pConstantBuffer0);
	m_pDeviceContext->PSSetConstantBuffers(0, 1, &m_pConstantBuffer0);
	// 頂点インプットレイアウトをセット
	m_pDeviceContext->IASetInputLayout(m_pVertexLayout);
	// プリミティブ・トポロジーをセット
	m_pDeviceContext->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);
	// 頂点バッファ―をセット
	UINT stride = sizeof(MY_VERTEX);
	UINT offset = 0;
	m_pDeviceContext->IASetVertexBuffers(0, 1, &m_pVertexBuffer, &stride, &offset);
	// マテリアルの数だけ、それぞれのマテリアルのインデックスバッファーを描画
	for (DWORD i = 0; i < m_dwNumMaterial; i++)
	{
		// 使用されないマテリアル対策
		if (m_pMaterial[i].dwNumFace == 0)
		{
			continue;
		}
		// インデックスバッファーをセット
		stride = sizeof(int);
		offset = 0;
		m_pDeviceContext->IASetIndexBuffer(m_ppIndexBuffer[i], DXGI_FORMAT_R32_UINT, 0);
		// マテリアルの各要素をシェーダーに渡す
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
		// テクスチャをシェーダーに渡す
		if (m_pMaterial[i].szTextureName[0] != NULL)
		{
			m_pDeviceContext->PSSetSamplers(0, 1, &m_pSampleLinear);
			m_pDeviceContext->PSSetShaderResources(0, 1, &m_pMaterial[i].pTexture);
		}
		// プリミティブをレンダリング
		m_pDeviceContext->DrawIndexed(m_pMaterial[i].dwNumFace * 3, 0, 0);
	}
}