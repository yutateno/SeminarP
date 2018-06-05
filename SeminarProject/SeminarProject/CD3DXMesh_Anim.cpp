#include "CD3DXMESH_ANIM.h"
//�{�����O�̃T���v���ł́A���̃N���X�͓ǂݍ��݂̂ݍs�Ȃ����̂��������A
//�{�T���v���ł́A�A�j���[�V�����s��쐬�ɂ��֘A����B

//
//
//
HRESULT MY_H_HIERARCHY::CreateFrame(LPCSTR Name, LPD3DXFRAME *ppNewFrame)
{
	HRESULT hr = S_OK;
	MYHFRAME *pFrame;
	*ppNewFrame = NULL;

	pFrame = new MYHFRAME;
	if (pFrame == NULL)
	{
		return E_OUTOFMEMORY;
	}
	pFrame->Name = new CHAR[lstrlenA(Name) + 1];
	if (!pFrame->Name)
	{
		return E_FAIL;
	}
	strcpy(pFrame->Name, Name);

	D3DXMatrixIdentity(&pFrame->TransformationMatrix);
	D3DXMatrixIdentity(&pFrame->CombinedTransformationMatrix);
	pFrame->pMeshContainer = NULL;
	pFrame->pFrameSibling = NULL;
	pFrame->pFrameFirstChild = NULL;
	*ppNewFrame = pFrame;

	return S_OK;
}

//
//HRESULT MY_HIERARCHY::CreateMeshContainer
//���b�V���R���e�i�[���쐬����
HRESULT MY_H_HIERARCHY::CreateMeshContainer(LPCSTR Name, CONST D3DXMESHDATA* pMeshData,
	CONST D3DXMATERIAL* pMaterials, CONST D3DXEFFECTINSTANCE* pEffectInstances,
	DWORD NumMaterials, CONST DWORD *pAdjacency, LPD3DXSKININFO pSkinInfo,
	LPD3DXMESHCONTAINER *ppMeshContainer)
{
	HRESULT hr;
	MYHMESHCONTAINER *pMeshContainer = NULL;
	int iFacesAmount;
	int iMaterial;
	LPDIRECT3DDEVICE9 pDevice = NULL;
	LPD3DXMESH pMesh = NULL;
	*ppMeshContainer = NULL;

	pMesh = pMeshData->pMesh;
	pMeshContainer = new MYHMESHCONTAINER;
	if (pMeshContainer == NULL)
	{
		return E_OUTOFMEMORY;
	}
	ZeroMemory(pMeshContainer, sizeof(MYHMESHCONTAINER));

	pMeshContainer->Name = new CHAR[strlen(Name) + 1];
	if (!pMeshContainer->Name)
	{
		return E_FAIL;
	}
	strcpy(pMeshContainer->Name, Name);
	pMesh->GetDevice(&pDevice);
	iFacesAmount = pMesh->GetNumFaces();

	// ���Y���b�V�����@���������Ȃ��ꍇ�͖@����ǉ�����
	if (!(pMesh->GetFVF() & D3DFVF_NORMAL))
	{
		pMeshContainer->MeshData.Type = D3DXMESHTYPE_MESH;
		hr = pMesh->CloneMeshFVF(pMesh->GetOptions(),
			pMesh->GetFVF() | D3DFVF_NORMAL,
			pDevice, &pMeshContainer->MeshData.pMesh);
		if (FAILED(hr))
		{
			return E_FAIL;
		}

		pMesh = pMeshContainer->MeshData.pMesh;
		D3DXComputeNormals(pMesh, NULL);
	}
	else
	{
		pMeshContainer->MeshData.pMesh = pMesh;
		pMeshContainer->MeshData.Type = D3DXMESHTYPE_MESH;
		pMesh->AddRef();
	}
	pMeshContainer->NumMaterials = max(1, NumMaterials);
	pMeshContainer->pMaterials = new D3DXMATERIAL[pMeshContainer->NumMaterials];
	pMeshContainer->ppTextures = new LPDIRECT3DTEXTURE9[pMeshContainer->NumMaterials];
	pMeshContainer->pAdjacency = new DWORD[iFacesAmount * 3];
	if ((pMeshContainer->pAdjacency == NULL) || (pMeshContainer->pMaterials == NULL))
	{
		return E_OUTOFMEMORY;
	}

	memcpy(pMeshContainer->pAdjacency, pAdjacency, sizeof(DWORD) * iFacesAmount * 3);
	memset(pMeshContainer->ppTextures, 0, sizeof(LPDIRECT3DTEXTURE9) * pMeshContainer->NumMaterials);

	if (NumMaterials > 0)
	{
		memcpy(pMeshContainer->pMaterials, pMaterials, sizeof(D3DXMATERIAL) * NumMaterials);

		for (iMaterial = 0; iMaterial < NumMaterials; iMaterial++)
		{
			if (pMaterials[iMaterial].pTextureFilename != NULL)
			{
				pMeshContainer->pMaterials[iMaterial].pTextureFilename = new char[strlen(pMaterials[iMaterial].pTextureFilename) + 1];
				strcpy(pMeshContainer->pMaterials[iMaterial].pTextureFilename, pMaterials[iMaterial].pTextureFilename);
			}
		}
	}
	else
	{
		pMeshContainer->pMaterials[0].pTextureFilename = NULL;
		memset(&pMeshContainer->pMaterials[0].MatD3D, 0, sizeof(D3DMATERIAL9));
		pMeshContainer->pMaterials[0].MatD3D.Diffuse.r = 0.5f;
		pMeshContainer->pMaterials[0].MatD3D.Diffuse.g = 0.5f;
		pMeshContainer->pMaterials[0].MatD3D.Diffuse.b = 0.5f;
		pMeshContainer->pMaterials[0].MatD3D.Specular = pMeshContainer->pMaterials[0].MatD3D.Diffuse;
	}
	*ppMeshContainer = pMeshContainer;
	(*ppMeshContainer)->pMaterials[0].pTextureFilename;
	pMeshContainer = NULL;

	return S_OK;
}

//
//HRESULT MY_HIERARCHY::DestroyFrame(LPD3DXFRAME pFrameToFree) 
//�t���[����j������
HRESULT MY_H_HIERARCHY::DestroyFrame(LPD3DXFRAME pFrameToFree)
{
	if (pFrameToFree->pFrameFirstChild)
	{
		DestroyFrame(pFrameToFree->pFrameFirstChild);
	}
	if (pFrameToFree->pFrameSibling)
	{
		DestroyFrame(pFrameToFree->pFrameSibling);
	}

	SAFE_DELETE_ARRAY(pFrameToFree->Name);
	SAFE_DELETE(pFrameToFree);

	return S_OK;
}
//
//HRESULT MY_HIERARCHY::DestroyMeshContainer(LPD3DXMESHCONTAINER pMeshContainerBase)
//���b�V���R���e�i�[��j������
HRESULT MY_H_HIERARCHY::DestroyMeshContainer(LPD3DXMESHCONTAINER pMeshContainerBase)
{
	int iMaterial;
	MYHMESHCONTAINER *pMeshContainer = (MYHMESHCONTAINER*)pMeshContainerBase;

	SAFE_RELEASE(pMeshContainer->pSkinInfo);
	SAFE_DELETE_ARRAY(pMeshContainer->Name);
	SAFE_DELETE_ARRAY(pMeshContainer->pAdjacency);
	SAFE_DELETE_ARRAY(pMeshContainer->pMaterials);

	if (pMeshContainer->ppTextures != NULL)
	{
		for (iMaterial = 0; iMaterial < pMeshContainer->NumMaterials; iMaterial++)
		{
			SAFE_RELEASE(pMeshContainer->ppTextures[iMaterial]);

		}
		SAFE_DELETE_ARRAY(pMeshContainer->ppTextures);
	}
	SAFE_RELEASE(pMeshContainer->MeshData.pMesh);
	SAFE_DELETE(pMeshContainer);

	return S_OK;
}

//
//
//
CD3DXMESH_ANIM::CD3DXMESH_ANIM()
{
	ZeroMemory(this, sizeof(CD3DXMESH_ANIM));
	m_fScale = 1.0f;
}
//
//
//
CD3DXMESH_ANIM::~CD3DXMESH_ANIM()
{
	SAFE_RELEASE(m_pMesh);
	SAFE_RELEASE(m_pDevice9);
	SAFE_RELEASE(m_pD3d9);
}
//
//
//
HRESULT CD3DXMESH_ANIM::Init(HWND hWnd, ID3D11Device* pDevice11, ID3D11DeviceContext* pContext11, LPSTR FileName)
{
	m_hWnd = hWnd;
	m_pDevice11 = pDevice11;
	m_pDeviceContext11 = pContext11;

	if (FAILED(InitDx9()))
	{
		return E_FAIL;
	}
	if (FAILED(LoadXAnimMesh(FileName)))
	{
		return E_FAIL;
	}

	if (FAILED(InitShader()))
	{
		return E_FAIL;
	}



	return S_OK;
}
//
//
//D3DX�̃p�[�T�[���g�����߂ɂ́ADx9�̃f�o�C�X���K�v�Ȃ̂ō쐬����B
HRESULT CD3DXMESH_ANIM::InitDx9()
{
	// �uDirect3D�v�I�u�W�F�N�g�̍쐬
	if (NULL == (m_pD3d9 = Direct3DCreate9(D3D_SDK_VERSION)))
	{
		MessageBoxA(0, "Direct3D9�̍쐬�Ɏ��s���܂���", "", MB_OK);
		return E_FAIL;
	}
	// �uDIRECT3D�f�o�C�X�v�I�u�W�F�N�g�̍쐬
	D3DPRESENT_PARAMETERS d3dpp;
	ZeroMemory(&d3dpp, sizeof(d3dpp));
	d3dpp.BackBufferFormat = D3DFMT_UNKNOWN;
	d3dpp.BackBufferCount = 1;
	d3dpp.SwapEffect = D3DSWAPEFFECT_DISCARD;
	d3dpp.Windowed = true;
	d3dpp.EnableAutoDepthStencil = true;
	d3dpp.AutoDepthStencilFormat = D3DFMT_D16;

	if (FAILED(m_pD3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, m_hWnd,
		D3DCREATE_HARDWARE_VERTEXPROCESSING,
		&d3dpp, &m_pDevice9)))
	{
		if (FAILED(m_pD3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_HAL, m_hWnd,
			D3DCREATE_SOFTWARE_VERTEXPROCESSING,
			&d3dpp, &m_pDevice9)))
		{
			MessageBoxA(0, "HAL���[�h��DIRECT3D�f�o�C�X���쐬�ł��܂���\nREF���[�h�ōĎ��s���܂�", NULL, MB_OK);
			if (FAILED(m_pD3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_REF, m_hWnd,
				D3DCREATE_HARDWARE_VERTEXPROCESSING,
				&d3dpp, &m_pDevice9)))
			{
				if (FAILED(m_pD3d9->CreateDevice(D3DADAPTER_DEFAULT, D3DDEVTYPE_REF, m_hWnd,
					D3DCREATE_SOFTWARE_VERTEXPROCESSING,
					&d3dpp, &m_pDevice9)))
				{
					MessageBoxA(0, "DIRECT3D�f�o�C�X�̍쐬�Ɏ��s���܂���", NULL, MB_OK);
					return E_FAIL;
				}
			}
		}
	}
	return S_OK;
}
//
//
// X�t�@�C�����烁�b�V�������[�h����
HRESULT CD3DXMESH_ANIM::LoadXAnimMesh(LPSTR FileName)
{
	m_pHierarchy = new MY_H_HIERARCHY;
	if (FAILED(D3DXLoadMeshHierarchyFromXA(FileName, D3DXMESH_MANAGED, m_pDevice9,
		m_pHierarchy, NULL, &m_pFrameRoot, &m_pAnimController)))
	{
		MessageBoxA(NULL, "X�t�@�C���̓ǂݍ��݂Ɏ��s���܂���", NULL, MB_OK);
		return E_FAIL;
	}
	//���̎��_�ŁA�t�@�C��������o�����S�t���[����m_pFrameRoot�ɓ����Ă���A
	//�܂��A�j���[�V�������R���g���[������ɂ�m_pAnimController������������Ă���͂��Ȃ̂ŁA������g���B

	//���Ƃ́A��������K�v�ȏ����Ƃ肾���A�����t���[�����ƂɃA�v���Ǝ��̃��b�V�����\�z���Ă���
	BuildAllMesh(m_pFrameRoot);

	return S_OK;
}
//
//
//D�RDXMESH����A�v�����b�V�����쐬����
void CD3DXMESH_ANIM::BuildAllMesh(D3DXFRAME* pFrame)
{
	if (pFrame && pFrame->pMeshContainer)
	{
		CreateAppMeshFromD3DXMesh(pFrame);
	}

	if (pFrame->pFrameSibling != NULL)
	{
		BuildAllMesh(pFrame->pFrameSibling);
	}

	if (pFrame->pFrameFirstChild != NULL)
	{
		BuildAllMesh(pFrame->pFrameFirstChild);
	}
}
//
//
//
HRESULT CD3DXMESH_ANIM::CreateAppMeshFromD3DXMesh(LPD3DXFRAME p)
{
	//���̎��_�ŁA�t�@�C��������o�������b�V���f�[�^���ADx9��D3DX���b�V���ɓ����Ă���A���Ƃ́A��������D���ȏ������o����Dx11�̎��O���b�V���ɗ��p���邾���B
	D3D11_BUFFER_DESC bd;
	D3D11_SUBRESOURCE_DATA InitData;

	MYHFRAME* pFrame = (MYHFRAME*)p;

	LPD3DXMESH pD3DXMesh = pFrame->pMeshContainer->MeshData.pMesh;//D3DX���b�V���i��������E�E�E�j
	PARTS_MESH* pAppMesh = new PARTS_MESH;//�A�v�����b�V���i�E�E�E�����Ƀ��b�V���f�[�^���R�s�[����j
	pAppMesh->Tex = false;

	pAppMesh->dwNumMaterial = pFrame->pMeshContainer->NumMaterials;

	pAppMesh->pMaterial = new MY_HMATERIAL[pAppMesh->dwNumMaterial];
	pAppMesh->ppIndexBuffer = new ID3D11Buffer*[pAppMesh->dwNumMaterial];

	for (DWORD i = 0; i<pAppMesh->dwNumMaterial; i++)
	{
		//�A���r�G���g
		pAppMesh->pMaterial[i].Ambient.x = pFrame->pMeshContainer->pMaterials[i].MatD3D.Ambient.r;
		pAppMesh->pMaterial[i].Ambient.y = pFrame->pMeshContainer->pMaterials[i].MatD3D.Ambient.g;
		pAppMesh->pMaterial[i].Ambient.z = pFrame->pMeshContainer->pMaterials[i].MatD3D.Ambient.b;
		pAppMesh->pMaterial[i].Ambient.w = pFrame->pMeshContainer->pMaterials[i].MatD3D.Ambient.a;
		//�f�B�t���[�Y
		pAppMesh->pMaterial[i].Diffuse.x = pFrame->pMeshContainer->pMaterials[i].MatD3D.Diffuse.r;
		pAppMesh->pMaterial[i].Diffuse.y = pFrame->pMeshContainer->pMaterials[i].MatD3D.Diffuse.g;
		pAppMesh->pMaterial[i].Diffuse.z = pFrame->pMeshContainer->pMaterials[i].MatD3D.Diffuse.b;
		pAppMesh->pMaterial[i].Diffuse.w = pFrame->pMeshContainer->pMaterials[i].MatD3D.Diffuse.a;
		//�X�y�L�����[
		pAppMesh->pMaterial[i].Specular.x = pFrame->pMeshContainer->pMaterials[i].MatD3D.Specular.r;
		pAppMesh->pMaterial[i].Specular.y = pFrame->pMeshContainer->pMaterials[i].MatD3D.Specular.g;
		pAppMesh->pMaterial[i].Specular.z = pFrame->pMeshContainer->pMaterials[i].MatD3D.Specular.b;
		pAppMesh->pMaterial[i].Specular.w = pFrame->pMeshContainer->pMaterials[i].MatD3D.Specular.a;
		//�e�N�X�`���[�������
		if (pFrame->pMeshContainer->pMaterials[i].pTextureFilename != NULL &&
			lstrlenA(pFrame->pMeshContainer->pMaterials[i].pTextureFilename) > 0)
		{
			pAppMesh->Tex = true;
			strcpy(pAppMesh->pMaterial[i].szTextureName, pFrame->pMeshContainer->pMaterials[i].pTextureFilename);
			//�e�N�X�`���[���쐬
			if (FAILED(D3DX11CreateShaderResourceViewFromFileA(m_pDevice11,
				pAppMesh->pMaterial[i].szTextureName, NULL, NULL, &pAppMesh->pMaterial[i].pTexture, NULL)))
			{
				return E_FAIL;
			}
		}
		//�C���f�b�N�X�o�b�t�@�[���쐬

		//����ɐ旧���A���b�V���̑������𓾂�B�������ŃC���f�b�N�X�o�b�t�@�[����ׂ����}�e���A�����Ƃ̃C���f�b�N�X�o�b�t�@�𕪗��ł���
		D3DXATTRIBUTERANGE* pAttrTable = NULL;
		DWORD NumAttr = 0;
		pD3DXMesh->GetAttributeTable(pAttrTable, &NumAttr);
		pAttrTable = new D3DXATTRIBUTERANGE[NumAttr];
		if (FAILED(pD3DXMesh->GetAttributeTable(pAttrTable, &NumAttr)))
		{
			MessageBoxA(0, "�����e�[�u���擾���s", "", MB_OK);
			return E_FAIL;
		}

		//D3DXMESH�̏ꍇ�A���b�N���Ȃ���D3DX�C���f�b�N�X�o�b�t�@�[������o���Ȃ��̂Ń��b�N����B

		//2�o�C�g�̃C���f�b�N�X�̏ꍇ�����肦��E�E�E
		DWORD* pIndex = (DWORD*)new DWORD[pD3DXMesh->GetNumFaces() * 3];

		LPDIRECT3DINDEXBUFFER9 pIB;
		pD3DXMesh->GetIndexBuffer(&pIB);
		D3DINDEXBUFFER_DESC desc;
		pIB->GetDesc(&desc);

		if (desc.Format == D3DFMT_INDEX16)
		{
			WORD *wordIndex = NULL;
			pD3DXMesh->LockIndexBuffer(D3DLOCK_READONLY, (void**)&wordIndex);
			for (int i = 0; i<pD3DXMesh->GetNumFaces() * 3; i++)
			{
				pIndex[i] = wordIndex[i];
			}
		}
		else if (desc.Format == D3DFMT_INDEX32)
		{
			DWORD *dwordIndex = NULL;
			pD3DXMesh->LockIndexBuffer(D3DLOCK_READONLY, (void**)&dwordIndex);
			memcpy(pIndex, dwordIndex, pD3DXMesh->GetNumFaces() * 3);
		}

		//�}�e���A�����Ƃ̃C���f�b�N�X�o�b�t�@���쐬
		for (DWORD i = 0; i<NumAttr; i++)
		{
			//Dx9��D3DMESH�̃C���f�b�N�X�o�b�t�@�[����̏��ŁADx11�̃C���f�b�N�X�o�b�t�@���쐬
			bd.Usage = D3D11_USAGE_DEFAULT;
			bd.ByteWidth = sizeof(DWORD) * pAttrTable[i].FaceCount * 3;
			bd.BindFlags = D3D11_BIND_INDEX_BUFFER;
			bd.CPUAccessFlags = 0;
			bd.MiscFlags = 0;
			InitData.pSysMem = &pIndex[pAttrTable[i].FaceStart * 3];//�傫���C���f�b�N�X�o�b�t�@���̃I�t�Z�b�g
			InitData.SysMemPitch = 0;
			InitData.SysMemSlicePitch = 0;
			if (FAILED(m_pDevice11->CreateBuffer(&bd, &InitData, &pAppMesh->ppIndexBuffer[i])))
			{
				return FALSE;
			}
			pAppMesh->pMaterial[i].dwNumFace = pAttrTable[i].FaceCount;
		}


		delete[] pAttrTable;
		SAFE_RELEASE(pIB);
		pD3DXMesh->UnlockIndexBuffer();
		delete pIndex;
	}
	//�o�[�e�b�N�X�o�b�t�@�[���쐬

	//D3DXMESH�̏ꍇ�A���b�N���Ȃ���D3DX�o�[�e�b�N�X�o�b�t�@�[������o���Ȃ��̂Ń��b�N����B
	LPDIRECT3DVERTEXBUFFER9 pVB = NULL;
	pD3DXMesh->GetVertexBuffer(&pVB);
	DWORD dwStride = pD3DXMesh->GetNumBytesPerVertex();
	BYTE *pVertices = NULL;
	MY_VERTEX_NOTEX* pvVertex = NULL;
	MY_VERTEX_TEX* pvVertexTex = NULL;
	if (SUCCEEDED(pVB->Lock(0, 0, (VOID**)&pVertices, 0)))
	{
		//Dx9��D3DMESH�̃o�[�e�b�N�X�o�b�t�@�[����̏��ŁADx11�̃A�v����`�o�[�e�b�N�X�o�b�t�@���쐬
		bd.Usage = D3D11_USAGE_DEFAULT;
		bd.BindFlags = D3D11_BIND_VERTEX_BUFFER;
		bd.CPUAccessFlags = 0;
		bd.MiscFlags = 0;

		if (!pAppMesh->Tex)
		{
			pvVertex = (MY_VERTEX_NOTEX*)pVertices;
			bd.ByteWidth = sizeof(MY_VERTEX_NOTEX) *pD3DXMesh->GetNumVertices();
			InitData.pSysMem = pvVertex;
		}
		else
		{
			pvVertexTex = (MY_VERTEX_TEX*)pVertices;
			bd.ByteWidth = sizeof(MY_VERTEX_TEX) *pD3DXMesh->GetNumVertices();
			InitData.pSysMem = pvVertexTex;
		}
		if (FAILED(m_pDevice11->CreateBuffer(&bd, &InitData, &pAppMesh->pVertexBuffer)))
			return FALSE;

		pVB->Unlock();
	}
	SAFE_RELEASE(pVB);

	pFrame->pPartsMesh = pAppMesh;

	return S_OK;
}
//
//
//
HRESULT CD3DXMESH_ANIM::InitShader()
{
	//hlsl�t�@�C���ǂݍ��� �u���u�쐬�@�u���u�Ƃ̓V�F�[�_�[�̉�݂����Ȃ��́BXX�V�F�[�_�[�Ƃ��ē����������Ȃ��B��Ŋe��V�F�[�_�[�ɐ��蓾��B
	ID3D10Blob *pCompiledShader = NULL;
	ID3D10Blob *pErrors = NULL;
	//�u���u����o�[�e�b�N�X�V�F�[�_�[�쐬
	if (FAILED(D3DX11CompileFromFile(L"Mesh.hlsl", NULL, NULL, "VS", "vs_5_0", 0, 0, NULL, &pCompiledShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl�ǂݍ��ݎ��s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);

	if (FAILED(m_pDevice11->CreateVertexShader(pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), NULL, &m_pVertexShader)))
	{
		SAFE_RELEASE(pCompiledShader);
		MessageBox(0, L"�o�[�e�b�N�X�V�F�[�_�[�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	//���_�C���v�b�g���C�A�E�g���`	
	D3D11_INPUT_ELEMENT_DESC layout[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "TEXCOORD", 0, DXGI_FORMAT_R32G32_FLOAT, 0, 24, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	UINT numElements = sizeof(layout) / sizeof(layout[0]);
	//���_�C���v�b�g���C�A�E�g���쐬
	if (FAILED(m_pDevice11->CreateInputLayout(layout, numElements, pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), &m_pVertexLayout)))
	{
		return FALSE;
	}
	//�e�N�X�`���[�Ȃ��V�F�[�_�[
	SAFE_RELEASE(pErrors);
	SAFE_RELEASE(pCompiledShader);
	if (FAILED(D3DX11CompileFromFile(L"Mesh.hlsl", NULL, NULL, "VS_NoTeX", "vs_5_0", 0, 0, NULL, &pCompiledShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl�ǂݍ��ݎ��s", NULL, MB_OK);
		return E_FAIL;
	}
	if (FAILED(m_pDevice11->CreateVertexShader(pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), NULL, &m_pVertexShaderNoTex)))
	{
		SAFE_RELEASE(pCompiledShader);
		MessageBox(0, L"�o�[�e�b�N�X�V�F�[�_�[�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	//�e�N�X�`���[�Ȃ�
	D3D11_INPUT_ELEMENT_DESC layout2[] =
	{
		{ "POSITION", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 0, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	{ "NORMAL", 0, DXGI_FORMAT_R32G32B32_FLOAT, 0, 12, D3D11_INPUT_PER_VERTEX_DATA, 0 },
	};
	numElements = sizeof(layout2) / sizeof(layout2[0]);
	//���_�C���v�b�g���C�A�E�g���쐬
	if (FAILED(m_pDevice11->CreateInputLayout(layout2, numElements, pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), &m_pVertexLayout_NoTex)))
	{
		return FALSE;
	}
	//�u���u����s�N�Z���V�F�[�_�[�쐬
	if (FAILED(D3DX11CompileFromFile(L"Mesh.hlsl", NULL, NULL, "PS", "ps_5_0", 0, 0, NULL, &pCompiledShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl�ǂݍ��ݎ��s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	if (FAILED(m_pDevice11->CreatePixelShader(pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), NULL, &m_pPixelShader)))
	{
		SAFE_RELEASE(pCompiledShader);
		MessageBox(0, L"�s�N�Z���V�F�[�_�[�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pCompiledShader);

	//�e�N�X�`���[�Ȃ�
	if (FAILED(D3DX11CompileFromFile(L"Mesh.hlsl", NULL, NULL, "PS_NoTex", "ps_5_0", 0, 0, NULL, &pCompiledShader, &pErrors, NULL)))
	{
		MessageBox(0, L"hlsl�ǂݍ��ݎ��s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pErrors);
	if (FAILED(m_pDevice11->CreatePixelShader(pCompiledShader->GetBufferPointer(), pCompiledShader->GetBufferSize(), NULL, &m_pPixelShaderNoTex)))
	{
		SAFE_RELEASE(pCompiledShader);
		MessageBox(0, L"�s�N�Z���V�F�[�_�[�쐬���s", NULL, MB_OK);
		return E_FAIL;
	}
	SAFE_RELEASE(pCompiledShader);
	//�R���X�^���g�o�b�t�@�[�쐬�@�ϊ��s��n���p
	D3D11_BUFFER_DESC cb;
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(BUFFER_PER_MESH);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;

	if (FAILED(m_pDevice11->CreateBuffer(&cb, NULL, &m_pConstantBuffer0)))
	{
		return E_FAIL;
	}
	//�R���X�^���g�o�b�t�@�[�쐬  �}�e���A���n���p
	cb.BindFlags = D3D11_BIND_CONSTANT_BUFFER;
	cb.ByteWidth = sizeof(BUFFER_PER_MATERIAL);
	cb.CPUAccessFlags = D3D11_CPU_ACCESS_WRITE;
	cb.MiscFlags = 0;
	cb.Usage = D3D11_USAGE_DYNAMIC;

	if (FAILED(m_pDevice11->CreateBuffer(&cb, NULL, &m_pConstantBuffer1)))
	{
		return E_FAIL;
	}
	return S_OK;
}
//
//
//
void CD3DXMESH_ANIM::UpdateHierarchyMatrices(D3DXFRAME* p, LPD3DXMATRIX pParentMatrix)
{
	MYHFRAME *pFrame = (MYHFRAME*)p;

	if (pParentMatrix != NULL)
	{
		pFrame->CombinedTransformationMatrix = pFrame->TransformationMatrix**pParentMatrix;
	}
	else
	{
		pFrame->CombinedTransformationMatrix = pFrame->TransformationMatrix;
	}

	if (pFrame->pFrameSibling != NULL)
	{
		UpdateHierarchyMatrices(pFrame->pFrameSibling, pParentMatrix);
	}

	if (pFrame->pFrameFirstChild != NULL)
	{
		UpdateHierarchyMatrices(pFrame->pFrameFirstChild, &pFrame->CombinedTransformationMatrix);
	}
}
//
//
//
void CD3DXMESH_ANIM::Render(D3DXMATRIX& mView, D3DXMATRIX& mProj, D3DXVECTOR3& vLight, D3DXVECTOR3& vEye)
{
	m_View = mView;
	m_Proj = mProj;
	m_LightDir = vLight;
	m_Eye = vEye;
	//�v���~�e�B�u�E�g�|���W�[���Z�b�g
	m_pDeviceContext11->IASetPrimitiveTopology(D3D11_PRIMITIVE_TOPOLOGY_TRIANGLELIST);

	D3DXMATRIX World;
	D3DXMatrixRotationY(&World, m_fYaw);
	UpdateHierarchyMatrices(m_pFrameRoot, &World);
	DrawFrame(m_pFrameRoot);
}
//
//
//
void CD3DXMESH_ANIM::DrawFrame(LPD3DXFRAME p)
{
	MYHFRAME* pFrame = (MYHFRAME*)p;
	PARTS_MESH* pPartsMesh = pFrame->pPartsMesh;
	if (pPartsMesh != NULL)
	{
		DrawPartsMesh(pPartsMesh, pFrame->CombinedTransformationMatrix);
	}

	if (pFrame->pFrameSibling != NULL)
	{
		DrawFrame(pFrame->pFrameSibling);
	}

	if (pFrame->pFrameFirstChild != NULL)
	{
		DrawFrame(pFrame->pFrameFirstChild);
	}
}
//
//
//
void CD3DXMESH_ANIM::DrawPartsMesh(PARTS_MESH* pPartsMesh, D3DXMATRIX World)
{
	D3DXMATRIX mWorld, mTran, mYaw, mPitch, mRoll, mScale;
	//���[���h�g�����X�t�H�[���i��΍��W�ϊ��j
	mWorld = World;
	//�V�F�[�_�[�̃R���X�^���g�o�b�t�@�[�Ɋe��f�[�^��n��
	D3D11_MAPPED_SUBRESOURCE pData;
	if (SUCCEEDED(m_pDeviceContext11->Map(m_pConstantBuffer0, 0, D3D11_MAP_WRITE_DISCARD, 0, &pData)))
	{
		BUFFER_PER_MESH sg;
		//���[���h�s���n��
		sg.mW = mWorld;
		D3DXMatrixTranspose(&sg.mW, &sg.mW);
		//���[���h�A�J�����A�ˉe�s���n��
		sg.mWVP = mWorld * m_View*m_Proj;
		D3DXMatrixTranspose(&sg.mWVP, &sg.mWVP);
		//���C�g�̕�����n��
		sg.vLightDir = D3DXVECTOR4(m_LightDir.x, m_LightDir.y, m_LightDir.z, 0.0f);
		//���_�ʒu��n��
		sg.vEye = D3DXVECTOR4(m_Eye.x, m_Eye.y, m_Eye.z, 0);

		memcpy_s(pData.pData, pData.RowPitch, (void*)&sg, sizeof(BUFFER_PER_MESH));
		m_pDeviceContext11->Unmap(m_pConstantBuffer0, 0);
	}
	//���̃R���X�^���g�o�b�t�@�[���g���V�F�[�_�[�̓o�^
	m_pDeviceContext11->VSSetConstantBuffers(0, 1, &m_pConstantBuffer0);
	m_pDeviceContext11->PSSetConstantBuffers(0, 1, &m_pConstantBuffer0);

	//�}�e���A�����ƂɃ����_�����O
	UINT stride = 0;
	if (pPartsMesh->Tex)
	{
		stride = sizeof(MY_VERTEX_TEX);
		//���_�C���v�b�g���C�A�E�g���Z�b�g
		m_pDeviceContext11->IASetInputLayout(m_pVertexLayout);
		//�g�p����V�F�[�_�[�̓o�^
		m_pDeviceContext11->VSSetShader(m_pVertexShader, NULL, 0);
		m_pDeviceContext11->PSSetShader(m_pPixelShader, NULL, 0);
	}
	else
	{
		stride = sizeof(MY_VERTEX_NOTEX);
		//���_�C���v�b�g���C�A�E�g���Z�b�g
		m_pDeviceContext11->IASetInputLayout(m_pVertexLayout_NoTex);
		//�g�p����V�F�[�_�[�̓o�^
		m_pDeviceContext11->VSSetShader(m_pVertexShaderNoTex, NULL, 0);
		m_pDeviceContext11->PSSetShader(m_pPixelShaderNoTex, NULL, 0);
	}
	//�o�[�e�b�N�X�o�b�t�@�[���Z�b�g
	UINT offset = 0;
	m_pDeviceContext11->IASetVertexBuffers(0, 1, &pPartsMesh->pVertexBuffer, &stride, &offset);

	//�}�e���A���̐������A���ꂼ��̃}�e���A���̃C���f�b�N�X�o�b�t�@�|��`��
	for (DWORD i = 0; i<pPartsMesh->dwNumMaterial; i++)
	{
		//�g�p����Ă��Ȃ��}�e���A���΍�
		if (pPartsMesh->pMaterial[i].dwNumFace == 0)
		{
			continue;
		}
		//�C���f�b�N�X�o�b�t�@�[���Z�b�g
		m_pDeviceContext11->IASetIndexBuffer(pPartsMesh->ppIndexBuffer[i], DXGI_FORMAT_R32_UINT, 0);
		//�}�e���A���̊e�v�f���G�t�F�N�g�i�V�F�[�_�[�j�ɓn��
		D3D11_MAPPED_SUBRESOURCE pData;
		if (SUCCEEDED(m_pDeviceContext11->Map(m_pConstantBuffer1, 0, D3D11_MAP_WRITE_DISCARD, 0, &pData)))
		{
			BUFFER_PER_MATERIAL sg;
			sg.vAmbient = pPartsMesh->pMaterial[i].Ambient;//�A���r�G���g���V�F�[�_�[�ɓn��
			sg.vDiffuse = pPartsMesh->pMaterial[i].Diffuse;//�f�B�t���[�Y�J���[���V�F�[�_�[�ɓn��
			sg.vSpecular = pPartsMesh->pMaterial[i].Specular;//�X�y�L�����[���V�F�[�_�[�ɓn��
			memcpy_s(pData.pData, pData.RowPitch, (void*)&sg, sizeof(BUFFER_PER_MATERIAL));
			m_pDeviceContext11->Unmap(m_pConstantBuffer1, 0);
		}
		m_pDeviceContext11->VSSetConstantBuffers(1, 1, &m_pConstantBuffer1);
		m_pDeviceContext11->PSSetConstantBuffers(1, 1, &m_pConstantBuffer1);
		//�e�N�X�`���[���V�F�[�_�[�ɓn��
		if (pPartsMesh->pMaterial[i].szTextureName[0] != NULL)
		{
			m_pDeviceContext11->PSSetSamplers(0, 1, &m_pSampleLinear);
			m_pDeviceContext11->PSSetShaderResources(0, 1, &pPartsMesh->pMaterial[i].pTexture);
		}
		else
		{
			ID3D11ShaderResourceView* Nothing[1] = { 0 };
			m_pDeviceContext11->PSSetShaderResources(0, 1, Nothing);
		}
		//�v���~�e�B�u�������_�����O
		m_pDeviceContext11->DrawIndexed(pPartsMesh->pMaterial[i].dwNumFace * 3, 0, 0);
	}
}