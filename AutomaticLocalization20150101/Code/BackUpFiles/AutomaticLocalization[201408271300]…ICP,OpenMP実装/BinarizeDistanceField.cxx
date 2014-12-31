/***** �f�B�X�^���X�t�B�[���h�̍쐬 *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �f�B�X�^���X�t�B�[���h�̓�l�� ***/
void BinarizationDFforPCDModel(
	int size,									// DF �̕�����( DF �̃{�N�Z���̕���\ )
	int *DFtemp,								// DF �쐬�̂��߂Ɉꎞ�I�ɕۑ����锠
	int **ClosestPointID,						// �ŋߓ_�� ID
	struct ModelPointCloud** ModelPointCloud,	// 3�����_�Q
	int *PointSize,								// 3�����_�Q�̐�
	struct ModelInformation *ModelInf,			// 3�����_�Q�̊e����
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,	// ����3�����_�Q
	double gravX,								// 3�����_�Q�� X �����̏d�S
	double gravY,								// 3�����_�Q�� Y �����̏d�S
	double gravZ								// 3�����_�Q�� Z �����̏d�S
){	// �����Ń{�N�Z���ƃp�b�`�Ƃ̋����� 0.5 �ȓ��̂Ƃ��ɋ����� 0 �Ƃ��� DF �ɕۑ�����

	int iSize, jSize, kSize;	// DF �̃T�C�Y
	int hiSize, hjSize, hkSize;	// DF �� 1/2 �̃T�C�Y
	int qiSize, qjSize, qkSize;	// DF �� 1/4 �̃T�C�Y
	int PCDPointSize;			// PCD �̃|�C���g�T�C�Y
	double boxel;				// 1 voxel �̃T�C�Y
	double mingravX, mingravY, mingravZ;	// min �Əd�S�̘a
	int ClosestPointIDnum = 0;				// ClosestPointID �̃i���o�[��ۑ�
	struct ModelPointCloud Surface;			// 3�����_�Q�̏��
	vector< struct ModelPointCloud > ModelPointCloudtemp;	// 3�����_�Q�̏����ꎞ�I�ɕۑ�
	

	/*** �������̂��߂ɁA�F�X�i�[���Ă��� ***/

	/* 3�����_�Q�� i, j, k �����̃T�C�Y */
	iSize = ModelInf->iSize;
	jSize = ModelInf->jSize;
	kSize = ModelInf->kSize;
	
	/* 3�����_�Q�� i, j, k ������ 1/2 �̃T�C�Y */
	hiSize = iSize / 2;
	hjSize = jSize / 2;
	hkSize = kSize / 2;
	
	/* 3�����_�Q�� i, j, k ������ 1/4 �̃T�C�Y */
	qiSize = iSize / 4;
	qjSize = jSize / 4;
	qkSize = kSize / 4;
	
	/* 1voexl �̃T�C�Y */
	boxel = 2 * ModelInf->max / DISTANCEFIELDSIZE;
	 
	/* �ŏ��l�̌v�Z */
	mingravX = ModelInf->Xmin + gravX;	// 3�����_�Q�� X �����̍ŏ��l�̌v�Z
	mingravY = ModelInf->Ymin + gravY;	// 3�����_�Q�� Y �����̍ŏ��l�̌v�Z
	mingravZ = ModelInf->Zmin + gravZ;	// 3�����_�Q�� Z �����̍ŏ��l�̌v�Z
	
	/* 3�����_�Q�̐� */
	PCDPointSize = (int)cloud->points.size();


	/*** �{�N�Z�����ɂ���_��T�����Ēl�� 0 �ɏ����� ***/
	cout << "3�����_�Q��ǂݍ��ݒ��c�c" << "\r" ;
	for( int pn = 0; pn < PCDPointSize; pn++ ){

		/* nan�̓_�͓ǂݔ�΂� */
		//if( !_isnan( cloud->points[ pn ].x ) ){
		//���f���͈̔�
		//�������덷���v���X���ĉ�??

			/* DF �̏d�S���甼���T�C�Y�܂ł̗̈��T�� */
			for( int i = 1; i <= hiSize + 1; i++ ){
				for( int j = 1; j <= hjSize + 1; j++ ){
					for( int k = 1; k <= hkSize + 1; k++ ){

						/* ���ɒl�� 0 �Ȃ�ȉ��ɓ���Ӗ��������̂ŃX���[ */
						if( DFtemp[ ( k + qkSize ) * iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] != 0 ){
							
							/* �ǂ̃{�N�Z���̒��ɂ��邩�̒��� */
							if( cloud->points[ pn ].x * TOMETER >= mingravX + ( i - 1 ) * boxel && cloud->points[ pn ].x * TOMETER < mingravX + i * boxel ){
								if( cloud->points[ pn ].y * TOMETER >= mingravY + ( j - 1 ) * boxel && cloud->points[ pn ].y * TOMETER < mingravY + j * boxel ){
									if( cloud->points[ pn ].z * TOMETER >= mingravZ + ( k - 1 ) * boxel && cloud->points[ pn ].z * TOMETER < mingravZ + k * boxel ){
										
										/* DF �� 0 �ɏ����� */
										DFtemp[ ( k + qkSize ) * iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] = 0;
										
										/* �ŋߓ_�ɂ��ď��� */
										( *ClosestPointID )[ ( k + qkSize ) *iSize * jSize + ( j + qjSize ) * iSize + ( i + qiSize ) ] = ClosestPointIDnum;
										ClosestPointIDnum++;
										
										/* ModelPointCloud ���i�[ */
										Surface.Xmodel = cloud->points[ pn ].x * TOMETER - gravX;
										Surface.Ymodel = cloud->points[ pn ].y * TOMETER - gravY;
										Surface.Zmodel = cloud->points[ pn ].z * TOMETER - gravZ;

										ModelPointCloudtemp.push_back( Surface );
										
										//�������������̃��[�v�͕K�v�Ȃ�
										goto LOOPEXIT;
									} // if z
								} // if y
							} // if x
						} // if not zero
					} // for k
				} // for j
			} // for i
		//} // nan
LOOPEXIT:; //�����K�v�Ȃ����玟�̃��[�v�ւƐi��
	} // pn
   
	/* �]�������_�̐� */
	*PointSize = ClosestPointIDnum;
	
	/* ���f���_�Q�̃t�@�C���Ɋi�[�E���f���̐F�����i�[ */
	( *ModelPointCloud ) = new struct ModelPointCloud[ ( *PointSize ) ];

	/* ���f���_�Q���i�[ */
	for( int i = 0; i < ClosestPointIDnum; i++ ){
		( *ModelPointCloud )[ i ].Xmodel = ModelPointCloudtemp[ i ].Xmodel;
		( *ModelPointCloud )[ i ].Ymodel = ModelPointCloudtemp[ i ].Ymodel;
		( *ModelPointCloud )[ i ].Zmodel = ModelPointCloudtemp[ i ].Zmodel;
	}
}


