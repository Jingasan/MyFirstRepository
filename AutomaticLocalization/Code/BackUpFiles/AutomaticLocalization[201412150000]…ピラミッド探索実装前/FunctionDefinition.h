/***** �֐��A�\���̂̒�` *****/


/*** �C���N���[�h�t�@�C�� ***/

/* C */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

/* C++ */
#include <iostream>
#include <fstream>
#include <vector>

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/io.h>
#include <pcl/point_types.h>

/* OpenMP */
#include <omp.h>


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** �\���̂̒�` ***/

/* 3�����_�Q�i�[�p */
struct ModelPointCloud{
	double Xmodel;	// �_�Q���f�����W�n�� X
	double Ymodel;	// �_�Q���f�����W�n�� Y
	double Zmodel;	// �_�Q���f�����W�n�� Z
};

/* 3�����_�Q�̊e����i�[�p */
struct ModelInformation{
     double Xmin;	// �_�Q�� X ���W�̍ŏ��l
     double Xmax;	// �_�Q�� X ���W�̍ő�l
     double Ymin;	// �_�Q�� Y ���W�̍ŏ��l
     double Ymax;	// �_�Q�� Y ���W�̍ő�l
     double Zmin;	// �_�Q�� Z ���W�̍ŏ��l
     double Zmax;	// �_�Q�� Z ���W�̍ő�l
     double deltaX;	// �_�Q�� X ���̕�
     double deltaY;	// �_�Q�� Y ���̕�
     double deltaZ;	// �_�Q�� Z ���̕�
     double max;	// �_�Q�̍ő啝
	 double gravX;	// �_�Q�̏d�S X
	 double gravY;	// �_�Q�̏d�S Y
	 double gravZ;	// �_�Q�̏d�S Z
     int iSize;		// DF �� i ���̕�
     int jSize;		// DF �� j ���̕�
     int kSize;		// DF �� k ���̕�
};



/*** �֐��̒�` ***/

/* DF �쐬�֐� */
void CreateDistanceFieldforPCDModel(
	int,									// DF �̕�����( DF �̃{�N�Z���̕���\ )
	int**,									// DF
	int**,									// �ŋߓ_�� ID
	int*,									// �T���Ώۓ_�Q��
	struct ModelPointCloud**,				// �T���Ώۓ_�Q( DF �̍��W�n�ɒ���������, �P��:mm )
	struct ModelInformation*,				// �T���Ώۓ_�Q�̊e����
	pcl::PointCloud< pcl::PointXYZ >::Ptr	// �T���Ώۓ_�Q( �P��:m )
);

/* DF �������֐� */
void BinarizationDFforPCDModel(
	int,									// DF �̕�����( DF �̃{�N�Z���̕���\ )
	int*,									// DF �쐬�̂��߂Ɉꎞ�I�ɕۑ����锠
	int**,									// �ŋߓ_ ID
	struct ModelPointCloud**,				// �_�Q( DF �̍��W�n�ɒ���������, �P��:mm )
	int*,									// �_�Q��
	struct ModelInformation*,				// �_�Q�̊e����
	pcl::PointCloud< pcl::PointXYZ >::Ptr,	// ���͓_�Q
	double,									// �_�Q�� X �����̏d�S
	double,									// �_�Q�� Y �����̏d�S
	double									// �_�Q�� Z �����̏d�S
);

/* �S�T�� */
int ExhaustiveSearch(
	int,										// DF �̕�����( DF �̃{�N�Z���̕���\ )
	int**,										// DF
	struct ModelInformation*,					// �T���Ώۓ_�Q�̊e����
	struct ModelInformation*,					// ���f���_�Q�̊e����
	int*,										// �T���Ώۓ_�Q�ƃ��f���_�Q�Ƃ̋����]�����v�l�̍ŏ��l
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// �T���Ώۓ_�Q
	vector< Eigen::Matrix< double, 3, 3 > >*,	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d >*,					// �S�T�����ʂ̕��i�x�N�g��
	vector< double >*							// �S�T�����ʂ� DF �l
	//Eigen::Matrix< double, 3, 3 >**			// ��]�s�� R
);



/* 2�_�Q�̓����Ɖ��� */
int mergedPointDataViewerRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char*, const char* );	// PointXYZRGB
int mergedPointDataViewer( pcl::PointCloud< pcl::PointXYZ >::Ptr, pcl::PointCloud< pcl::PointXYZ >::Ptr, const char*, const char* );			// PointXYZ

/* �_�Q�̉��� */
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr );	// PointXYZRGB
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr );		// PointXYZ

/* �_�Q�̏o�͕ۑ� */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )

/* ICP �A���S���Y����p�����ʒu���킹 */
int ICPOptimizationRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// �œK����̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// �œK����̃��f���_�Q�ƒT���Ώۓ_�Q�̓�������
	struct ModelInformation*,					// �T���Ώۓ_�Q�̊e����
	struct ModelInformation*,					// ���f���_�Q�̊e����
	vector< Eigen::Matrix< double, 3, 3 > >*,	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d >*					// �S�T�����ʂ̕��i�x�N�g��
);
int ICPOptimization(
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// �T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,		// �œK����̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,		// �œK����̃��f���_�Q�ƒT���Ώۓ_�Q�̓�������
	struct ModelInformation*,					// �T���Ώۓ_�Q�̊e����
	struct ModelInformation*,					// ���f���_�Q�̊e����
	vector< Eigen::Matrix< double, 3, 3 > >*,	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d >*					// �S�T�����ʂ̕��i�x�N�g��
);

/* �}�[�J�[�g�@��p�����ʒu���킹 */
int LevMarOptimization(
	int**,										// DF
	int**,										// �ŋߓ_ ID
	struct ModelInformation*,					// �T���Ώۓ_�Q�̊e����
	struct ModelInformation*,					// ���f���_�Q�̊e����
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// ���f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// �T���Ώۓ_�Q
	vector< Eigen::Matrix< double, 3, 3 > >*,	// �S�T�����ʂ̉�]�s��
	vector< Eigen::Vector3d >*,					// �S�T�����ʂ̕��i�x�N�g��
	vector< double >*							// �S�T�����ʂ� DF �l
);

/* �ʏ�̉�]�s�� R ���� Rodrigues �̉�]�\�� r = [ r1, r2, r3 ] �ւ̕ϊ� */
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >*	// Rodrigues �ϊ��O�� 3 * 3 ��]�s��
);

/* ��]�s��� r1, r2, r3 �����ɂ��Δ��� */
Eigen::Matrix< double, 3, 3 > PartialDerivativeForRodrigues(
	int,				// �Δ��������]�����ԍ� �� 0�Fr1, 1�Fr2, 2�Fr3
	Eigen::Vector3d*	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
);