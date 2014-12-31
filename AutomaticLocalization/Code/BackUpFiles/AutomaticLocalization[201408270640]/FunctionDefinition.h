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
#include <pcl/visualization/cloud_viewer.h>	// �_�Q�̉���

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
	int,
	int **,
	int **,
	int *,
	struct ModelPointCloud**,
	struct ModelInformation*,
	pcl::PointCloud< pcl::PointXYZ >::Ptr
);

/* DF �������֐� */
void BinarizationDFforPCDModel(
	int,
	int *,
	int **,
	struct ModelPointCloud**,
	int *,
	struct ModelInformation *,
	pcl::PointCloud< pcl::PointXYZ >::Ptr,
	double,
	double,
	double
);

/* �S�T�� */
int ExhaustiveSearch(
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
