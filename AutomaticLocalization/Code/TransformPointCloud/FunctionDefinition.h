/***** �֐��̒�`�p�w�b�_�t�@�C�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// �_�Q�̃f�[�^�^


/*** �\���̂̒�` ***/

/* 3�����_�Q�̊e����i�[�p */
struct PointCloudInfo{
     double Xmin;	// �_�Q�� X ���W�̍ŏ��l
     double Xmax;	// �_�Q�� X ���W�̍ő�l
     double Ymin;	// �_�Q�� Y ���W�̍ŏ��l
     double Ymax;	// �_�Q�� Y ���W�̍ő�l
     double Zmin;	// �_�Q�� Z ���W�̍ŏ��l
     double Zmax;	// �_�Q�� Z ���W�̍ő�l
	 double gravX;	// �_�Q�̏d�S X
	 double gravY;	// �_�Q�̏d�S Y
	 double gravZ;	// �_�Q�̏d�S Z
};


/*** �֐��̒�` ***/

/* 3�����_�Q�̕\�� */
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, int );	// PointXYZRGB
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, int );		// PointXYZ

/* 3�����_�Q�̏o�͕ۑ� */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
