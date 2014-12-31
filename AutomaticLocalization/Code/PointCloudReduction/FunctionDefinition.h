/***** �֐��̒�`�p�w�b�_�t�@�C�� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// �_�Q�̃f�[�^�^


/*** �֐��̒�` ***/

/* 3�����_�Q�̓ǂݍ��� */
int loadPCDPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int loadPCDPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );		// PCD( PointXYZ )
int loadPLYPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int loadPLYPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );		// PLY( PointXYZ )

/* 3�����G�b�W�_�Q���̍팸 */
int reductPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, pcl::PointCloud< pcl::PointXYZRGB >::Ptr, float, float, float );	// PointXYZRGB
int reductPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr, pcl::PointCloud< pcl::PointXYZ >::Ptr, float, float, float );			// PointXYZ

/* 3�����_�Q�̏o�͕ۑ� */
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PCD( PointXYZRGB )
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PCD( PointXYZ )
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr, const char* );	// PLY( PointXYZRGB )
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr, const char* );			// PLY( PointXYZ )
