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

/* �_�Q�̍��W�ϊ��F�_�Q�̏d�S�����_���W�ɐݒ� */
int CoordinateTransformation(
	struct PointCloudInfo*,						// ���f���_�Q���
	struct PointCloudInfo*,						// �T���Ώۓ_�Q���
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// ���W�ϊ��O�̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr,		// ���W�ϊ��O�̒T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr,	// ���W�ϊ���̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr	// ���W�ϊ���̒T���Ώۓ_�Q
);

/* �ʏ�̉�]�s�� R ���� Rodrigues �̉�]�\�� r = [ r1, r2, r3 ] �ւ̕ϊ� */
Eigen::Vector3d ConvertRodrigues(
	Eigen::Matrix< double, 3, 3 >*	// Rodrigues �ϊ��O�� 3 * 3 ��]�s��
);

/* Rodrigues �̉�]�s��̕��� */
Eigen::Matrix< double, 3, 3 > ReconstructRFromRodrigues(
	Eigen::Vector3d*	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
);

/* ��]�s��� r1, r2, r3 �����ɂ��Δ��� */
void PartialDerivativeForRodrigues(
	Eigen::Vector3d*,	// Rodrigues ��p������]�\�� r = [ r1, r2, r3 ]
	Eigen::Matrix3d* dR	// 
);
