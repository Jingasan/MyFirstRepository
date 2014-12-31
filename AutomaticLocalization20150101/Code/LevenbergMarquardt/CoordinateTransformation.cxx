/***** �_�Q�̍��W�ϊ� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �_�Q�̏d�S�����_���W�ɐݒ� ***/
int CoordinateTransformation(
	struct PointCloudInfo* modelInformation,					// ���f���_�Q���
	struct PointCloudInfo* targetInformation,					// �T���Ώۓ_�Q���
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudM,				// ���W�ϊ��O�̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloudT,				// ���W�ϊ��O�̒T���Ώۓ_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr modelPointCloud,	// ���W�ϊ���̃��f���_�Q
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr targetPointCloud	// ���W�ϊ���̒T���Ώۓ_�Q
){

	/* �ϐ��̒�` */
	pcl::PointXYZRGB temp;	// �_���W�̈ꎞ�i�[�p�ϐ�

	/* ������ */
	modelInformation->Xmin = DBL_MAX;
	modelInformation->Ymin = DBL_MAX;
	modelInformation->Zmin = DBL_MAX;
	modelInformation->Xmax = -DBL_MAX;
	modelInformation->Ymax = -DBL_MAX;
	modelInformation->Zmax = -DBL_MAX;
	targetInformation->Xmin = DBL_MAX;
	targetInformation->Ymin = DBL_MAX;
	targetInformation->Zmin = DBL_MAX;
	targetInformation->Xmax = -DBL_MAX;
	targetInformation->Ymax = -DBL_MAX;
	targetInformation->Zmax = -DBL_MAX;

	/* ���f���_�Q�̒[�_�̎Z�o */
	for( int i = 0; i < cloudM->points.size(); i++ ){
		
		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloudM->points[ i ].x < modelInformation->Xmin ) modelInformation->Xmin = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y < modelInformation->Ymin ) modelInformation->Ymin = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z < modelInformation->Zmin ) modelInformation->Zmin = cloudM->points[ i ].z;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloudM->points[ i ].x > modelInformation->Xmax ) modelInformation->Xmax = cloudM->points[ i ].x;
		if( cloudM->points[ i ].y > modelInformation->Ymax ) modelInformation->Ymax = cloudM->points[ i ].y;
		if( cloudM->points[ i ].z > modelInformation->Zmax ) modelInformation->Zmax = cloudM->points[ i ].z;
	}

	/* �T���Ώۓ_�Q�̒[�_�̎Z�o */
	for( int i = 0; i < cloudT->points.size(); i++ ){
		
		/* X, Y, Z �̍ŏ��l��ݒ� */
		if( cloudT->points[ i ].x < targetInformation->Xmin ) targetInformation->Xmin = cloudT->points[ i ].x;
		if( cloudT->points[ i ].y < targetInformation->Ymin ) targetInformation->Ymin = cloudT->points[ i ].y;
		if( cloudT->points[ i ].z < targetInformation->Zmin ) targetInformation->Zmin = cloudT->points[ i ].z;
		
		/* X, Y, Z �̍ő�l��ݒ� */
		if( cloudT->points[ i ].x > targetInformation->Xmax ) targetInformation->Xmax = cloudT->points[ i ].x;
		if( cloudT->points[ i ].y > targetInformation->Ymax ) targetInformation->Ymax = cloudT->points[ i ].y;
		if( cloudT->points[ i ].z > targetInformation->Zmax ) targetInformation->Zmax = cloudT->points[ i ].z;
	}
	
	/* �_�Q�̏d�S�����f�����Ƃ��Ċi�[ */
	modelInformation->gravX = ( modelInformation->Xmax + modelInformation->Xmin ) / 2;
	modelInformation->gravY = ( modelInformation->Ymax + modelInformation->Ymin ) / 2;
	modelInformation->gravZ = ( modelInformation->Zmax + modelInformation->Zmin ) / 2;
	targetInformation->gravX = ( targetInformation->Xmax + targetInformation->Xmin ) / 2;
	targetInformation->gravY = ( targetInformation->Ymax + targetInformation->Ymin ) / 2;
	targetInformation->gravZ = ( targetInformation->Zmax + targetInformation->Zmin ) / 2;

	/* �_�Q�̏d�S�����_�ɐݒ� */
	modelInformation->Xmin -= modelInformation->gravX; modelInformation->Xmax -= modelInformation->gravX;
	modelInformation->Ymin -= modelInformation->gravY; modelInformation->Ymax -= modelInformation->gravY;
	modelInformation->Zmin -= modelInformation->gravZ; modelInformation->Zmax -= modelInformation->gravZ;
	targetInformation->Xmin -= targetInformation->gravX; targetInformation->Xmax -= targetInformation->gravX;
	targetInformation->Ymin -= targetInformation->gravY; targetInformation->Ymax -= targetInformation->gravY;
	targetInformation->Zmin -= targetInformation->gravZ; targetInformation->Zmax -= targetInformation->gravZ;


	/* ���f���_�Q */
	for( int i = 0; i < cloudM->size(); i++ ){
			
		/* ���f���_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = cloudM->points[ i ].x - modelInformation->gravX;
		temp.y = cloudM->points[ i ].y - modelInformation->gravY;
		temp.z = cloudM->points[ i ].z - modelInformation->gravZ;
		temp.r = 255;
		temp.g = 0;
		temp.b = 0;
		modelPointCloud->points.push_back( temp );
	}
	modelPointCloud->width = cloudM->points.size();
	modelPointCloud->height = 1;

	/* �T���Ώۓ_�Q */
	for( int i = 0; i < cloudT->size(); i++ ){
			
		/* �T���Ώۓ_�Q�� DF ���W�n�֕ϊ��FDF ���W�n�͓_�Q�̏d�S�����_ */
		temp.x = cloudT->points[ i ].x - targetInformation->gravX;
		temp.y = cloudT->points[ i ].y - targetInformation->gravY;
		temp.z = cloudT->points[ i ].z - targetInformation->gravZ;
		temp.r = 0;
		temp.g = 255;
		temp.b = 0;
		targetPointCloud->points.push_back( temp );
	}
	targetPointCloud->width = targetPointCloud->points.size();
	targetPointCloud->height = 1;



	return 0;
}