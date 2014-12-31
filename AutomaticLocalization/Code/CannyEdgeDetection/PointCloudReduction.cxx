/***** 3�����_�Q���̍팸 *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>				// �_�Q�̃f�[�^�^
#include <pcl/filters/voxel_grid.h>			// �{�N�Z���O���b�h�ɂ��_�Q�̊Ԉ���

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** �_�Q���̍팸( PointXYZRGB ) ***/
int reductPointCloudRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr reCloud,
	float voxelGridSizeX,
	float voxelGridSizeY,
	float voxelGridSizeZ
){

	/* �{�N�Z���O���b�h�ɂ��_�E���T���v�����O */
	pcl::VoxelGrid< pcl::PointXYZRGB > bg;
	bg.setInputCloud( cloud );	// �팸�O��3�����_�Q
	bg.setLeafSize( voxelGridSizeX, voxelGridSizeY, voxelGridSizeZ );
	bg.filter( *reCloud );		// �팸���3�����_�Q

	return 0;
}


/*** �_�Q���̍팸( PointXYZ ) ***/
int reductPointCloud(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZ >::Ptr reCloud,
	float voxelGridSizeX,
	float voxelGridSizeY,
	float voxelGridSizeZ
){

	/* �{�N�Z���O���b�h�ɂ��_�E���T���v�����O */
	pcl::VoxelGrid< pcl::PointXYZ > bg;
	bg.setInputCloud( cloud );	// �팸�O��3�����_�Q
	bg.setLeafSize( voxelGridSizeX, voxelGridSizeY, voxelGridSizeZ );
	bg.filter( *reCloud );		// �팸���3�����_�Q

	return 0;
}