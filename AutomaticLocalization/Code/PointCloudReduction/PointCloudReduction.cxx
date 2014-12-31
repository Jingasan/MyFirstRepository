/***** 3�����_�Q���̍팸 *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>		// �_�Q�̃f�[�^�^
#include <pcl/filters/voxel_grid.h>	// �{�N�Z���O���b�h�ɂ��_�Q�̊Ԉ���

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** �_�Q���̍팸�֐�( PointXYZRGB ) ***/
int reductPointCloudRGB(
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr reCloud,
	float boxelGridSizeX,
	float boxelGridSizeY,
	float boxelGridSizeZ
){

	/* �{�N�Z���O���b�h�ɂ��_�E���T���v�����O */
	cout << ">>> �_�Q���̍팸��" << "\r";
	pcl::VoxelGrid< pcl::PointXYZRGB > bg;
	bg.setInputCloud( cloud );	// �팸�O��3�����_�Q
	bg.setLeafSize( boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	bg.filter( *reCloud );		// �팸���3�����_�Q
	cout << ">>> �_�Q���̍팸 OK" << endl;

	return 0;
}

/*** �_�Q���̍팸�֐�( PointXYZ ) ***/
int reductPointCloud(
	pcl::PointCloud< pcl::PointXYZ >::Ptr cloud,
	pcl::PointCloud< pcl::PointXYZ >::Ptr reCloud,
	float boxelGridSizeX,
	float boxelGridSizeY,
	float boxelGridSizeZ
){

	/* �{�N�Z���O���b�h�ɂ��_�E���T���v�����O */
	cout << ">>> �_�Q���̍팸��" << "\r";
	pcl::VoxelGrid< pcl::PointXYZ > bg;
	bg.setInputCloud( cloud );	// �팸�O��3�����_�Q
	bg.setLeafSize( boxelGridSizeX, boxelGridSizeY, boxelGridSizeZ );
	bg.filter( *reCloud );		// �팸���3�����_�Q
	cout << ">>> �_�Q���̍팸 OK" << endl;

	return 0;
}