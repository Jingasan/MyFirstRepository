/***** 3�����_�Q�̕\�� *****/

// �y Viewer �̑�����@ �z
// �E���h���b�O - ���_�̉�]
// �EShift + ���h���b�O - ���_�̕��s�ړ�
// �ECtrl + ���h���b�O - ��ʏ�̉�]
// �E�E�h���b�O - �Y�[��
// �Eg�F���W���[�̕\��
// �Ej�F�X�N���[���V���b�g�̕ۑ�


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/point_types.h>				// �_�Q�̃f�[�^�^
#include <pcl/visualization/cloud_viewer.h>	// �_�Q�̉���

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** 3�����_�Q�� Viewer �\��( PointXYZRGB ) ***/
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, int i ){
	
	/* Viewer ���̐ݒ� */
	char ViewerName[ 128 ];
	sprintf( ViewerName, "PointCloud XYZRGB ID:%d", i );

	/* �_�Q�� Viewer �\�� */
	pcl::visualization::CloudViewer viewer( ViewerName );	// Viewer ���̐ݒ�
	viewer.showCloud( cloud );		// �_�Q�̕\��
	while( !viewer.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v	

	return 0;
}


/*** 3�����_�Q�� Viewer �\��( PointXYZ ) ***/
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, int i ){
	
	/* Viewer ���̐ݒ� */
	char ViewerName[ 128 ];
	sprintf( ViewerName, "PointCloud XYZ ID:%d", i );

	/* �_�Q�� Viewer �\�� */
	pcl::visualization::CloudViewer viewer( ViewerName );	// Viewer ���̐ݒ�
	viewer.showCloud( cloud );		// �_�Q�̕\��
	while( !viewer.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v 

	return 0;
}