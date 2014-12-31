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
#include <pcl/visualization/cloud_viewer.h>	// �_�Q�̉���

/* �ݒ�p�w�b�_�t�@�C�� */
#include "Configuration.h"
#include "FunctionDefinition.h"


/*** 3�����_�Q�� Viewer �\��( PointXYZRGB ) ***/
int showPointCloudRGB( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud ){
	
	/* �_�Q�� Viewer �\�� */
	pcl::visualization::CloudViewer viewer( "PointCloud XYZRGB" );	// Viewer ���̐ݒ�
	viewer.showCloud( cloud );		// �_�Q�̕\��
	while( !viewer.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v	

	return 0;
}


/*** 3�����_�Q�� Viewer �\��( PointXYZ ) ***/
int showPointCloud( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud ){
	
	/* �_�Q�� Viewer �\�� */
	pcl::visualization::CloudViewer viewer( "PointCloud XYZ" );	// Viewer ���̐ݒ�
	viewer.showCloud( cloud );		// �_�Q�̕\��
	while( !viewer.wasStopped() ){}	// Viewer ��������܂Ŗ������[�v 

	return 0;
}

