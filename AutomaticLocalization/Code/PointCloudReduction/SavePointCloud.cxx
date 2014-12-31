/***** 3�����_�Q�̏o�͕ۑ� *****/


/*** �C���N���[�h�t�@�C�� ***/

/* PCL 1.6.0 */
#include <pcl/io/pcd_io.h>		// PCD �f�[�^���o��
#include <pcl/io/ply_io.h>		// PLY �f�[�^���o��
#include <pcl/io/vtk_io.h>		// VTK �f�[�^���o��
#include <pcl/common/io.h>		// ���o��
#include <pcl/point_types.h>	// �_�Q�f�[�^�^

/* C++ */
#include <iostream>

/* �쐬�w�b�_�t�@�C�� */
#include "FunctionDefinition.h"	// �֐��̒�`


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** 3�����_�Q�� PCDData �o�͕ۑ�( PointXYZRGB ) ***/
int savePointCloudRGBtoPCD( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* outputFileName ){
	
	/* �_�Q( PCD �f�[�^ )�̏������� */
	cout << ">>> PCD �f�[�^( PointXYZRGB )�̏o�͕ۑ���" << "\r";
	if( pcl::io::savePCDFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̏������ݎ��s" << endl; return 1; }
	cout << ">>> PCD �f�[�^( PointXYZRGB )�̏o�͕ۑ� OK" << endl;

	return 0;
}


/*** 3�����_�Q�� PCDData �o�͕ۑ�( PointXYZRGB ) ***/
int savePointCloudtoPCD( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* outputFileName ){
	
	/* �_�Q( PCD �f�[�^ )�̏������� */
	cout << ">>> PCD �f�[�^( PointXYZ )�̏o�͕ۑ���" << "\r";
	if( pcl::io::savePCDFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PCD �f�[�^�̏������ݎ��s" << endl; return 1; }
	cout << ">>> PCD �f�[�^( PointXYZ )�̏o�͕ۑ� OK" << endl;

	return 0;
}


/*** 3�����_�Q�� PLYData �o�͕ۑ�( PointXYZRGB ) ***/
int savePointCloudRGBtoPLY( pcl::PointCloud< pcl::PointXYZRGB >::Ptr cloud, const char* outputFileName ){
	
	/* �_�Q( PLY �f�[�^ )�̏������� */
	cout << ">>> PLY �f�[�^( PointXYZRGB )�̏o�͕ۑ���" << "\r";
	if( pcl::io::savePLYFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PLY �f�[�^�̏������ݎ��s" << endl; return 1; }
	cout << ">>> PLY �f�[�^( PointXYZRGB )�̏o�͕ۑ� OK" << endl;

	return 0;
}


/*** 3�����_�Q�� PLYData �o�͕ۑ�( PointXYZ ) ***/
int savePointCloudtoPLY( pcl::PointCloud< pcl::PointXYZ >::Ptr cloud, const char* outputFileName ){
	
	/* �_�Q( PLY �f�[�^ )�̏������� */
	cout << ">>> PLY �f�[�^( PointXYZ )�̏o�͕ۑ���" << "\r";
	if( pcl::io::savePLYFileASCII( outputFileName, *cloud ) == -1 ){ cout << "Caution!: PLY �f�[�^�̏������ݎ��s" << endl; return 1; }
	cout << ">>> PLY �f�[�^( PointXYZ )�̏o�͕ۑ� OK" << endl;

	return 0;
}