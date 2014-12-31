/***** 3���� Canny �G�b�W���o *****/

// �y �R�}���h���C������ �z
// 1. ���̓{�����[���f�[�^��.�g���q 
// 2. �o�̓{�����[���f�[�^��.�g���q
// 3. �o�͓_�Q�f�[�^��.ply
// 4. �o�͓_�Q�f�[�^��.pcd
// 5. ���U�l�F�K�E�V�A���t�B���^�̑��T�C�Y
// 6. �G�b�W�A���̏��臒l
// 7. �G�b�W�A���̉���臒l
// 8. �{�N�Z���O���b�h�T�C�Y

// �� 1�F
// Input/VolumeData/ID_0002.mhd Output/EdgeVolumeData/ID_0002_LiverV31U10L1.mha Output/EdgePointData/ID_0002_LiverV31U10L1.ply Output/EdgePointData/ID_0002_LiverV31U10L1.pcd 31 10 1

// �� 2�F
// Input/VolumeData/LiverID_0002.mhd Output/EdgeVolumeData/LiverID_0002V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002V5U0.01L0.01.pcd 5 0.01 0.01

// �� 3�F
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_0.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_0V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_1.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_1V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_2.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_2V5U0.01L0.01.pcd 5 0.01 0.01
// Output/MultiResolutionalVolumeData/LiverID_0002/LiverID_0002_3.mha Output/EdgeVolumeData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.mha Output/EdgePointData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.ply Output/EdgePointData/LiverID_0002/LiverID_0002_3V5U0.01L0.01.pcd 5 0.01 0.01

// �� 4�F
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_0.mha Output/EdgeVolumeData/ID_0002/ID_0002_0V31U10L1.mha Output/EdgePointData/ID_0002/ID_0002_0V31U10L1.ply Output/EdgePointData/ID_0002/ID_0002_0V31U10L1.pcd 31 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_1.mha Output/EdgeVolumeData/ID_0002/ID_0002_1V16U10L1.mha Output/EdgePointData/ID_0002/ID_0002_1V16U10L1.ply Output/EdgePointData/ID_0002/ID_0002_1V16U10L1.pcd 16 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_2.mha Output/EdgeVolumeData/ID_0002/ID_0002_2V9U10L1.mha Output/EdgePointData/ID_0002/ID_0002_2V9U10L1.ply Output/EdgePointData/ID_0002/ID_0002_2V9U10L1.pcd 9 10 1
// Output/MultiResolutionalVolumeData/ID_0002/ID_0002_3.mha Output/EdgeVolumeData/ID_0002/ID_0002_3V5U10L1.mha Output/EdgePointData/ID_0002/ID_0002_3V5U10L1.ply Output/EdgePointData/ID_0002/ID_0002_3V5U10L1.pcd 5 10 1


//  This example introduces the use of the CannyEdgeDetectionImageFilter.
//  This filter is widely used for edge detection
//  since it is the optimal solution satisfying the constraints of good sensitivity, localization and noise robustness.


/*** �C���N���[�h�t�@�C�� ***/

/* ITK 4.5.2 */
#include "itkImage.h"							// �摜�f�[�^�^
#include "itkImageFileReader.h"					// �摜�̏�������
#include "itkImageFileWriter.h"					// �摜�̓ǂݍ���
#include "itkCastImageFilter.h"					// �摜�̃f�[�^�^�̕ϊ�
#include "itkRescaleIntensityImageFilter.h"		// �s�N�Z���l�̃��X�P�[��
#include "itkCannyEdgeDetectionImageFilter.h"	// CannyEdgeDetection �t�B���^
#include "itkImageRegionIteratorWithIndex.h"	// Iterator

/* PCL 1.6.0 */
#include <pcl/point_types.h>	// �_�Q�̃f�[�^�^

/* C */
#include <windows.h>	// �������Ԍv��

/* �쐬�w�b�_�t�@�C�� */
#include "Configuration.h"		// �ݒ�w�b�_�t�@�C��
#include "FunctionDefinition.h"	// �֐��̒�`


/*** ���O��Ԃ̐錾 ***/
using namespace std;


/*** ���C�������� ***/
int main( int argc, char* argv[] ){


	/* �v���O�����J�n�̃R�[�� */
	DWORD wholeTimeStart = GetTickCount();	// �����S�̂̎��Ԍv���J�n
	cout << "�y �v���O�����J�n �z" << endl;

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 5 ){
		cerr << "Caution! : �R�}���h���C�������G���[" << endl;
		cerr << "Usage: " << endl;
		cerr << argv[ 0 ] << endl;
		cerr << "1. InputImage" << endl;
		cerr << "2. OutputImage" << endl;
		cerr << "3. OutputPlyFile" << endl;
		cerr << "4. OutputPCDFile" << endl;
		cerr << "[ 5. variance ]" << endl;
		cerr << "[ 6. upperThreshold ]" << endl;
		cerr << "[ 7. lowerThreshold ]" << endl;
		cerr << "[ 8. voxelGridSize ]" << endl;
		system( "pause" );
		return EXIT_FAILURE;
	}



	/* �ϐ��̐錾 */
	const char * inputFilename  = argv[ 1 ];	// ���̓{�����[���f�[�^��
	const char * outputFilename1 = argv[ 2 ];	// �o�̓{�����[���f�[�^��
	const char * outputFilename2 = argv[ 3 ];	// �o�͓_�Q�f�[�^��
	const char * outputFilename3 = argv[ 4 ];	// �o�͓_�Q�f�[�^��
	
	/* �e��v���p�e�B�̏����l�ݒ� */
	float variance = VARIANCE;
	float upperThreshold = UPPERTHRESHOLD;
	float lowerThreshold = LOWERTHRESHOLD;
	float voxelGridSizeX = VOXELGRIDSIZE;
	float voxelGridSizeY = VOXELGRIDSIZE;
	float voxelGridSizeZ = VOXELGRIDSIZE;

	/* CannyEdgeDetection �t�B���^�ɑ΂���e��l�̓ǂݍ��� */
	if( argc > 5 ){ variance = atof( argv[ 5 ] ); }			// ���U�̒l
	if( argc > 6 ){ upperThreshold = atof( argv[ 6 ] ); }	// Canny �t�B���^�̏��臒l
	if( argc > 7 ){ lowerThreshold = atof( argv[ 7 ] ); }	// Canny �t�B���^�̉���臒l
	if( argc > 8 ){
		voxelGridSizeX = atof( argv[ 8 ] );	// �{�N�Z���O���b�h X �T�C�Y[�P��:m]
		voxelGridSizeY = atof( argv[ 8 ] );	// �{�N�Z���O���b�h Y �T�C�Y[�P��:m]
		voxelGridSizeZ = atof( argv[ 8 ] );	// �{�N�Z���O���b�h Z �T�C�Y[�P��:m]
	}

	/* ���̓v���p�e�B�l�̃R�}���h���C���o�� */
	cout << "Variance = " << variance << endl;
	cout << "UpperThreshold = " << upperThreshold << endl;
	cout << "LowerThreshold = " << lowerThreshold << endl;
	

	/*** �e��f�[�^�^�A�N���X�^�̒�` ***/

	/* �f�[�^�^�̒�` */
	typedef unsigned char CharPixelType;	// ���o�͉摜�̃s�N�Z���̃f�[�^�^�Funsigned char
	typedef double RealPixelType;			// CannyFilter �̏o�͌��ʉ摜�̃s�N�Z���̃f�[�^�^�Fdouble
	const unsigned int Dimension = 3;		// ���͉摜�f�[�^�̎������̎w��

	/* �摜�̌^�̒�` */
	typedef itk::Image< CharPixelType, Dimension > CharImageType;	// �s�N�Z���^�Funsigned char, �������F3
	typedef itk::Image< RealPixelType, Dimension > RealImageType;	// �s�N�Z���^�Fdouble, �������F3

	/* �t�@�C���X�g���[���^�̒�` */
	typedef itk::ImageFileReader< CharImageType > ReaderType;	// CharImageType �^�̉摜��ǂݍ��ރt�@�C���X�g���[��
	typedef itk::ImageFileWriter< CharImageType > WriterType;	// RealImageType �^�̉摜�������o���t�@�C���X�g���[�� 
	
	/* ���͉摜�̉�f�l�̃f�[�^�^( unsigned char )���t�B���^�����O�����ň����f�[�^�^( double )�ɕϊ����邽�߂̃N���X�̒�` */
	typedef itk::CastImageFilter< CharImageType, RealImageType > CastToRealFilterType;
	// This filter operates on image of pixel type float.
	// It is then necessary to cast the type of the input images that are usually of integer type.
	// The CastImageFilter is used here for that purpose.
	// Its image template parameters are defined for casting from the input type to the float type using for processing.
	
	/* ��f�l�̃X�P�[�����O�ϊ��p�N���X�̒�` */
	typedef itk::RescaleIntensityImageFilter< RealImageType, CharImageType > RescaleFilter;	// ��l�F0-1���O���[�X�P�[���l�F0-255�ɕϊ����邽�߂̃N���X

	/* CannyEdgeDetectionImageFilter �N���X�̒�` */
	typedef itk::CannyEdgeDetectionImageFilter< RealImageType, RealImageType > CannyFilter;
	// CannyEdgeDetectionImageFilter �͏����l���g�p����
	// The CannyEdgeDetectionImageFilter is instantiated using the float image type.


	/*** �I�u�W�F�N�g�ϐ��̐錾 ***/

	/* �������̊m�� */
	ReaderType::Pointer reader = ReaderType::New();						// �ǂݍ��ݐ�p�t�@�C���X�g���[���^�ϐ�
	WriterType::Pointer writer = WriterType::New();						// �������ݐ�p�t�@�C���X�g���[���^�ϐ�
	CastToRealFilterType::Pointer toReal = CastToRealFilterType::New();	// �f�[�^�^�̕ϊ��p�ϐ�
	RescaleFilter::Pointer rescale = RescaleFilter::New();				// ��f�l0-1 �� 0-255�̃X�P�[�����O�ϊ��p�ϐ�
	CannyFilter::Pointer cannyFilter = CannyFilter::New();				// CannyEdgeDetectionImageFilter �N���X�̕ϐ�
	

	/*** �{�����[���f�[�^�̓��� ***/

	/* �t�@�C�����̎w�� */
	cout << ">>> �{�����[���f�[�^���͒�" << "\r";
	reader->SetFileName( inputFilename );	// �ǂݍ��ݐ�p�t�@�C���X�g���[���^�ϐ��ɓ��̓t�@�C�������w��
	cout << ">>> �{�����[���f�[�^���� OK" << endl;
	//try{
		//reader->Update();
	//}
	//catch( itk::ExceptionObject & exp ){
		//cerr << "Exception thrown while reading the input file " << endl;
		//cerr << exp << endl;
		//return EXIT_FAILURE;
	//}


	/*** ��f�l�̃X�P�[�����O�ϊ��̐ݒ� ***/
	rescale->SetOutputMinimum( 0 );				// �����F0
	rescale->SetOutputMaximum( 255 );			// ����F255
	// The output of an edge filter is 0 or 1

	/*** ��f�l�̃f�[�^�^�̕ϊ� ***/
	toReal->SetInput( reader->GetOutput() );	// ���͉摜�t�@�C�����w��



	/*** 3�����G�b�W�̒��o ***/

	/* �{�����[���f�[�^�̃G�b�W���o�J�n�̃R�[�� */
	cout << "----- Canny �G�b�W���o�J�n -----" << endl;
	DWORD cannyTimeStart = GetTickCount();	// Canny �G�b�W���o�̎��Ԍv���J�n

	/* CannyEdgeDetectionImageFilter �ɑ΂���v���p�e�B�̐ݒ� */
	cannyFilter->SetInput( toReal->GetOutput() );		// CannyEdgeDetectionImageFilter �ɑ΂�����̓f�[�^
	cannyFilter->SetVariance( variance );				// Canny �t�B���^�̑傫��
	cannyFilter->SetUpperThreshold( upperThreshold );	// Canny �t�B���^�̏��臒l
	cannyFilter->SetLowerThreshold( lowerThreshold );	// Canny �t�B���^�̉���臒l
	
	/* ��f�l�̃X�P�[�����O�ϊ� */
	rescale->SetInput( cannyFilter->GetOutput() );	// CannyFilter �̏o�̓G�b�W�摜�̒l�� 0-1 �Ȃ̂ŁA0-255 �ɕϊ�������
	
	
	/*** �{�����[���f�[�^�̏o�� ***/
	cout << ">>> �{�����[���f�[�^�o�͒�" << "\r";

	/* �o�̓t�@�C�����̎w�� */
	writer->SetFileName( outputFilename1 );	// �������ݐ�p�t�@�C���X�g���[���^�ϐ��ɏo�̓t�@�C�������w��

	/* �����o����p�t�@�C���X�g���[���^�ϐ��Ƀ��X�P�[�����ꂽ�G�b�W���o�摜��ݒ� */
	writer->SetInput( rescale->GetOutput() );
	
	/* �������ʂƂȂ�3�����G�b�W�{�N�Z���f�[�^�̏o�� */
	try{
		writer->Update();
	}

	/* �o�͎��s�� */
	catch( itk::ExceptionObject & err ){
		cout << "ExceptionObject caught !" << endl;
		cout << err << endl;
		return EXIT_FAILURE;
	}
	cout << ">>> �{�����[���f�[�^�o�� OK" << endl;

	/* �{�����[���f�[�^�̃G�b�W���o�I���̃R�[�� */
	DWORD cannyTimeEnd = GetTickCount();	// Canny �G�b�W���o�̎��Ԍv���I��
	cout << "----- Canny �G�b�W���o�I�� -----" << endl;


	/*** 3�����_�Q�f�[�^�̍쐬 ***/

	/* 3�����_�Q�f�[�^�쐬�J�n�̃R�[�� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̍쐬�J�n -----" << endl;
	DWORD generateEdgePointCloudTimeStart = GetTickCount();

	/* �G�b�W���o�̌��ʉ摜�̎擾 */
	CharImageType::Pointer edgeImage = rescale->GetOutput();

	/* XYZ �����̉摜�T�C�Y�̎擾 */
	CharImageType::RegionType edgeImageRegion = edgeImage->GetLargestPossibleRegion();
	
	/* �G�b�W���o�摜�̎������A�J�n�ʂ��ԍ��A�T�C�Y�̏o�� */
	cout << edgeImageRegion << endl;

	/* �J��Ԃ������p�N���X ImageRegionIteratorWithIndex �̕ϐ��̐錾 */
	itk::ImageRegionIteratorWithIndex< CharImageType > imageIterator( edgeImage, edgeImageRegion );	// ���W�l�Ɖ�f�l�̗����𑀍삷��ۂɎg�p

	/* PointCloud �^�ϐ��̐錾�ƃ������̊m�� */
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr eCloud( new pcl::PointCloud< pcl::PointXYZRGB > );	// �G�b�W���3�����_�Q
	pcl::PointXYZRGB tmp;

	/* Iterator �̏����� */
	imageIterator.Begin();

	/* �w�肵���摜�̈�������[�v */
	while( !imageIterator.IsAtEnd() ){

		/* �G�b�W�_�̍��W�l�݂̂��擾 �c �G�b�W��̃s�N�Z���l�F255, ����ȊO�F0 */
		if( (int)imageIterator.Get() > 127 ){

			tmp.x = imageIterator.GetIndex().GetElement( 0 );	// x ���W�l
			tmp.y = imageIterator.GetIndex().GetElement( 1 );	// y ���W�l
			tmp.z = imageIterator.GetIndex().GetElement( 2 );	// z ���W�l
			tmp.r = 255; tmp.g = 0; tmp.b = 255;						// �_�Q�̐F�F���F�ɐݒ�
			eCloud->points.push_back( tmp );							// �_�Q�� PUSH �Ŋi�[���Ă���
			// �� ���W�l ( x, y, z ) �̔z�񂪕����ς܂ꂽ2�����z��̃C���[�W
		}

		/* ���[�v�̍X�V */
		++imageIterator;
	}

	/* �G�b�W�_�Q���Ȃ������ꍇ */
	if( eCloud->points.size() == 0 ){ cout << "Caution!: �G�b�W�_�Q�Ȃ�" << endl; return 0; }

	/* �_�Q�f�[�^���̎擾�ƃ��������̐ݒ� */
	eCloud->width = eCloud->points.size();
	eCloud->height = 1;

	/* �G�b�W�_�Q���̏o�� */
	DWORD generateEdgePointCloudTimeEnd = GetTickCount();
	cout << ">>> �G�b�W�_�Q��: " << eCloud->points.size() << " �_" << endl;

	/* 3�����_�Q�f�[�^�쐬�I���̃R�[�� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̍쐬�I�� -----" << endl;



	/* �v���O�����I���̃R�[�� */
	cout << "�y �v���O�����I�� �z" << endl;
	DWORD wholeTimeEnd = GetTickCount();	// �����S�̂̎��Ԍv���I��
	
	/* �e�폈�����Ԃ̃R�}���h���C���o�� */
	cout << endl << "�y ResultTime �z" << endl;
	cout << "CannyEdgeDetectionTime�F" << (double)( cannyTimeEnd - cannyTimeStart ) / 1000 << " sec." << endl;
	cout << "EdgePointCloudGenerationTime�F" << (double)( generateEdgePointCloudTimeEnd - generateEdgePointCloudTimeStart ) / 1000 << " sec." << endl;
	cout << "WholeTime�F" << (double)( wholeTimeEnd - wholeTimeStart ) / 1000 << " sec." << endl;


	/* 3�����_�Q�f�[�^�̕\�� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̕\�� -----" << endl;
	cout << ">>> �\����" << "\r";
	showPointCloudRGB( eCloud );
	cout << ">>> �\���I��" << endl;


#ifdef REDUCTION

	/* 3�����_�Q���̍팸 */
	cout << " �_�Q���팸��..." << "\r";
	pcl::PointCloud< pcl::PointXYZRGB >::Ptr reCloud( new pcl::PointCloud< pcl::PointXYZRGB > );
	reductPointCloudRGB( eCloud, reCloud, voxelGridSizeX, voxelGridSizeY, voxelGridSizeZ );
	cout << " �_�Q���팸����" << endl;
	cout << " >>> BeforeCloudSize�F " << eCloud->size() << " [points]" << endl;
	cout << " >>> AfterCloudSize�F " << reCloud->size() << " [points]" << endl;

	/* 3�����_�Q�f�[�^�̕\�� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̕\�� -----" << endl;
	showPointCloudRGB( reCloud );
	
	/* 3�����_�Q�f�[�^�̏o�͕ۑ� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̏o�͕ۑ� -----" << endl;
	savePointCloudRGBtoPLY( reCloud, outputFilename2 );
	savePointCloudRGBtoPCD( reCloud, outputFilename3 );
	
#else

	/* 3�����_�Q�f�[�^�̏o�͕ۑ� */
	cout << "----- 3�����G�b�W�_�Q�f�[�^�̏o�͕ۑ� -----" << endl;
	savePointCloudRGBtoPLY( eCloud, outputFilename2 );
	savePointCloudRGBtoPCD( eCloud, outputFilename3 );

#endif

	return EXIT_SUCCESS;
}


