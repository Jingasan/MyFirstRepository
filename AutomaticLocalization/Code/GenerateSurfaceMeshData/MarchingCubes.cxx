/***** MarchingCubes �@�ɂ��\�ʒ��o *****/
// ���͂ƂȂ�}�X�N�{�����[���f�[�^���烂�f���̕\�ʂ����b�V���f�[�^�Ƃ��ďo�͂���
// ���b�V���f�[�^�͒��_��3�������W�Q�ƃ|���S�����\�����郁�b�V����ID�ԍ��Q�ō\�������

// �R�}���h���C������
// 1. ���̓{�����[���f�[�^��.mha
// 2. �o�̓��b�V���f�[�^��.vtk
// 3. ���̗̈敔���̉�f�l

// ��1�F
// Output/MultiResolutionalVolumeData/LiverID_0002_0.mha Output/MeshData/LiverID_0002/LiverID_0002_0.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_1.mha Output/MeshData/LiverID_0002/LiverID_0002_1.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_2.mha Output/MeshData/LiverID_0002/LiverID_0002_2.vtk 1
// Output/MultiResolutionalVolumeData/LiverID_0002_3.mha Output/MeshData/LiverID_0002/LiverID_0002_3.vtk 1


/*** �C���N���[�h ***/

/* C++ */
#include <iostream>

/* ITK 4.5.2 */
#include "itkImageFileReader.h"
#include "itkBinaryMask3DMeshSource.h"
#include "itkImage.h"
#include "itkObject.h"
#include "itkVTKPolyDataWriter.h"

/* �ݒ�w�b�_�t�@�C�� */
#include "Configuration.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������� ***/
int main( int argc, char * argv[] ){
	
	
	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 3 ){
		cerr << "Usage: IsoSurfaceExtraction inputImageFile outputMeshFile objectValue " << endl;
		return EXIT_FAILURE;
	}


	/* �v���O�����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;


	/*** �e��f�[�^�^�A�N���X�^�̒�` ***/

	/* �f�[�^�^�̒�` */
	const unsigned int Dimension = 3;						// ���͉摜�f�[�^�̎������̎w��
	typedef unsigned char PixelType;						// ���͉摜�̃s�N�Z���̃f�[�^�^�Funsigned char

	/* �摜�^�A���b�V���^�̒�` */
	typedef itk::Image< PixelType, Dimension > ImageType;	// �摜�^�Funsigned char, �������F3
	typedef itk::Mesh< double > MeshType;					// ���b�V���^�Fdouble

	/* �t�@�C���X�g���[���^�̒�` */
	typedef itk::ImageFileReader< ImageType > ReaderType;	// ImageType �^�̉摜��ǂݍ��ރt�@�C���X�g���[��
	typedef itk::VTKPolyDataWriter< MeshType > WriterType;	// MeshType �^�̃��b�V�����������ރt�@�C���X�g���[��
	
	/* �ϐ��̒�`�ƃ������̊m�� */
	ReaderType::Pointer reader = ReaderType::New();	// �ǂݍ��ݐ�p�t�@�C���X�g���[���^�ϐ�
	WriterType::Pointer writer = WriterType::New();	// �������ݐ�p�t�@�C���X�g���[���^�ϐ�

	/* �t�@�C�����̎w�� */
	reader->SetFileName( argv[ 1 ] );	// �ǂݍ��ݐ�p�t�@�C���X�g���[���^�ϐ��ɓ��̓t�@�C�������w��
	writer->SetFileName( argv[ 2 ] );


	/*** ���b�V�����Ώۂ̃{�����[���f�[�^�̓ǂݍ��� ***/

	/* �t�@�C���̓ǂݍ��� */
	try{
		reader->Update();
	}
	catch( itk::ExceptionObject & exp ){
		cerr << "Exception thrown while reading the input file " << endl;
		cerr << exp << endl;
		return EXIT_FAILURE;
	}
	

	/*** ���̓f�[�^���̎擾 ***/

	/* ���͉摜�̎擾 */
	ImageType::Pointer image = reader->GetOutput();
	
	/* ���̓{�����[���f�[�^�̃v���p�e�B�̃R�}���h���C���o�� */
	cout << endl << ">> ���̓f�[�^���" << endl;
	cout << "1voxel�� X �����T�C�Y�F" << image->GetSpacing().GetElement( 0 ) << endl;
	cout << "1voxel�� Y �����T�C�Y�F" << image->GetSpacing().GetElement( 1 ) << endl;
	cout << "1voxel�� Z �����T�C�Y�F" << image->GetSpacing().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ X ���W�F" << image->GetOrigin().GetElement( 0 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Y ���W�F" << image->GetOrigin().GetElement( 1 ) << endl;
	cout << "�{�����[���f�[�^�̌��_ Z ���W�F" << image->GetOrigin().GetElement( 2 ) << endl;
	cout << "�{�����[���f�[�^�� X �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 0 ) << endl;
	cout << "�{�����[���f�[�^�� Y �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 1 ) << endl;
	cout << "�{�����[���f�[�^�� Z �����T�C�Y�F" << image->GetLargestPossibleRegion().GetSize( 2 ) << endl << endl;


	/*** ���b�V���� ***/
	cout << "�y MarchingCubes �@ �ɂ�郁�b�V���� �z" << endl;

	/* BinaryMask3DMeshSource �N���X�̒�` */
	typedef itk::BinaryMask3DMeshSource< ImageType, MeshType > MeshSourceType;	// �{�����[���f�[�^�̃��b�V����

	/* �������̊m�� */
	MeshSourceType::Pointer meshSource = MeshSourceType::New();
	
	/* �}�X�N�{�����[���f�[�^�ɂ����镨�̗̈敔���̋P�x�l�̎擾 */
	const PixelType objectValue = static_cast< PixelType >( atof( argv[ 3 ] ) );
	
	/* �\�ʒ��o�t�B���^ */
	meshSource->SetObjectValue( objectValue );		// �}�X�N�{�����[���f�[�^�̕��̗̈敔���̋P�x�l���w��
	meshSource->SetInput( reader->GetOutput() );	// ���͂����}�X�N�{�����[���f�[�^���w��

	/* �}�X�N�{�����[���f�[�^�� MarchingCubes �@��p�������b�V���� */
	cout << ">> ���b�V��������" << "\r";
	try{
		meshSource->Update();
	}
	catch( itk::ExceptionObject & exp ){
		cerr << "Exception thrown during Update() " << endl;
		cerr << exp << endl;
		return EXIT_FAILURE;
	}
	cout << ">> ���b�V����������" << endl;


	/*** �o�̓f�[�^���̎擾 ***/
	cout << endl;
	cout << ">> �o�̓f�[�^���̏o��" << endl;
	cout << "���b�V�����_���F" << meshSource->GetNumberOfNodes() << endl;	// ���_��( Nodes )
	cout << "�|���S�����F" << meshSource->GetNumberOfCells() << endl;		// �|���S����( Cells )
	cout << endl;

	
	/*** ���b�V���f�[�^�̏o�� ***/
	cout << ">> ���b�V���o�͒�" << "\r";

	/* MarchingCubes �@�ɂ�蓾��ꂽ���b�V�����o�͂Ɏw�� */
	writer->SetInput( meshSource->GetOutput() );
	
	/* ���b�V���̏o�� */
	writer->Write();


	/* �v���O�����I���̃R�[�� */
	cout << ">> ���b�V���o�͊���" << endl;
	cout << "�y �v���O�����I�� �z" << endl;


	return EXIT_SUCCESS;
}
