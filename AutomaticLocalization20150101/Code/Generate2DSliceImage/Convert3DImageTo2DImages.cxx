/***** 3�����{�N�Z���f�[�^��2�����X���C�X�摜�ւ̕ϊ� *****/

// �y �R�}���h���C������ �z
// �w�肷��3�̃R�}���h���C������
// 1. ����3�����{�����[���f�[�^�t�@�C����.�g���q
// 2. �o�͉摜��
// 3. png

// ��1�F
// 1. Output/EdgeVolumeData/LiverID_0002V2U0.01L0.01.mha Output/2DSliceImages/LiverID_0002V2U0.01L0.01/LiverID_0002V2U0.01L0.01_ png

// ��2�F
// Output/MultiResolutionalVolumeData/LiverID_0002_0.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_0/ png

// ��3�F
// Output/MultiResolutionalVolumeData/LiverID_0002_1.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_1/ png

// ��4�F
// Output/MultiResolutionalVolumeData/LiverID_0002_2.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_2/ png

// ��5�F
// Output/MultiResolutionalVolumeData/LiverID_0002_3.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_3/ png


// This example illustrates how to save an image using the ImageSeriesWriter.
// This class enables the saving of a 3D volume as a set of files containing one 2D slice per file.

/*** �C���N���[�h�t�@�C�� ***/

/* ITK 4.5.2 */
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageSeriesWriter.h"
#include "itkNumericSeriesFileNames.h"


/*** ���O��Ԃ̐錾 ***/

/* C++ */
using namespace std;


/*** ���C�������� ***/
int main( int argc, char *argv[] ){
	

	/* �R�}���h���C�������ǂݍ��ݎ��s�� */
	if( argc < 4 ){
		cerr << "Usage: ImageReadImageSeriesWrite inputFile outputPrefix outputExtension" << endl;
		return EXIT_FAILURE;
	}
	
	// The type of the input image is declared here and it is used for declaring the type of the reader.
	// This will be a conventional 3D image reader.



	/*** �e��f�[�^�^�A�N���X�^�̒�` ***/

	/* �摜�̌^�̒�` */
	typedef itk::Image< unsigned char, 3 > ImageType;		// ���͉摜�̃s�N�Z���^�Funsigned char, �������F3
	typedef itk::Image< unsigned char, 2 > Image2DType;		// �o�͉摜�̃s�N�Z���^�Funsigned char, �������F2

	/* �t�@�C���X�g���[���^�̒�` */
	typedef itk::ImageFileReader< ImageType > ReaderType;					// ImageType �^�̉摜��ǂݍ��ރt�@�C���X�g���[��
	typedef itk::ImageSeriesWriter< ImageType, Image2DType > WriterType;	// ImageType �^�̉摜�� Image2DType �^�̉摜�ɕϊ����ď����o���t�@�C���X�g���[��
	// The type of the series writer must be instantiated taking into account that
	// the input file is a 3D volume and the output files are 2D images.
	//  Additionally, the output of the reader is connected as input to the writer.

	/* �����̃t�@�C���l�[���쐬�p�N���X�̒�` */
	typedef itk::NumericSeriesFileNames NameGeneratorType;	// 2�����X���C�X�摜�̕����̃t�@�C���l�[���𐶐�����N���X
	// The writer requires a list of filenames to be generated.
	// This list can be produced with the help of the NumericSeriesFileNames class.

	/* �������̊m�� */
	ReaderType::Pointer reader = ReaderType::New();	// �ǂݍ��ݐ�p�t�@�C���X�g���[���^�ϐ�
	WriterType::Pointer writer = WriterType::New();	// �������ݐ�p�t�@�C���X�g���[���^�ϐ�
	NameGeneratorType::Pointer nameGenerator = NameGeneratorType::New();



	/* �v���O�����J�n�̃R�[�� */
	cout << "�y �v���O�����J�n �z" << endl;


	/*** ���̓t�@�C�����̎w�� ***/
	reader->SetFileName( argv[ 1 ] );	// ����3�����{�����[���f�[�^�̃t�@�C������ݒ�



	/*** ����3�����{�����[���f�[�^�̃X���C�X ***/
	cout << ">> �{�����[���f�[�^�̃X���C�X�J�n" << "\r";

	/* �����o����p�t�@�C���X�g���[���^�ϐ��ɓ��͉摜��ݒ肵�A2�����摜�ɃX���C�X */
	writer->SetInput( reader->GetOutput() );

	//  The NumericSeriesFileNames class requires an input string in order
	//  to have a template for generating the filenames of all the output slices.
	//  Here we compose this string using a prefix taken from the command line
	//  arguments and adding the extension for PNG files.
	


	/*** �X���C�X�摜�̃t�@�C���X�g���[���̐ݒ� ***/

	/* �t�@�C���l�[���̃R�}���h���C����������̓ǂݍ��� */
	string format = argv[ 2 ];	// �t�@�C���l�[��
	format += "%03d.";			// �t�@�C���̒ʂ��ԍ�3��
	format += argv[ 3 ];		// �t�@�C���̊g���q

	/* �t�@�C������ NameGeneratorType �^�N���X�̕ϐ��Ɏw�� */
	nameGenerator->SetSeriesFormat( format.c_str() );



	/*** ���̓t�@�C���̓ǂݍ��� ***/
	try{
		reader->Update();
    }

	/* �ǂݍ��ݎ��s�� */
	catch( itk::ExceptionObject & excp ){
		cerr << "Exception thrown while reading the image" << endl;
		cerr << excp << endl;
    }
	// The input string is going to be used for generating filenames by setting
	// the values of the first and last slice. This can be done by collecting
	// information from the input image. Note that before attempting to take any
	// image information from the reader, its execution must be triggered with
	// the invocation of the Update() method, and since this invocation
	// can potentially throw exceptions, it must be put inside a try / catch block.
	

	/*** ����3�����{�����[���f�[�^�̒ʂ��ԍ���傫���̎擾�Ɛݒ� ***/

	/* ���̓t�@�C���̃v���p�e�B�̎擾 */
	ImageType::ConstPointer inputImage = reader->GetOutput();				// ����3�����{�����[���f�[�^
	ImageType::RegionType region = inputImage->GetLargestPossibleRegion();	// �̈�̎擾
	ImageType::IndexType start = region.GetIndex();							// �J�n�ʂ��ԍ��̎擾
	ImageType::SizeType size = region.GetSize();							// ���v�X���C�X�����̎擾
	// Now that the image has been read we can query its largest possible region
	// and recover information about the number of pixels along every dimension.
	
	/* �X���C�X�ԍ��̊i�[ */
	const unsigned int firstSlice = start[ 2 ];					// �X���C�X�摜�̒ʂ��ԍ��̎n�܂�F1
	const unsigned int lastSlice = start[ 2 ] + size[ 2 ] - 1;	// �X���C�X�摜�̒ʂ��ԍ��̏I���F�J�n�ԍ� + ���v�X���C�X���� - 1
	
	/* �X���C�X�摜�̒ʂ��ԍ��̐ݒ� */
	nameGenerator->SetStartIndex( firstSlice );	// �X���C�X�摜�̊J�n�ԍ��F1
	nameGenerator->SetEndIndex( lastSlice );	// �X���C�X�摜�̏I���ԍ�
	nameGenerator->SetIncrementIndex( 1 );		// �C���N�������g���F1
	// With this information we can find the number that will identify the first
	// and last slices of the 3D data set. This numerical values are then passed to
	// the filenames generator object that will compose the names of the files
	// where the slices are going to be stored.

	

	/*** ���ʂ̏o�� ***/

	/* �����o����p�t�@�C���X�g���[���^�ϐ��ɃX���C�X�摜�̃t�@�C�����̃��X�g��ݒ� */
	writer->SetFileNames( nameGenerator->GetFileNames() );
	
	/* �������ʂƂȂ�2�����X���C�X�摜�Q�̏o�� */
	try{
		writer->Update();
    }

	/* �o�͎��s�� */
	catch( itk::ExceptionObject & excp ){
		cerr << "Exception thrown while reading the image" << endl;
		cerr << excp << endl;
	}


	/* �v���O�����I���̃R�[�� */
	cout << ">> �{�����[���f�[�^�̃X���C�X�I��" << endl;
	cout << "�y �v���O�����I�� �z" << endl;


	// Finally we trigger the execution of the pipeline with the Update() method on the writer.
	// At this point the slices of the image will be saved in
	// individual files containing a single slice per file.
	// The filenames used for these slices are those produced by the filenames generator.

	// Note that by saving data into isolated slices we are losing information
	// that may be significant for medical applications, such as the interslice spacing in millimeters.
	
	return EXIT_SUCCESS;
}
