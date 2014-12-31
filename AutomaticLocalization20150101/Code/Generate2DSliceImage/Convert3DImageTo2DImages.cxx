/***** 3次元ボクセルデータの2次元スライス画像への変換 *****/

// 【 コマンドライン引数 】
// 指定する3つのコマンドライン引数
// 1. 入力3次元ボリュームデータファイル名.拡張子
// 2. 出力画像名
// 3. png

// 例1：
// 1. Output/EdgeVolumeData/LiverID_0002V2U0.01L0.01.mha Output/2DSliceImages/LiverID_0002V2U0.01L0.01/LiverID_0002V2U0.01L0.01_ png

// 例2：
// Output/MultiResolutionalVolumeData/LiverID_0002_0.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_0/ png

// 例3：
// Output/MultiResolutionalVolumeData/LiverID_0002_1.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_1/ png

// 例4：
// Output/MultiResolutionalVolumeData/LiverID_0002_2.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_2/ png

// 例5：
// Output/MultiResolutionalVolumeData/LiverID_0002_3.mha Output/MultiResolutionalVolumeData/2DSliceImage/LiverID_0002_3/ png


// This example illustrates how to save an image using the ImageSeriesWriter.
// This class enables the saving of a 3D volume as a set of files containing one 2D slice per file.

/*** インクルードファイル ***/

/* ITK 4.5.2 */
#include "itkImage.h"
#include "itkImageFileReader.h"
#include "itkImageSeriesWriter.h"
#include "itkNumericSeriesFileNames.h"


/*** 名前空間の宣言 ***/

/* C++ */
using namespace std;


/*** メイン処理文 ***/
int main( int argc, char *argv[] ){
	

	/* コマンドライン引数読み込み失敗時 */
	if( argc < 4 ){
		cerr << "Usage: ImageReadImageSeriesWrite inputFile outputPrefix outputExtension" << endl;
		return EXIT_FAILURE;
	}
	
	// The type of the input image is declared here and it is used for declaring the type of the reader.
	// This will be a conventional 3D image reader.



	/*** 各種データ型、クラス型の定義 ***/

	/* 画像の型の定義 */
	typedef itk::Image< unsigned char, 3 > ImageType;		// 入力画像のピクセル型：unsigned char, 次元数：3
	typedef itk::Image< unsigned char, 2 > Image2DType;		// 出力画像のピクセル型：unsigned char, 次元数：2

	/* ファイルストリーム型の定義 */
	typedef itk::ImageFileReader< ImageType > ReaderType;					// ImageType 型の画像を読み込むファイルストリーム
	typedef itk::ImageSeriesWriter< ImageType, Image2DType > WriterType;	// ImageType 型の画像を Image2DType 型の画像に変換して書き出すファイルストリーム
	// The type of the series writer must be instantiated taking into account that
	// the input file is a 3D volume and the output files are 2D images.
	//  Additionally, the output of the reader is connected as input to the writer.

	/* 複数のファイルネーム作成用クラスの定義 */
	typedef itk::NumericSeriesFileNames NameGeneratorType;	// 2次元スライス画像の複数のファイルネームを生成するクラス
	// The writer requires a list of filenames to be generated.
	// This list can be produced with the help of the NumericSeriesFileNames class.

	/* メモリの確保 */
	ReaderType::Pointer reader = ReaderType::New();	// 読み込み専用ファイルストリーム型変数
	WriterType::Pointer writer = WriterType::New();	// 書き込み専用ファイルストリーム型変数
	NameGeneratorType::Pointer nameGenerator = NameGeneratorType::New();



	/* プログラム開始のコール */
	cout << "【 プログラム開始 】" << endl;


	/*** 入力ファイル名の指定 ***/
	reader->SetFileName( argv[ 1 ] );	// 入力3次元ボリュームデータのファイル名を設定



	/*** 入力3次元ボリュームデータのスライス ***/
	cout << ">> ボリュームデータのスライス開始" << "\r";

	/* 書き出し専用ファイルストリーム型変数に入力画像を設定し、2次元画像にスライス */
	writer->SetInput( reader->GetOutput() );

	//  The NumericSeriesFileNames class requires an input string in order
	//  to have a template for generating the filenames of all the output slices.
	//  Here we compose this string using a prefix taken from the command line
	//  arguments and adding the extension for PNG files.
	


	/*** スライス画像のファイルストリームの設定 ***/

	/* ファイルネームのコマンドライン引数からの読み込み */
	string format = argv[ 2 ];	// ファイルネーム
	format += "%03d.";			// ファイルの通し番号3桁
	format += argv[ 3 ];		// ファイルの拡張子

	/* ファイル名を NameGeneratorType 型クラスの変数に指定 */
	nameGenerator->SetSeriesFormat( format.c_str() );



	/*** 入力ファイルの読み込み ***/
	try{
		reader->Update();
    }

	/* 読み込み失敗時 */
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
	

	/*** 入力3次元ボリュームデータの通し番号や大きさの取得と設定 ***/

	/* 入力ファイルのプロパティの取得 */
	ImageType::ConstPointer inputImage = reader->GetOutput();				// 入力3次元ボリュームデータ
	ImageType::RegionType region = inputImage->GetLargestPossibleRegion();	// 領域の取得
	ImageType::IndexType start = region.GetIndex();							// 開始通し番号の取得
	ImageType::SizeType size = region.GetSize();							// 合計スライス枚数の取得
	// Now that the image has been read we can query its largest possible region
	// and recover information about the number of pixels along every dimension.
	
	/* スライス番号の格納 */
	const unsigned int firstSlice = start[ 2 ];					// スライス画像の通し番号の始まり：1
	const unsigned int lastSlice = start[ 2 ] + size[ 2 ] - 1;	// スライス画像の通し番号の終わり：開始番号 + 合計スライス枚数 - 1
	
	/* スライス画像の通し番号の設定 */
	nameGenerator->SetStartIndex( firstSlice );	// スライス画像の開始番号：1
	nameGenerator->SetEndIndex( lastSlice );	// スライス画像の終了番号
	nameGenerator->SetIncrementIndex( 1 );		// インクリメント数：1
	// With this information we can find the number that will identify the first
	// and last slices of the 3D data set. This numerical values are then passed to
	// the filenames generator object that will compose the names of the files
	// where the slices are going to be stored.

	

	/*** 結果の出力 ***/

	/* 書き出し専用ファイルストリーム型変数にスライス画像のファイル名のリストを設定 */
	writer->SetFileNames( nameGenerator->GetFileNames() );
	
	/* 処理結果となる2次元スライス画像群の出力 */
	try{
		writer->Update();
    }

	/* 出力失敗時 */
	catch( itk::ExceptionObject & excp ){
		cerr << "Exception thrown while reading the image" << endl;
		cerr << excp << endl;
	}


	/* プログラム終了のコール */
	cout << ">> ボリュームデータのスライス終了" << endl;
	cout << "【 プログラム終了 】" << endl;


	// Finally we trigger the execution of the pipeline with the Update() method on the writer.
	// At this point the slices of the image will be saved in
	// individual files containing a single slice per file.
	// The filenames used for these slices are those produced by the filenames generator.

	// Note that by saving data into isolated slices we are losing information
	// that may be significant for medical applications, such as the interslice spacing in millimeters.
	
	return EXIT_SUCCESS;
}
